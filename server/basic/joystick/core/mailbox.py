"""每手单槽最新值信箱与冻结输入帧（joystick 控制热路径）。

``LatestValueCell`` 只依赖单属性赋值的原子性：每个槽保证读到完整的冻结帧；
帧与 policy/会话状态的配对由发布侧提交规则保证（store 与 policy 推进同一
``_policy_lock`` 临界区）。帧内只放不可变对象；一次批次一次落地（双手同批），
观测读者读与序号成对提交的镜像。
"""

import threading
from dataclasses import dataclass, replace
from typing import Dict, Optional, Tuple

from carm_teleop_input_contracts import (
    HAND_LEFT,
    HAND_RIGHT,
    ButtonSample,
    JointTargetSample,
    PoseSample,
    ReengageStatus,
    SourceLease,
    UnifiedHandSample,
)


class LatestValueCell:
    """单写者/单读者、最新值胜出的单槽信箱。

    写者：provider 回调线程（``store``）；读者：控制线程或 SSE 快照线程
    （``load``）。``clear`` 也只在上述写者/持有相关临界区的路径上调用。
    """

    __slots__ = ("_value",)

    def __init__(self) -> None:
        """初始化为空槽位。"""
        self._value = None

    def store(self, value) -> None:
        """发布一个完整的冻结值（单次属性赋值）。"""
        self._value = value

    def load(self):
        """读取当前槽位（可能是旧值，但永远是完整对象）。"""
        return self._value

    def clear(self) -> None:
        """清空槽位（仅在写者/持有相关临界区路径调用）。"""
        self._value = None


class PublicationSignal:
    """输入发布计数 + 唤醒：发布侧 ``notify``，控制线程 ``wait_newer`` 等待新发布。

    只是唤醒提示，不携带数据；读者醒来后仍从信箱读最新帧。内部锁是叶子锁，
    ``notify`` 不得在持有 gate（``_policy_lock``）时调用。
    """

    def __init__(self) -> None:
        """初始化发布序号为 0。"""
        self._condition = threading.Condition(threading.Lock())
        self._sequence = 0

    @property
    def sequence(self) -> int:
        """当前发布序号（单调递增）。"""
        with self._condition:
            return self._sequence

    def notify(self) -> None:
        """推进发布序号并唤醒全部等待者。"""
        with self._condition:
            self._sequence += 1
            self._condition.notify_all()

    def wait_newer(self, seen: int, timeout: float) -> int:
        """等待序号超过 ``seen``，最多 ``timeout`` 秒；返回当前序号。"""
        with self._condition:
            if self._sequence == seen and timeout > 0:
                self._condition.wait(timeout)
            return self._sequence


@dataclass(frozen=True)
class HandInputFrame:
    """
一只手的最新接受输入帧：``pose``/``button`` 独立接受（位姿无效帧不阻塞按钮刷新），
``control_sample`` 仅在位姿与按钮均被接受时更新。冻结对象整体替换。
"""

    hand: str
    pose: Optional[PoseSample]
    button: Optional[ButtonSample]
    control_sample: Optional[UnifiedHandSample]
    input_sequence: int
    lease: Optional[SourceLease]
    timestamp_monotonic: float


def _hand_frame(publication, hand: str):
    """从一次双手 publication 快照里取一手帧（``None`` 透传）。"""
    if publication is None:
        return None
    return publication.left if hand == HAND_LEFT else publication.right


def _with_hand_frame(publication, hand: str, frame):
    """置换一次双手 publication 快照中的单手帧（其余字段原样保留）。"""
    if hand == HAND_LEFT:
        return replace(publication, left=frame)
    return replace(publication, right=frame)


@dataclass(frozen=True)
class InputPublication:
    """
一次输入批次对外可见的双手同批冻结快照：写者 gate 临界区内一次 ``store``，
读者一次 ``load`` 即得双手同批的帧（单手更新批次里另一手携带上一 publication
的同一帧对象）。``batch_sequence``/``lease``/``timestamp_monotonic`` 为批次元数据。
"""

    left: Optional[HandInputFrame]
    right: Optional[HandInputFrame]
    batch_sequence: int
    lease: Optional[SourceLease]
    timestamp_monotonic: float

    def frame(self, hand: str) -> Optional[HandInputFrame]:
        """取本批次中某一手的输入帧（另一手帧不受影响）。"""
        return _hand_frame(self, hand)

    def with_frame(self, hand: str, frame: Optional[HandInputFrame]) -> "InputPublication":
        """置换单手帧，返回新的 publication（其余字段与另一手帧原样保留）。"""
        return _with_hand_frame(self, hand, frame)

    def without_display(self) -> "InputPublication":
        """清空双手帧的位姿/按钮显示槽位；无需清空时返回原对象（身份不变）。"""
        changed = False
        frames = {}
        for hand, frame in ((HAND_LEFT, self.left), (HAND_RIGHT, self.right)):
            if frame is not None and (frame.pose is not None or frame.button is not None):
                frame = replace(frame, pose=None, button=None)
                changed = True
            frames[hand] = frame
        if not changed:
            return self
        return replace(self, left=frames[HAND_LEFT], right=frames[HAND_RIGHT])


@dataclass(frozen=True)
class JointInputFrame:
    """一只手的最新关节目标帧（关节 provider 路径）。"""

    lease: Optional[SourceLease]
    batch_sequence: int
    target: Optional[JointTargetSample]
    received_monotonic: float


@dataclass(frozen=True)
class JointPublication:
    """一次关节批次的双手同批冻结快照（r5，P2）。"""

    left: Optional[JointInputFrame]
    right: Optional[JointInputFrame]
    batch_sequence: int
    lease: Optional[SourceLease]

    def frame(self, hand: str) -> Optional[JointInputFrame]:
        """取本批次中某一手的关节帧（另一手帧不受影响）。"""
        return _hand_frame(self, hand)

    def with_frame(self, hand: str, frame: Optional[JointInputFrame]) -> "JointPublication":
        """置换单手帧，返回新的 publication（诊断与内部改写用）。"""
        return _with_hand_frame(self, hand, frame)


def reengage_payload(status: Optional[ReengageStatus]) -> Optional[dict]:
    """信箱内的再接入冻结状态 → SSE/HTTP 普通 dict 投影（``None`` 透传）。"""
    return None if status is None else status.as_projection()


def hand_cells() -> Dict[str, LatestValueCell]:
    """为左右手各建一个空信箱（``_reengage_cells`` 与测试夹具共用）。"""
    return {hand: LatestValueCell() for hand in (HAND_LEFT, HAND_RIGHT)}
