"""旧式原始按键到功能结果的唯一适配实现，实例由每手连接生命周期持有。"""

from carm_teleop_input_contracts import ButtonSample, HandIntentSample


class LegacyHandIntentAdapter:
    def __init__(self) -> None:
        self.reset()

    def reset(self, home_sequence: int = 0) -> None:
        """新 lease 从零开始；同 lease 的传输重连保留累计序号。"""
        self.home_sequence = home_sequence
        self._home_pressed = False

    def adapt(self, button: ButtonSample) -> HandIntentSample:
        if button.btn_xa and not self._home_pressed:
            self.home_sequence += 1
        self._home_pressed = button.btn_xa
        return HandIntentSample(
            motion_engaged=button.grip > 100,
            rotation_locked=button.btn_yb,
            # 原始按键保持来源数值域供观测；功能结果截断到末端指令范围。
            end_effector=max(0, min(127, int(button.trigger))),
            home_sequence=self.home_sequence,
            timestamp=button.timestamp,
        )
