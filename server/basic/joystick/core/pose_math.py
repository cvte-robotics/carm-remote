"""手柄位姿数学：标定矩阵、yaw 换算与四元数分支缩放。"""

import math
import numbers

from ..runtime.dependencies import np, R


_LEGACY_CALIBRATION_YAW = {
    0: 90,
    1: -90,
    2: 0,
    3: 180,
}
_YAW_TO_LEGACY_CALIBRATION = {
    90: 0,
    -90: 1,
    0: 2,
    180: 3,
    -180: 3,
}


def legacy_mode_to_yaw(mode):
    """把历史 0-3 标定按钮映射为 yaw 角度（度）。

    Raises: ValueError mode 不是 0-3 的整数。
    """
    if isinstance(mode, bool) or not isinstance(mode, int) or mode not in _LEGACY_CALIBRATION_YAW:
        raise ValueError("calibration mode must be 0-3")
    return _LEGACY_CALIBRATION_YAW[mode]


def yaw_to_legacy_mode(angle_deg):
    """yaw 命中历史 cardinal 值时返回 0-3，否则返回 None。"""
    if isinstance(angle_deg, bool) or not isinstance(angle_deg, numbers.Real):
        return None
    angle = float(angle_deg)
    if not math.isfinite(angle) or not angle.is_integer():
        return None
    return _YAW_TO_LEGACY_CALIBRATION.get(int(angle))


def build_calibration_matrix(angle_deg):
    """构建标定旋转矩阵：0=前，+90=左，-90=右；含手性转换（det=-1）。

    Raises: ValueError 角度非整数或超出 -180..180。
    """
    if isinstance(angle_deg, bool) or not isinstance(angle_deg, numbers.Real):
        raise ValueError("calibration angle must be an integer")
    angle = float(angle_deg)
    if not math.isfinite(angle) or not angle.is_integer() or not -180 <= angle <= 180:
        raise ValueError("calibration angle must be between -180 and 180 degrees")
    radians = math.radians(angle)
    c, s = math.cos(radians), math.sin(radians)
    base = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ base


def project_input_to_arm(homogeneous_matrix, calibration_mode):
    """把中性（neutral）输入位姿投影到机械臂坐标系。"""
    transform = np.eye(4)
    transform[:3, :3] = build_calibration_matrix(calibration_mode)
    return transform @ homogeneous_matrix

def apply_pose_delta(pose, position_delta, quaternion_delta):
    """把 world-frame 位置增量与四元数增量施加到单臂 pose。"""
    new_position = np.asarray(pose[:3], dtype=float) + np.asarray(
        position_delta, dtype=float
    )
    current_rotation = R.from_quat(pose[3:])
    delta_rotation = R.from_quat(quaternion_delta)
    new_quaternion = (delta_rotation * current_rotation).as_quat()
    return np.concatenate([new_position, new_quaternion])

def _is_valid_joint_position(position, expected_count: int) -> bool:
    """判断关节位置是否为长度匹配的有限数值序列。"""
    return (
        isinstance(position, (list, tuple))
        and len(position) == expected_count
        and all(
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(value)
            for value in position
        )
    )
def _scale_quaternion_branch(quaternion, scale):
    """缩放已选定的单位四元数分支，不做最短路径翻转。"""
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.all(np.isfinite(quaternion)) or norm <= 1e-12:
        raise ValueError("quaternion must be finite and normalizable")

    quaternion = quaternion / norm
    half_angle = np.arccos(np.clip(quaternion[3], -1.0, 1.0))
    sin_half_angle = np.sin(half_angle)
    if abs(sin_half_angle) <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0])

    scaled = np.empty(4, dtype=float)
    scaled[:3] = (
        quaternion[:3]
        * np.sin(float(scale) * half_angle)
        / sin_half_angle
    )
    scaled[3] = np.cos(float(scale) * half_angle)
    return scaled / np.linalg.norm(scaled)
def _continuous_pose_diff_with_scale(
    baseline_pose,
    current_pose,
    scale,
    previous_relative_quaternion=None,
    ambiguity_atol=1e-9,
):
    """缩放 pose 增量，同时保持其已选定的四元数分支。

    Args: baseline_pose/current_pose 4x4 齐次位姿；scale 缩放系数；
        previous_relative_quaternion 上一帧相对四元数（用于分支选择）；
        ambiguity_atol 分支歧义判定容差。
    Returns: (位置增量, 缩放后的分支四元数, 归一化相对四元数, 段夹角弧度,
        是否存在分支歧义)。
    Raises: ValueError 参考四元数非有限或不可归一化。
    """
    pos_diff = (current_pose[:3, 3] - baseline_pose[:3, 3]) * scale
    relative_rotation = (
        current_pose[:3, :3] @ baseline_pose[:3, :3].T
    )
    relative_quaternion = R.from_matrix(relative_rotation).as_quat()

    reference = (
        np.array([0.0, 0.0, 0.0, 1.0])
        if previous_relative_quaternion is None
        else np.asarray(previous_relative_quaternion, dtype=float)
    )
    reference_norm = np.linalg.norm(reference)
    if not np.all(np.isfinite(reference)) or reference_norm <= 1e-12:
        raise ValueError("previous relative quaternion must be finite and normalizable")
    reference = reference / reference_norm

    sample_dot = float(np.dot(reference, relative_quaternion))
    if sample_dot < 0.0:
        relative_quaternion = -relative_quaternion
        sample_dot = -sample_dot

    sample_is_ambiguous = bool(
        np.isclose(sample_dot, 0.0, rtol=0.0, atol=ambiguity_atol)
    )
    segment_angle = 2.0 * np.arccos(
        np.clip(relative_quaternion[3], -1.0, 1.0)
    )
    branch_is_undefined = bool(
        relative_quaternion[3] < 0.0
        and abs(np.sin(segment_angle / 2.0)) <= 1e-9
    )
    return (
        pos_diff,
        _scale_quaternion_branch(relative_quaternion, scale),
        relative_quaternion,
        float(segment_angle),
        sample_is_ambiguous or branch_is_undefined,
    )
