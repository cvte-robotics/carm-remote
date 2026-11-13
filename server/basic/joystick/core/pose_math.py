"""手柄位姿数学。

热路径中的位姿表示为四行四列的不可变元组：``Pose4 = (row0, row1,
row2, row3)``，每一行也是四元素元组。旋转矩阵使用三行三列元组。
这样可以保留标定矩阵含手性转换时的矩阵算法，同时避免为小数组调用
第三方数组运算库。
"""

import math
import numbers


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


def _normalize_quaternion(quaternion, error_message="quaternion must be finite and normalizable"):
    """归一化四元数，顺序为 ``(x, y, z, w)``。"""
    values = tuple(float(value) for value in quaternion)
    if len(values) != 4 or not all(math.isfinite(value) for value in values):
        raise ValueError(error_message)
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 1e-12:
        raise ValueError(error_message)
    return tuple(value / norm for value in values)


def _quaternion_multiply(left, right):
    """计算 Hamilton 四元数乘积。"""
    x1, y1, z1, w1 = _normalize_quaternion(left)
    x2, y2, z2, w2 = _normalize_quaternion(right)
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _quaternion_to_matrix(quaternion):
    """把归一化输入四元数转换为 3×3 旋转矩阵。"""
    x, y, z, w = _normalize_quaternion(quaternion)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def _matrix_multiply(left, right):
    """计算两个 3×3 矩阵的乘积。"""
    return tuple(
        tuple(
            left[row][0] * right[0][column]
            + left[row][1] * right[1][column]
            + left[row][2] * right[2][column]
            for column in range(3)
        )
        for row in range(3)
    )


def _matrix_transpose(matrix):
    """转置 3×3 矩阵。"""
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _matrix_vector_multiply(matrix, vector):
    """计算 3×3 矩阵与三维向量的乘积。"""
    return tuple(
        matrix[row][0] * vector[0]
        + matrix[row][1] * vector[1]
        + matrix[row][2] * vector[2]
        for row in range(3)
    )


def _matrix_to_quaternion(matrix):
    """按参考旋转实现的最大分量分支转换四元数。

    每个分支直接令所选分量为正，因此与参考实现的规范符号一致；相对旋转
    矩阵在本模块中始终是正交旋转。
    """
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    candidates = (
        1.0 + m00 + m11 + m22,
        1.0 + m00 - m11 - m22,
        1.0 - m00 + m11 - m22,
        1.0 - m00 - m11 + m22,
    )
    choice = max(range(4), key=lambda index: candidates[index])
    if choice == 0:
        s = math.sqrt(max(0.0, candidates[0])) * 2.0
        quaternion = (
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
            0.25 * s,
        )
    elif choice == 1:
        s = math.sqrt(max(0.0, candidates[1])) * 2.0
        quaternion = (
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
            (m21 - m12) / s,
        )
    elif choice == 2:
        s = math.sqrt(max(0.0, candidates[2])) * 2.0
        quaternion = (
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
            (m02 - m20) / s,
        )
    else:
        s = math.sqrt(max(0.0, candidates[3])) * 2.0
        quaternion = (
            (m02 + m20) / s,
            (m12 + m21) / s,
            0.25 * s,
            (m10 - m01) / s,
        )
    return _normalize_quaternion(quaternion)


def _pose4_from_parts(rotation, translation):
    """由旋转矩阵和位移构造不可变四维位姿。"""
    return (
        (rotation[0][0], rotation[0][1], rotation[0][2], translation[0]),
        (rotation[1][0], rotation[1][1], rotation[1][2], translation[1]),
        (rotation[2][0], rotation[2][1], rotation[2][2], translation[2]),
        (0.0, 0.0, 0.0, 1.0),
    )


def _pose4_parts(pose):
    """读取四维位姿的旋转和位移，兼容测试中的二维数组输入。"""
    rotation = tuple(tuple(float(pose[row][column]) for column in range(3)) for row in range(3))
    translation = tuple(float(pose[row][3]) for row in range(3))
    return rotation, translation


def _copy_pose4(pose):
    """复制位姿到本模块的不可变表示。"""
    rotation, translation = _pose4_parts(pose)
    return _pose4_from_parts(rotation, translation)


def legacy_mode_to_yaw(mode):
    """把历史 0-3 标定按钮映射为 yaw 角度（度）。"""
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
    """构建含手性转换（det=-1）的标定旋转矩阵。"""
    if isinstance(angle_deg, bool) or not isinstance(angle_deg, numbers.Real):
        raise ValueError("calibration angle must be an integer")
    angle = float(angle_deg)
    if not math.isfinite(angle) or not angle.is_integer() or not -180 <= angle <= 180:
        raise ValueError("calibration angle must be between -180 and 180 degrees")
    radians = math.radians(angle)
    c, s = math.cos(radians), math.sin(radians)
    return (
        (c, 0.0, -s),
        (s, 0.0, c),
        (0.0, 1.0, 0.0),
    )


def project_input_to_arm(homogeneous_matrix, calibration_mode):
    """把中性输入位姿投影到机械臂坐标系，返回 ``Pose4``。"""
    calibration = build_calibration_matrix(calibration_mode)
    rotation, translation = _pose4_parts(homogeneous_matrix)
    return _pose4_from_parts(
        _matrix_multiply(calibration, rotation),
        _matrix_vector_multiply(calibration, translation),
    )


def apply_pose_delta(pose, position_delta, quaternion_delta):
    """把 world-frame 位置增量与四元数增量施加到七分量 pose。"""
    position = tuple(float(value) for value in pose[:3])
    delta = tuple(float(value) for value in position_delta)
    new_position = tuple(position[index] + delta[index] for index in range(3))
    new_quaternion = _quaternion_multiply(quaternion_delta, pose[3:7])
    return new_position + new_quaternion


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
    quaternion = _normalize_quaternion(quaternion)
    half_angle = math.acos(max(-1.0, min(1.0, quaternion[3])))
    sin_half_angle = math.sin(half_angle)
    if abs(sin_half_angle) <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)

    scaled = (
        quaternion[0] * math.sin(float(scale) * half_angle) / sin_half_angle,
        quaternion[1] * math.sin(float(scale) * half_angle) / sin_half_angle,
        quaternion[2] * math.sin(float(scale) * half_angle) / sin_half_angle,
        math.cos(float(scale) * half_angle),
    )
    return _normalize_quaternion(scaled)


def _continuous_pose_diff_with_scale(
    baseline_pose,
    current_pose,
    scale,
    previous_relative_quaternion=None,
    ambiguity_atol=1e-9,
):
    """缩放 pose 增量，同时保持其已选定的四元数分支。"""
    baseline_rotation, baseline_translation = _pose4_parts(baseline_pose)
    current_rotation, current_translation = _pose4_parts(current_pose)
    pos_diff = tuple(
        (current_translation[index] - baseline_translation[index]) * scale
        for index in range(3)
    )
    relative_rotation = _matrix_multiply(
        current_rotation,
        _matrix_transpose(baseline_rotation),
    )
    relative_quaternion = _matrix_to_quaternion(relative_rotation)

    reference = (
        (0.0, 0.0, 0.0, 1.0)
        if previous_relative_quaternion is None
        else previous_relative_quaternion
    )
    reference = _normalize_quaternion(
        reference,
        "previous relative quaternion must be finite and normalizable",
    )

    sample_dot = sum(reference[index] * relative_quaternion[index] for index in range(4))
    if sample_dot < 0.0:
        relative_quaternion = tuple(-value for value in relative_quaternion)
        sample_dot = -sample_dot

    sample_is_ambiguous = math.isclose(
        sample_dot, 0.0, rel_tol=0.0, abs_tol=ambiguity_atol
    )
    segment_angle = 2.0 * math.acos(max(-1.0, min(1.0, relative_quaternion[3])))
    branch_is_undefined = (
        relative_quaternion[3] < 0.0
        and abs(math.sin(segment_angle / 2.0)) <= 1e-9
    )
    return (
        pos_diff,
        _scale_quaternion_branch(relative_quaternion, scale),
        relative_quaternion,
        float(segment_angle),
        sample_is_ambiguous or branch_is_undefined,
    )


def input_pose_to_arm(components, calibration):
    """把 7 分量 (x,y,z,qx,qy,qz,qw) 输入位姿投影到机械臂坐标系，返回 Pose4 元组。

    手臂与腰共用。
    """
    rotation = _quaternion_to_matrix(components[3:7])
    translation = tuple(float(value) for value in components[:3])
    return project_input_to_arm(_pose4_from_parts(rotation, translation), calibration)
