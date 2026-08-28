import math

from ..runtime.dependencies import np, R


def project_input_to_arm(homogeneous_matrix, calibration_mode):
    """Project a neutral input pose into the arm coordinate system."""
    mode = getattr(calibration_mode, "value", calibration_mode)
    transform = np.eye(4)
    rotations = {
        0: [[0, 0, -1], [1, 0, 0], [0, 1, 0]],
        1: [[0, 0, 1], [-1, 0, 0], [0, 1, 0]],
        2: [[1, 0, 0], [0, 0, 1], [0, 1, 0]],
        3: [[-1, 0, 0], [0, 0, -1], [0, 1, 0]],
    }
    rotation = rotations.get(mode)
    if rotation is None:
        raise ValueError("Invalid calibration mode")
    transform[:3, :3] = rotation
    return transform @ homogeneous_matrix


def apply_pose_delta(pose, position_delta, quaternion_delta):
    """Apply a world-frame position and quaternion delta to one arm pose."""
    new_position = np.asarray(pose[:3], dtype=float) + np.asarray(
        position_delta, dtype=float
    )
    current_rotation = R.from_quat(pose[3:])
    delta_rotation = R.from_quat(quaternion_delta)
    new_quaternion = (delta_rotation * current_rotation).as_quat()
    return np.concatenate([new_position, new_quaternion])

def _is_valid_joint_position(position, expected_count: int) -> bool:
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
    """Scale a selected unit-quaternion branch without shortest-path flipping."""
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
    """Scale a pose delta while preserving its selected quaternion branch."""
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
