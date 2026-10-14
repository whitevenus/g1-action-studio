"""纯轨迹计算：位置校验、差分、插值、统计和内容摘要。"""

import hashlib
import json
import math

from .constants import ARM_JOINT_COUNT, ARM_OFFSETS, DT


def finite_vector(values):
    if not isinstance(values, list) or len(values) != ARM_JOINT_COUNT:
        raise ValueError("每帧必须包含全部 14 个双臂关节")
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError("关节数据包含 NaN 或 Infinity")
    return result


def frame_digest(frames):
    canonical = json.dumps(frames, sort_keys=True, separators=(",", ":"),
                           allow_nan=False).encode("utf-8")
    return hashlib.sha256(canonical).hexdigest()


def interpolate_positions(start, end, ratio):
    return [a + (b - a) * ratio for a, b in zip(start, end)]


def smoothstep(ratio):
    """五次平滑曲线：两端速度和加速度为零。"""
    return ratio ** 3 * (10 + ratio * (-15 + 6 * ratio))


def hold_other_arm(frames, baseline, side):
    """只保留选中侧的动作，另一侧统一使用基线位置。"""
    selected = ARM_OFFSETS[side]
    return [{"t": frame["t"], "q": [value if i in selected else baseline[i]
             for i, value in enumerate(frame["q"])]} for frame in frames]


def derivatives(values):
    """单个关节：位置差分得到速度，再差分得到加速度。"""
    velocities = [(b - a) / DT for a, b in zip(values, values[1:])]
    accelerations = [(b - a) / DT for a, b in zip(velocities, velocities[1:])]
    return velocities, accelerations


def joint_motion(frames):
    """校验等间隔轨迹，按关节返回位置、速度、加速度三列。"""
    if not isinstance(frames, list) or len(frames) < 3:
        raise ValueError("轨迹至少需要三帧")
    if any(not isinstance(frame, dict) for frame in frames):
        raise ValueError("每帧必须是包含 t 和 q 的对象")
    points = [finite_vector(frame["q"]) for frame in frames]
    times = [float(frame["t"]) for frame in frames]
    if (not all(math.isfinite(t) for t in times) or abs(times[0]) > 1e-8
            or any(abs((b - a) - DT) > 1e-5 for a, b in zip(times, times[1:]))):
        raise ValueError("轨迹必须从 0 开始，以 50 Hz 等间隔采样")
    return [(values, *derivatives(values)) for values in zip(*points)]


def metrics(frames, motion=None):
    """motion 可复用已计算的逐关节差分，避免筛查时再算一遍。"""
    if motion is None:
        motion = joint_motion(frames)
    return {
        "duration_s": round(float(frames[-1]["t"]), 6),
        "frame_count": len(frames),
        "max_abs_velocity_rad_s": max(abs(v) for _, velocities, _ in motion for v in velocities),
        "max_abs_acceleration_rad_s2": max(abs(a) for _, _, accelerations in motion for a in accelerations),
        "max_excursion_from_start_rad": max(abs(v - values[0]) for values, _, _ in motion for v in values),
        "start_end_max_error_rad": max(abs(values[0] - values[-1]) for values, _, _ in motion),
        "first_step_max_velocity_rad_s": max(abs(velocities[0]) for _, velocities, _ in motion),
        "last_step_max_velocity_rad_s": max(abs(velocities[-1]) for _, velocities, _ in motion),
    }
