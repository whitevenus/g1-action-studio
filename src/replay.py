"""离线回放调度与数值筛查；不导入 SDK，不生成可发送的电机命令。"""

import math
from bisect import bisect_right
from pathlib import Path
import xml.etree.ElementTree as ET

from .constants import ARM_SLOTS, DEFAULT_MODEL, DT, JOINT_NAMES, PLAN_SCHEMA, PROFILE
from .trajectory import (finite_vector, frame_digest, hold_other_arm, interpolate_positions,
                         joint_motion, metrics, smoothstep)


def load_limits(path):
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as error:
        raise ValueError("关节模型 XML 无效") from error
    compiler = root.find("compiler")
    if compiler is None or compiler.get("angle") != "radian":
        raise ValueError("关节模型必须明确使用弧度")
    nodes = {node.get("name"): node for node in root.findall(".//joint")}
    limits = []
    for name in JOINT_NAMES:
        node = nodes.get(name + "_joint")
        values = [] if node is None else node.get("range", "").split()
        if len(values) != 2:
            raise ValueError("模型缺少关节范围：{}".format(name))
        low, high = map(float, values)
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError("模型关节范围无效：{}".format(name))
        limits.append((low, high))
    return limits


def _steps(seconds):
    if (type(seconds) not in (int, float) or not math.isfinite(seconds)
            or not DT <= seconds <= 120):
        raise ValueError("阶段时长须为 0.02–120 秒的有限数")
    steps = round(seconds / DT)
    if abs(seconds - steps * DT) > 1e-8:
        raise ValueError("阶段时长必须是 0.02 秒的整数倍")
    return steps


def retime_frames(frames, speed_scale, smooth=True):
    """沿原路径调整时间；单个动作平滑两端，组合动作保留已有衔接。"""
    metrics(frames)
    if (type(speed_scale) not in (int, float) or not math.isfinite(speed_scale)
            or not 0 < speed_scale <= 1):
        raise ValueError("平均播放速度比例须在 0–1 之间且不为零")
    duration = frames[-1]["t"]
    steps = math.ceil(duration / speed_scale / DT)
    if steps * DT > 120:
        raise ValueError("减速后的动作不得超过 120 秒")
    output = []
    for step in range(steps + 1):
        u = step / steps
        source_t = duration * (smoothstep(u) if smooth else u)
        position = source_t / DT
        left = min(int(position), len(frames) - 2)
        ratio = min(1.0, max(0.0, position - left))
        q = interpolate_positions(frames[left]["q"], frames[left + 1]["q"], ratio)
        output.append({"t": round(step * DT, 6), "q": q})
    output[0]["q"] = list(frames[0]["q"])
    output[-1]["q"] = list(frames[-1]["q"])
    return output


def build_schedule(frames, baseline, takeover=3.0, enter=5.0, return_seconds=5.0, release=2.0,
                   enter_motion=None, return_motion=None):
    """完整正常路径；返回显式基线。weight 仅是离线分析值。"""
    metrics(frames)
    baseline = finite_vector(baseline)
    durations = [_steps(value) for value in (takeover, enter, return_seconds, release)]
    if frames[-1]["t"] > 540:
        raise ValueError("动作序列时长不得超过 540 秒")
    output = [{"t": 0.0, "phase": "takeover", "q": list(baseline), "weight": 0.0}]
    phases = []

    def transition(name, start, end, steps, weight_start=1.0, weight_end=1.0):
        began = output[-1]["t"]
        for step in range(1, steps + 1):
            u = step / steps
            # 五次平滑曲线：两端一阶、二阶导数为零。
            blend = smoothstep(u)
            output.append({"t": round(len(output) * DT, 6), "phase": name,
                           "q": interpolate_positions(start, end, blend),
                           "weight": weight_start + (weight_end - weight_start) * blend})
        phases.append({"phase": name, "start_s": began, "end_s": output[-1]["t"]})

    def add_motion(name, motion, minimum_steps):
        began = output[-1]["t"]
        positions = [frame["q"] for frame in motion[1:]]
        positions += [motion[-1]["q"]] * max(0, minimum_steps - len(positions))
        for q in positions:
            output.append({"t": round(len(output) * DT, 6), "phase": name,
                           "q": list(q), "weight": 1.0})
        phases.append({"phase": name, "start_s": began, "end_s": output[-1]["t"]})

    transition("takeover", baseline, baseline, durations[0], 0.0, 1.0)
    if enter_motion is None:
        transition("enter", baseline, frames[0]["q"], durations[1])
    else:
        add_motion("enter", enter_motion, durations[1])
    began = output[-1]["t"]
    for frame in frames[1:]:
        output.append({"t": round(len(output) * DT, 6), "phase": "replay",
                       "q": list(frame["q"]), "weight": 1.0})
    phases.append({"phase": "replay", "start_s": began, "end_s": output[-1]["t"]})
    if return_motion is None:
        transition("return", frames[-1]["q"], baseline, durations[2])
    else:
        add_motion("return", return_motion, durations[2])
    transition("release", baseline, baseline, durations[3], 1.0, 0.0)
    return output, phases


def sample_schedule(frames, elapsed, times=None):
    """按单调时钟的相对秒数插值；开始前/结束后保持端点，避免依赖循环帧数。"""
    if not math.isfinite(elapsed):
        raise ValueError("播放时间必须有限")
    if times is None:
        times = [frame["t"] for frame in frames]
    right = bisect_right(times, elapsed)
    if right == 0 or right == len(frames):
        point = frames[0] if right == 0 else frames[-1]
        return {**point, "q": list(point["q"])}
    before, after = frames[right - 1:right + 1]
    ratio = (elapsed - before["t"]) / (after["t"] - before["t"])
    return {"t": elapsed, "phase": after["phase"] if ratio > 0 else before["phase"],
            "q": interpolate_positions(before["q"], after["q"], ratio),
            "weight": before["weight"] + (after["weight"] - before["weight"]) * ratio}


def screen_schedule(frames, limits, max_velocity, max_acceleration):
    for value in (max_velocity, max_acceleration):
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("离线速度和加速度阈值必须是正的有限数")
    if len(limits) != len(ARM_SLOTS):
        raise ValueError("必须提供全部 14 个关节范围")
    motion = joint_motion(frames)
    computed = metrics(frames, motion)
    violations = []
    per_joint = []
    for offset, (low, high) in enumerate(limits):
        values = [frame["q"][offset] for frame in frames]
        _, velocities, accelerations = motion[offset]
        per_joint.append({"slot": ARM_SLOTS[offset], "name": JOINT_NAMES[offset],
                          "range_rad": [low, high], "min_rad": min(values), "max_rad": max(values),
                          "max_abs_velocity_rad_s": max(map(abs, velocities)),
                          "max_abs_acceleration_rad_s2": max(map(abs, accelerations))})
        checks = (("position", values, 0, lambda v: v < low or v > high),
                  ("velocity", velocities, 1, lambda v: abs(v) > max_velocity),
                  ("acceleration", accelerations, 2, lambda v: abs(v) > max_acceleration))
        # 每个关节每类记录首个超限点；完整序列参与检查，包括阶段边界。
        for kind, series, shift, rejected in checks:
            for index, value in enumerate(series):
                if rejected(value):
                    frame = frames[index + shift]
                    violations.append({"slot": ARM_SLOTS[offset], "kind": kind,
                                       "t": frame["t"], "phase": frame["phase"], "value": value})
                    break
    return {"metrics": computed, "per_joint": per_joint, "violations": violations}


def create_plan(data, baseline, model_path, max_velocity, max_acceleration,
                takeover=3.0, enter=5.0, return_seconds=5.0, release=2.0,
                speed_scale=None, side=None, collision_check=False):
    """离线预览和现场规划共用；现场基线由只读反馈提供。"""
    q = finite_vector(baseline["q"])
    limits = load_limits(model_path)
    actions = data if isinstance(data, list) else [data]
    if not actions:
        raise ValueError("至少需要一个动作")
    if side is not None and side not in ("left", "right"):
        raise ValueError("回放侧必须为 left 或 right")
    if any(action["kind"] == "sequence" and side != action["side"] for action in actions):
        raise ValueError("组合动作只能按制作时的回放侧执行")
    motions = [action["frames"] if speed_scale is None else retime_frames(
        action["frames"], speed_scale, smooth=action["kind"] == "single") for action in actions]
    if side is not None:
        motions = [hold_other_arm(motion, q, side) for motion in motions]
    collision_model = None
    body_pose = None
    enter_motion = return_motion = None
    if collision_check or len(actions) > 1 or actions[0]["kind"] == "sequence":
        settings = next((action["collision"] for action in actions if action["kind"] == "sequence"),
                        {"body_q": [0.0] * 15, "clearance_m": 0.02})
        body_pose = list(baseline.get("body_q", settings["body_q"]))
        from .collision import CollisionModel, find_path, smooth_path
        collision_model = CollisionModel(body_pose)
        margin, seed = settings["clearance_m"], actions[0].get("seed", 0)
        motion = list(motions[0])
        for following in motions[1:]:
            path = find_path(collision_model, motion[-1]["q"], following[0]["q"], side, margin, seed)
            bridge = smooth_path(path, max_velocity, max_acceleration)
            for frame in bridge[1:] + following[1:]:
                motion.append({"t": round(len(motion) * DT, 6), "q": list(frame["q"])})
        enter_path = find_path(collision_model, q, motion[0]["q"], side, margin, seed)
        return_path = find_path(collision_model, motion[-1]["q"], q, side, margin, seed)
        enter_motion = smooth_path(enter_path, max_velocity, max_acceleration)
        return_motion = smooth_path(return_path, max_velocity, max_acceleration)
    else:
        motion = motions[0]
    frames, phases = build_schedule(motion, q, takeover, enter, return_seconds, release,
                                    enter_motion, return_motion)
    screening = screen_schedule(frames, limits, max_velocity, max_acceleration)
    collision = None if collision_model is None else collision_model.check_motion(frames, margin)
    return {"schema": PLAN_SCHEMA,
            "qualification": "offline_only_not_executable", "hardware_enabled": False,
            "status": "offline_numerical_rejected" if screening["violations"] else "offline_numerical_pass",
            "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS),
            "action_code": actions[0]["action_code"], "action_version": actions[0]["action_version"],
            "source_trajectory_sha256": actions[0]["trajectory_sha256"],
            "actions": [{"code": action["action_code"], "trajectory_sha256": action["trajectory_sha256"]}
                        for action in actions],
            "baseline": baseline, "model": {"file_name": Path(model_path).name,
                                           "limits_rad": [list(pair) for pair in limits]},
            "thresholds": {"max_velocity_rad_s": max_velocity, "max_acceleration_rad_s2": max_acceleration},
            "phases": phases, "frames": frames, "schedule_sha256": frame_digest(frames),
            "body_pose": body_pose, "collision": collision,
            **screening}
