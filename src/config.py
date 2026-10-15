"""参数校验与 JSON 读写；同一份快照检查，新文件禁止覆盖。"""

import json
import math
from pathlib import Path

from .constants import DT, PROFILE, PROFILE_SCHEMA


def read_json(path, *, encoding="utf-8-sig"):
    return json.loads(Path(path).read_text(encoding=encoding))


def dump_json(data, stream, *, compact=False, allow_nan=False):
    options = {"separators": (",", ":")} if compact else {"indent": 2}
    json.dump(data, stream, ensure_ascii=False, allow_nan=allow_nan, **options)
    stream.write("\n")


def write_json(data, output, *, compact=True, allow_nan=False):
    with Path(output).open("x", encoding="utf-8") as stream:
        dump_json(data, stream, compact=compact, allow_nan=allow_nan)


def _require_positive(data, keys, message):
    for key in keys:
        value = data.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(message.format(key))


def validate_teach_profile(data, target_stage="record"):
    required = ("hold_kp", "hold_kd", "hand_hold_kp", "hand_hold_kd")
    if target_stage != "hold":
        required += ("teach_kp", "teach_kd")
    if data.get("schema") != PROFILE_SCHEMA or data.get("target_profile") != PROFILE:
        raise ValueError("示教参数文件的格式或目标机型不匹配")
    _require_positive(data, required, "示教参数 {} 必须是正的有限数")
    if target_stage != "hold" and data["teach_kp"] >= data["hold_kp"]:
        raise ValueError("示教刚度须小于保持刚度")
    if target_stage != "hold" and data["teach_kd"] > data["hold_kd"]:
        raise ValueError("示教阻尼不得大于保持阻尼")
    if data["hold_kp"] > 60 or data["hold_kd"] > 3:
        raise ValueError("保持增益超过候选程序上限")
    if target_stage != "hold" and (data["teach_kp"] > 20 or data["teach_kd"] > 2):
        raise ValueError("示教增益超过候选程序上限")
    if target_stage != "hold":
        value = data.get("hand_teach_kp")
        if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value < data["hand_hold_kp"]:
            raise ValueError("手指示教刚度须为非负数且小于手指保持刚度")
        _require_positive(data, ("hand_teach_kd",), "示教参数 {} 必须是正的有限数")
        if data["hand_teach_kd"] > data["hand_hold_kd"]:
            raise ValueError("手指示教阻尼不得大于保持阻尼")
        required += ("hand_teach_kp", "hand_teach_kd")
    result = {key: float(data[key]) for key in required}
    if target_stage == "hold":
        result.update(teach_kp=result["hold_kp"], teach_kd=result["hold_kd"])
        result.update(hand_teach_kp=result["hand_hold_kp"], hand_teach_kd=result["hand_hold_kd"])
    return result


def load_teach_profile(path, target_stage="record"):
    return validate_teach_profile(read_json(path, encoding="utf-8"), target_stage)


def validate_replay_profile(profile):
    required = ("hold_kp", "hold_kd", "hand_hold_kp", "hand_hold_kd",
                "max_tracking_error_rad", "initial_pose_tolerance_rad", "return_tolerance_rad", "return_speed_rad_s",
                "state_timeout_s", "max_tick_gap_s", "settle_timeout_s", "recovery_release_s")
    if not isinstance(profile, dict):
        raise ValueError("回放参数必须是对象")
    optional = tuple(key for key in ("max_speed_rad_s", "max_offset_rad") if key in profile)
    _require_positive(profile, required + optional, "回放参数 {} 必须为正的有限数")
    if not DT <= profile["recovery_release_s"] <= 120 or not DT <= profile["settle_timeout_s"] <= 120:
        raise ValueError("恢复释放和回位等待时长须为 0.02–120 秒")
    if profile["max_tick_gap_s"] < DT:
        raise ValueError("控制周期超时阈值不得小于 0.02 秒")
    if profile["return_tolerance_rad"] > profile["max_tracking_error_rad"]:
        raise ValueError("回位容差不得大于跟踪误差阈值")
    if "max_speed_rad_s" in profile and profile["return_speed_rad_s"] > profile["max_speed_rad_s"]:
        raise ValueError("回位速度容差不得大于反馈速度阈值")
    return dict(profile)


def load_replay_profile(path):
    data = read_json(path, encoding="utf-8")
    validate_teach_profile(data, "hold")
    return validate_replay_profile(data)
