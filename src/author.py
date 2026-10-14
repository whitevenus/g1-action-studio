"""将观测到的 Arm7 状态离线整理为未验收的动作草稿。"""

import hashlib
import json
import math
import re
from bisect import bisect_right
from pathlib import Path

from .constants import ARM_SLOTS, DRAFT_SCHEMA, DT, JOINT_NAMES, PROFILE, RECORDING_SCHEMA
from .files import read_json, write_json as write_draft
from .trajectory import finite_vector, frame_digest, interpolate_positions, metrics


MAX_SOURCE_GAP = 0.10  # 拒绝状态采样缺口；这不是动作安全阈值。
CODE_PATTERN = re.compile(r"^[a-z][a-z0-9]*(?:[._][a-z0-9]+)*$")


def load_source(path, *, allow_synthetic=True):
    path = Path(path)
    raw = path.read_bytes()
    data = json.loads(raw.decode("utf-8-sig"))
    if not allow_synthetic and data.get("qualification") == "synthetic_example_not_for_hardware":
        raise ValueError("合成示例只供离线学习，不允许真机执行")
    if data.get("arm_slots") != list(ARM_SLOTS):
        raise ValueError("记录的关节槽位与 Arm7 J15–J28 不符")
    schema = data.get("schema")
    if schema == RECORDING_SCHEMA:
        if data.get("target_profile") != PROFILE:
            raise ValueError("目标设备配置不匹配")
        rows = data.get("samples")
        extract = lambda row: (row["t"], row["q"], row["dq"],
                               row["mode_machine"], row["mode_pr"])
    elif schema == "g1-preset-arm-readonly-trace-v1":
        rows = data.get("sampled_states")
        extract = lambda row: (row["elapsed_s"], row["arm_q"], row["arm_dq"],
                               row["mode_machine"], None)
    else:
        raise ValueError("不支持的录制格式")
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError("至少需要三帧状态")
    if data.get("invalid_messages", 0) != 0:
        raise ValueError("原始记录含无效帧，请先调查数据质量")
    samples = []
    last_time = float("-inf")
    for row in rows:
        t, q, dq, mode_machine, mode_pr = extract(row)
        t = float(t)
        if not math.isfinite(t) or t <= last_time or t < 0:
            raise ValueError("录制时间必须为严格递增的非负有限值")
        samples.append({"t": t, "q": finite_vector(q), "dq": finite_vector(dq),
                        "mode_machine": int(mode_machine),
                        "mode_pr": None if mode_pr is None else int(mode_pr)})
        last_time = t
    return schema, hashlib.sha256(raw).hexdigest(), samples


def _interpolate(samples, times, moment):
    right = bisect_right(times, moment)
    if right == 0 or (right == len(times) and moment > times[-1]):
        raise ValueError("裁剪时间超出录制范围")
    if right == len(times):
        return list(samples[-1]["q"])
    left = right - 1
    span = times[right] - times[left]
    if span > MAX_SOURCE_GAP:
        raise ValueError("录制中存在超过 0.1 秒的状态缺口")
    ratio = (moment - times[left]) / span
    return interpolate_positions(samples[left]["q"], samples[right]["q"], ratio)


def build_draft(source, start, end, code, version, name, *, allow_synthetic=True):
    """裁剪并重采样；start/end 为 None 时使用录制的完整范围。"""
    if not CODE_PATTERN.fullmatch(code) or len(code) > 120:
        raise ValueError("动作代码只能使用小写字母、数字、点和下划线")
    if type(version) is not int or version <= 0 or not name.strip():
        raise ValueError("动作版本和名称无效")
    schema, source_sha, samples = load_source(source, allow_synthetic=allow_synthetic)
    start = samples[0]["t"] if start is None else float(start)
    end = samples[-1]["t"] if end is None else float(end)
    if not math.isfinite(start) or not math.isfinite(end) or end - start < 0.4:
        raise ValueError("裁剪时间无效，动作至少需要 0.4 秒")
    times = [row["t"] for row in samples]
    if start < times[0] or end > times[-1]:
        raise ValueError("裁剪时间超出录制范围")
    steps = int(math.floor((end - start) / DT + 1e-9))
    if steps < 2:
        raise ValueError("轨迹太短")
    # 保留裁剪区间两端相邻的采样帧，供插值使用。
    first = max(0, bisect_right(times, start) - 1)
    last = min(len(samples) - 1, bisect_right(times, start + steps * DT))
    selected = samples[first:last + 1]
    modes = {(row["mode_machine"], row["mode_pr"]) for row in selected}
    if len(modes) != 1:
        raise ValueError("录制区间内设备模式发生变化")
    frames = [{"t": round(i * DT, 6),
               "q": _interpolate(samples, times, start + i * DT)}
              for i in range(steps + 1)]
    result = {
        "schema": DRAFT_SCHEMA,
        "qualification": "draft_unqualified_not_executable",
        "action_code": code,
        "action_version": version,
        "display_name": name.strip(),
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "source": {"file_name": Path(source).name, "schema": schema,
                   "sha256": source_sha, "start_s": start,
                   "requested_end_s": end},
        "observed_mode_machine": selected[0]["mode_machine"],
        "observed_mode_pr": selected[0]["mode_pr"],
        "hardware_enabled": False,
        "frames": frames,
        "metrics": metrics(frames),
        "trajectory_sha256": frame_digest(frames),
    }
    return result


def audit_draft(path):
    data = read_json(path)
    return validate_draft(data)


def validate_draft(data):
    """对已经读取的同一份草稿进行结构与摘要检查。"""
    if (not isinstance(data, dict)
            or data.get("schema") != DRAFT_SCHEMA
            or data.get("qualification") != "draft_unqualified_not_executable"
            or data.get("hardware_enabled") is not False
            or data.get("target_profile") != PROFILE
            or data.get("arm_slots") != list(ARM_SLOTS)):
        raise ValueError("不是本工具生成的未验收 Arm7 草稿")
    frames = data.get("frames")
    computed = metrics(frames)
    if data.get("trajectory_sha256") != frame_digest(frames):
        raise ValueError("轨迹摘要不匹配")
    return computed
