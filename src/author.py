"""将统一录制或动作文件加载为可供规划的动作；读取一次，自动检查。"""

import hashlib
import json
import math
import re
from bisect import bisect_right
from pathlib import Path

from .constants import ACTION_SCHEMA, ARM_SLOTS, DT, EXPECTED_MODE, JOINT_NAMES, PROFILE, RECORDING_SCHEMA, SIDE_OFFSETS
from .trajectory import Frame, finite_vector, frame_digest, interpolate_positions, metrics

MAX_SOURCE_GAP = 0.10
CODE_PATTERN = re.compile(r"^[a-z][a-z0-9]*(?:[._][a-z0-9]+)*$")


def _interpolate(samples: list[dict], times: list[float], moment: float) -> list[float]:
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


def validate_action(data):
    if (
        not isinstance(data, dict)
        or data.get("schema") != ACTION_SCHEMA
        or data.get("target_profile") != PROFILE
        or data.get("arm_slots") != list(ARM_SLOTS)
        or data.get("joint_names") != list(JOINT_NAMES)
        or data.get("kind") not in ("single", "sequence")
        or type(data.get("synthetic")) is not bool
    ):
        raise ValueError("不是本项目的统一动作文件")
    code = data.get("action_code")
    version = data.get("action_version")
    name = data.get("display_name")
    if (
        not isinstance(code, str)
        or not CODE_PATTERN.fullmatch(code)
        or len(code) > 120
        or type(version) is not int
        or version <= 0
        or not isinstance(name, str)
        or not name.strip()
    ):
        raise ValueError("动作代码、版本或名称无效")
    if data["kind"] == "sequence":
        collision = data.get("collision")
        if data.get("side") not in SIDE_OFFSETS or not isinstance(collision, dict):
            raise ValueError("组合动作缺少回放侧或碰撞规划参数")
        body = collision.get("body_q")
        margin = collision.get("clearance_m")
        if (
            not isinstance(body, list)
            or len(body) != ARM_SLOTS[0]
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in body)
            or type(margin) not in (int, float)
            or not 0.001 <= margin <= 0.10
        ):
            raise ValueError("组合动作的机体姿态或避碰余量无效")
    result = metrics(data.get("frames"))
    if data.get("trajectory_sha256") != frame_digest(data["frames"]):
        raise ValueError("轨迹摘要不匹配")
    return result


def _recording_frames(rows: list[dict], start: float | None, end: float | None) -> tuple[list[Frame], float, float]:
    """校验录制状态并按裁剪区间重采样；不读取文件或修改原始数据。"""
    samples = []
    last_time = float("-inf")
    for row in rows:
        t = float(row["t"])
        if not math.isfinite(t) or t <= last_time or t < 0:
            raise ValueError("录制时间必须为严格递增的非负有限值")
        samples.append(
            {
                "t": t,
                "q": finite_vector(row["q"]),
                "dq": finite_vector(row["dq"]),
                "mode_machine": int(row["mode_machine"]),
                "mode_pr": int(row["mode_pr"]),
            }
        )
        last_time = t
    start = samples[0]["t"] if start is None else float(start)
    end = samples[-1]["t"] if end is None else float(end)
    if not math.isfinite(start) or not math.isfinite(end) or end - start < 2 * DT - 1e-9:
        raise ValueError("裁剪时间无效，动作至少需要三帧")
    times = [row["t"] for row in samples]
    if start < times[0] or end > times[-1]:
        raise ValueError("裁剪时间超出录制范围")
    steps = int(math.floor((end - start) / DT + 1e-9))
    first = max(0, bisect_right(times, start) - 1)
    last = min(len(samples) - 1, bisect_right(times, start + steps * DT))
    modes = {(row["mode_pr"], row["mode_machine"]) for row in samples[first : last + 1]}
    if len(modes) != 1:
        raise ValueError("录制区间内设备模式发生变化")
    if modes != {EXPECTED_MODE}:
        raise ValueError("录制模式不符合 0/5")
    frames = [{"t": round(i * DT, 6), "q": _interpolate(samples, times, start + i * DT)} for i in range(steps + 1)]
    return frames, start, end


def load_action(
    source: str | Path,
    start: float | None = None,
    end: float | None = None,
    code: str = "recorded.action",
    version: int = 1,
    name: str | None = None,
) -> dict:
    """录制自动转成动作；已有动作直接加载，不重复制作。"""
    source = Path(source)
    raw = source.read_bytes()
    data = json.loads(raw.decode("utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("输入必须是录制或动作 JSON 对象")
    if data.get("schema") == ACTION_SCHEMA:
        if start is not None or end is not None:
            raise ValueError("裁剪输入须为原始录制文件")
        validate_action(data)
        return data
    if (
        data.get("schema") != RECORDING_SCHEMA
        or data.get("target_profile") != PROFILE
        or data.get("arm_slots") != list(ARM_SLOTS)
        or data.get("joint_names") != list(JOINT_NAMES)
    ):
        raise ValueError("不支持的录制或动作格式")
    rows = data.get("samples")
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError("至少需要三帧状态")
    if not CODE_PATTERN.fullmatch(code) or len(code) > 120:
        raise ValueError("动作代码只能使用小写字母、数字、点和下划线")
    name = source.stem if name is None else name.strip()
    if type(version) is not int or version <= 0 or not name:
        raise ValueError("动作版本和名称无效")
    frames, start, end = _recording_frames(rows, start, end)
    return {
        "schema": ACTION_SCHEMA,
        "kind": "single",
        "action_code": code,
        "action_version": version,
        "display_name": name,
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "synthetic": data.get("qualification") == "synthetic_example_not_for_hardware",
        "source": {
            "file_name": source.name,
            "sha256": hashlib.sha256(raw).hexdigest(),
            "start_s": start,
            "requested_end_s": end,
        },
        "frames": frames,
        "metrics": metrics(frames),
        "trajectory_sha256": frame_digest(frames),
    }
