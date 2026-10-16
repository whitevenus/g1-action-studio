"""将统一录制或动作文件加载为可供规划的动作；读取一次，自动检查。"""

import hashlib
import json
import math
import re
from bisect import bisect_right
from pathlib import Path

from .config import document_layout, normalize_document
from .constants import ACTION_SCHEMA, ACTION_SCHEMA_V2, ARM_SLOTS, DT, EXPECTED_MODE
from .trajectory import Frame, finite_vector, frame_digest, interpolate_positions, metrics

MAX_SOURCE_GAP = 0.10
CODE_PATTERN = re.compile(r"^[a-z][a-z0-9]*(?:[._][a-z0-9]+)*$")


def _interpolate(samples: list[dict], times: list[float], moment: float) -> list[float]:
    right = bisect_right(times, moment)
    if right == len(times):
        return list(samples[-1]["q"])
    left = right - 1
    span = times[right] - times[left]
    ratio = (moment - times[left]) / span
    return interpolate_positions(samples[left]["q"], samples[right]["q"], ratio)


def validate_action(data):
    layout = document_layout(data, "action")
    if (
        not isinstance(data, dict)
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
        if data.get("side") not in ("left", "right") or not isinstance(collision, dict):
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
    from .trajectory import validate_frames

    validate_frames(data.get("frames"), size=layout.joint_count)
    result = metrics(data.get("frames"))
    if data.get("trajectory_sha256") != frame_digest(data["frames"]):
        raise ValueError("轨迹摘要不匹配")
    return result


def _recording_frames(rows: list[dict], size) -> list[Frame]:
    """校验整段录制并重采样到 50 Hz，保留起点和最终实测姿态。"""
    samples = []
    last_time = float("-inf")
    for row in rows:
        t = float(row["t"])
        if not math.isfinite(t) or t <= last_time or t < 0:
            raise ValueError("录制时间必须为严格递增的非负有限值")
        if samples and t - last_time > MAX_SOURCE_GAP:
            raise ValueError("录制中存在超过 0.1 秒的状态缺口")
        samples.append(
            {
                "t": t,
                "q": finite_vector(row["q"], size=size),
                "dq": finite_vector(row["dq"], size=size),
                "mode_machine": int(row["mode_machine"]),
                "mode_pr": int(row["mode_pr"]),
            }
        )
        last_time = t
    duration = samples[-1]["t"] - samples[0]["t"]
    if duration < 2 * DT - 1e-9:
        raise ValueError("录制时长不足，动作至少需要三帧")
    times = [row["t"] for row in samples]
    steps = math.ceil(duration / DT - 1e-9)
    modes = {(row["mode_pr"], row["mode_machine"]) for row in samples}
    if len(modes) != 1:
        raise ValueError("录制过程中设备模式发生变化")
    if modes != {EXPECTED_MODE}:
        raise ValueError("录制模式不符合 0/5")
    return [
        {"t": round(i * DT, 6), "q": _interpolate(samples, times, times[-1] if i == steps else times[0] + i * DT)}
        for i in range(steps + 1)
    ]


def load_action(
    source: str | Path,
    *,
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
    if data.get("schema") in (ACTION_SCHEMA, ACTION_SCHEMA_V2):
        validate_action(data)
        return normalize_document(data, "action")
    layout = document_layout(data, "recording")
    rows = data.get("samples")
    if not isinstance(rows, list) or len(rows) < 3:
        raise ValueError("至少需要三帧状态")
    if not CODE_PATTERN.fullmatch(code) or len(code) > 120:
        raise ValueError("动作代码只能使用小写字母、数字、点和下划线")
    name = source.stem if name is None else name.strip()
    if type(version) is not int or version <= 0 or not name:
        raise ValueError("动作版本和名称无效")
    frames = _recording_frames(rows, layout.joint_count)
    return {
        "schema": ACTION_SCHEMA_V2,
        "kind": "single",
        "action_code": code,
        "action_version": version,
        "display_name": name,
        "target_profile": layout.profile_id,
        "collision_hand_model": "dex3-1",
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(layout.joint_names),
        "synthetic": data.get("qualification") == "synthetic_example_not_for_hardware",
        "source": {
            "file_name": source.name,
            "schema": data["schema"],
            "sha256": hashlib.sha256(raw).hexdigest(),
        },
        "frames": frames,
        "metrics": metrics(frames),
        "trajectory_sha256": frame_digest(frames),
    }
