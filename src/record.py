"""只读录制 rt/lowstate；不创建命令发布者，也不请求动作。"""

import math
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

from .constants import ARM_SLOTS, DT, JOINT_NAMES, LOWSTATE_TOPIC, PROFILE, RECORDING_SCHEMA
from .files import write_json


def record(interface, seconds, output):
    """保存观测到的手臂状态；经授权的动作由现场操作流程另行触发。"""
    if not 1 <= seconds <= 120:
        raise ValueError("录制时长必须在 1 到 120 秒之间")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请保留原始记录并换文件名")

    # 离线命令不依赖机器人 SDK，因此在这里才导入 SDK。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    lock = Lock()
    started = time.monotonic()
    samples = []
    received = 0
    invalid = 0
    last_saved = float("-inf")

    def on_state(message):
        nonlocal received, invalid, last_saved
        now = time.monotonic()
        try:
            if len(message.motor_state) <= ARM_SLOTS[-1]:
                raise ValueError("电机状态缺少手臂槽位")
            motors = [message.motor_state[i] for i in ARM_SLOTS]
            q = [float(m.q) for m in motors]
            dq = [float(m.dq) for m in motors]
            if not all(math.isfinite(x) for x in q + dq):
                raise ValueError("关节状态包含非有限值")
            sample = {
                "t": round(now - started, 6),
                "mode_pr": int(message.mode_pr),
                "mode_machine": int(message.mode_machine),
                "q": q,
                "dq": dq,
            }
        except (AttributeError, IndexError, TypeError, ValueError):
            with lock:
                invalid += 1
            return
        with lock:
            received += 1
            if now - last_saved < DT:
                return
            last_saved = now
            samples.append(sample)

    subscriber = None
    try:
        ChannelFactoryInitialize(0, interface)
        subscriber = ChannelSubscriber(LOWSTATE_TOPIC, LowState_)
        subscriber.Init(on_state, 10)
        print("只读记录 rt/lowstate；此程序不创建动作命令发布者。", flush=True)
        time.sleep(seconds)
    finally:
        if subscriber is not None:
            subscriber.Close()

    if len(samples) < 2:
        raise RuntimeError("未收到足够的有效关节状态；未保存录制")
    payload = {
        "schema": RECORDING_SCHEMA,
        "qualification": "raw_observation_not_executable",
        "target_profile": PROFILE,
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "interface": interface,
        "requested_seconds": seconds,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "messages_received": received,
        "invalid_messages": invalid,
        "samples": samples,
    }
    write_json(payload, output, allow_nan=True)
    return {"path": str(output), "samples": len(samples), "received": received,
            "invalid": invalid}
