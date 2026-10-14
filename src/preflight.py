"""只读观测 G1 运控与手臂服务状态。"""

import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock

from .constants import ARM_COMMAND_TOPIC, LOWSTATE_TOPIC
from .files import write_json
from .unitree_binding import create_service_clients, read_arm_service, read_control_state


def report_summary(report):
    observation = report["observation"]
    return {
        "path": report["path"],
        "qualification": report["qualification"],
        "fsm_before": report["fsm_before"],
        "fsm_after": report["fsm_after"],
        "arm_service_before": report["arm_service_before"],
        "arm_service_after": report["arm_service_after"],
        "arm_sdk_messages": observation["arm_sdk_messages"],
        "lowstate_messages": observation["lowstate_messages"],
        "errors": report["errors"],
    }


def preflight(interface, seconds, output):
    """保存只读报告；不切换服务、不请求动作、不发送电机命令。"""
    if not 1 <= seconds <= 60:
        raise ValueError("观测时长必须在 1 到 60 秒之间")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请换一个报告文件名")

    # 离线制作草稿不依赖机器人 SDK，因此在这里才导入 SDK。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    report = {
        "schema": "g1-arm7-readonly-preflight-v3",
        "qualification": "read_only_observation_control_authority_unconfirmed",
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "interface": interface, "requested_seconds": seconds,
        "fsm_before": None, "fsm_after": None,
        "arm_service_before": None, "arm_service_after": None,
        "observation": {"arm_sdk_messages": 0, "lowstate_messages": 0,
                        "mode_pairs": [], "first_lowstate_at_seconds": None,
                        "last_lowstate_at_seconds": None},
        "exclusive_control": "unconfirmed",
        "errors": [],
    }
    subscribers = []
    lock = Lock()
    mode_pairs = set()
    started = time.monotonic()

    def on_arm_command(_message):
        with lock:
            report["observation"]["arm_sdk_messages"] += 1

    def on_lowstate(message):
        now = round(time.monotonic() - started, 6)
        with lock:
            observed = report["observation"]
            observed["lowstate_messages"] += 1
            if observed["first_lowstate_at_seconds"] is None:
                observed["first_lowstate_at_seconds"] = now
            observed["last_lowstate_at_seconds"] = now
            try:
                mode_pairs.add((int(message.mode_pr), int(message.mode_machine)))
            except (AttributeError, TypeError, ValueError):
                pass

    try:
        ChannelFactoryInitialize(0, interface)
        loco, switcher, robot_state = create_service_clients()
        try:
            report["fsm_before"] = read_control_state(loco, switcher)
        except Exception as error:
            report["errors"].append("初始运控查询：{}".format(error))
        try:
            report["arm_service_before"] = read_arm_service(robot_state)
        except Exception as error:
            report["errors"].append("初始手臂服务查询：{}".format(error))

        for topic, message_type, callback in (
            (ARM_COMMAND_TOPIC, LowCmd_, on_arm_command),
            (LOWSTATE_TOPIC, LowState_, on_lowstate),
        ):
            subscriber = ChannelSubscriber(topic, message_type)
            subscribers.append(subscriber)
            subscriber.Init(callback, 10)

        print("只读观测 {} 秒；请保持机器人静止，不要在本窗口触发动作。".format(seconds), flush=True)
        time.sleep(seconds)
        try:
            report["fsm_after"] = read_control_state(loco, switcher)
        except Exception as error:
            report["errors"].append("结束运控查询：{}".format(error))
        try:
            report["arm_service_after"] = read_arm_service(robot_state)
        except Exception as error:
            report["errors"].append("结束手臂服务查询：{}".format(error))
    finally:
        for subscriber in subscribers:
            try:
                subscriber.Close()
            except Exception as error:
                report["errors"].append("关闭只读订阅：{}".format(error))

    report["observation"]["mode_pairs"] = [list(pair) for pair in sorted(mode_pairs)]
    if report["observation"]["lowstate_messages"] == 0:
        report["errors"].append("观测窗口未收到 rt/lowstate")
    if report["fsm_before"] != report["fsm_after"]:
        report["errors"].append("观测前后运控状态不同或查询不完整")
    if report["arm_service_before"] is None or report["arm_service_after"] is None:
        report["errors"].append("未确认 g1_arm_example 服务状态")
    elif report["arm_service_before"] != report["arm_service_after"]:
        report["errors"].append("观测前后 g1_arm_example 服务状态不同")
    report["path"] = str(output)
    write_json(report, output, compact=False, allow_nan=True)
    return report_summary(report)
