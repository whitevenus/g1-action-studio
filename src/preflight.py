"""只读观测 G1 运控与手臂服务状态。"""

import json
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Lock


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


def _read_control_state(loco, switcher):
    for attempt in range(3):
        code, mode = switcher.CheckMode()
        # SDK 的 3104 表示等待 RPC 回复超时；只重试只读查询。
        if code != 3104 or attempt == 2:
            break
        print("CheckMode 查询超时，稍后进行第 {} 次只读查询。".format(attempt + 2), flush=True)
        time.sleep(0.5)
    if code != 0 or not isinstance(mode, dict) or not isinstance(mode.get("name"), str):
        raise RuntimeError("CheckMode 失败：返回码={!r}，数据={!r}".format(code, mode))
    code, fsm_id = loco.GetFsmId()
    if code != 0 or type(fsm_id) is not int:
        raise RuntimeError("GetFsmId 失败：返回码={!r}，数据={!r}".format(code, fsm_id))
    return {"motion_service": mode["name"], "fsm_id": fsm_id}


def _read_arm_service(robot_state):
    code, services = robot_state.ServiceList()
    if code != 0 or services is None:
        raise RuntimeError("ServiceList 失败：返回码={!r}".format(code))
    for service in services:
        if service.name == "g1_arm_example":
            return {"name": service.name, "status": service.status,
                    "protect": service.protect}
    return None


def preflight(interface, seconds, output):
    """保存只读报告；不切换服务、不请求动作、不发送电机命令。"""
    if not 1 <= seconds <= 60:
        raise ValueError("观测时长必须在 1 到 60 秒之间")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请换一个报告文件名")

    # 离线制作草稿不依赖机器人 SDK，因此在这里才导入 SDK。
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
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
        loco = LocoClient()
        loco.SetTimeout(2.0)
        loco.Init()
        switcher = MotionSwitcherClient()
        switcher.SetTimeout(2.0)
        switcher.Init()
        robot_state = RobotStateClient()
        robot_state.SetTimeout(2.0)
        robot_state.Init()
        try:
            report["fsm_before"] = _read_control_state(loco, switcher)
        except Exception as error:
            report["errors"].append("初始运控查询：{}".format(error))
        try:
            report["arm_service_before"] = _read_arm_service(robot_state)
        except Exception as error:
            report["errors"].append("初始手臂服务查询：{}".format(error))

        for topic, message_type, callback in (
            ("rt/arm_sdk", LowCmd_, on_arm_command),
            ("rt/lowstate", LowState_, on_lowstate),
        ):
            subscriber = ChannelSubscriber(topic, message_type)
            subscribers.append(subscriber)
            subscriber.Init(callback, 10)

        print("只读观测 {} 秒；请保持机器人静止，不要在本窗口触发动作。".format(seconds), flush=True)
        time.sleep(seconds)
        try:
            report["fsm_after"] = _read_control_state(loco, switcher)
        except Exception as error:
            report["errors"].append("结束运控查询：{}".format(error))
        try:
            report["arm_service_after"] = _read_arm_service(robot_state)
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
    with output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return report_summary(report)
