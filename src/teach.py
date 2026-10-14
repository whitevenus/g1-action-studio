"""Arm7 手把手示教候选：显式接管、低刚度记录、原位释放。"""

import json
import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, Lock

from . import ARM_SLOTS, JOINT_NAMES, PROFILE, WEIGHT_SLOT
from .preflight import _read_arm_service, _read_control_state

# 控制循环目标间隔为20毫秒，大约50Hz。
PERIOD = 0.02
# 状态数据如果超过250毫秒就认为过期
STATE_TIMEOUT = 0.25
# 左右关节：15～21、22～28
LEFT = tuple(range(15, 22))
RIGHT = tuple(range(22, 29))


def load_profile(path, target_stage="record"):
    """检查候选参数；数值检查不代表本机已验收。hold 不需要示教增益。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    required = ("hold_kp", "hold_kd")
    if target_stage != "hold":
        required += ("teach_kp", "teach_kd")
    if data.get("schema") != "g1-arm7-teach-profile-v1" or data.get("target_profile") != PROFILE:
        raise ValueError("示教参数文件的格式或目标机型不匹配")
    for key in required:
        value = data.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError("示教参数 {} 必须是正的有限数".format(key))
    if target_stage != "hold" and data["teach_kp"] >= data["hold_kp"]:
        raise ValueError("示教刚度须小于保持刚度")
    if target_stage != "hold" and data["teach_kd"] > data["hold_kd"]:
        raise ValueError("示教阻尼不得大于保持阻尼")
    if data["hold_kp"] > 60 or data["hold_kd"] > 3:
        raise ValueError("保持增益超过候选程序上限")
    if target_stage != "hold" and (data["teach_kp"] > 20 or data["teach_kd"] > 2):
        raise ValueError("示教增益超过候选程序上限")
    result = {key: float(data[key]) for key in required}
    if target_stage == "hold":
        result.update(teach_kp=result["hold_kp"], teach_kd=result["hold_kd"])
    return result


def validate_snapshot(snapshot, baseline_modes):
    """每次发布前核对状态；任何不确定情况都停止示教。"""
    if snapshot is None:
        raise RuntimeError("rt/lowstate 已超时")
    if (len(snapshot["q"]) != len(ARM_SLOTS)
            or len(snapshot["dq"]) != len(ARM_SLOTS)
            or len(snapshot["motor_modes"]) != len(ARM_SLOTS)):
        raise RuntimeError("手臂状态关节数量不匹配")
    age = time.monotonic() - snapshot["at"]
    if not math.isfinite(age) or not 0 <= age <= STATE_TIMEOUT:
        raise RuntimeError("rt/lowstate 已超时或时间戳无效")
    if (snapshot["mode_pr"], snapshot["mode_machine"]) != (0, 5):
        raise RuntimeError("机器人模式已改变")
    if snapshot["motor_modes"] != baseline_modes:
        raise RuntimeError("手臂电机模式已改变")
    if not all(math.isfinite(x) for x in list(snapshot["q"]) + list(snapshot["dq"])):
        raise RuntimeError("关节状态包含非有限数")


class SdkArmPort:
    """DDS 读写边界；实例化后才会创建 rt/arm_sdk 发布者。"""

    def __init__(self, interface):
        self.publisher = self.arm_subscriber = self.subscriber = None
        try:
            self._initialize(interface)
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup_error:
                raise RuntimeError("初始化失败：{}；关闭通信失败：{}".format(
                    error, cleanup_error)) from error
            raise

    def _initialize(self, interface):
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
        from unitree_sdk2py.utils.crc import CRC

        ChannelFactoryInitialize(0, interface)
        self.lock = Lock()
        self.ready = Event()
        self.latest = None
        self.other_commands = 0
        self.publisher = None
        self.command_type = unitree_hg_msg_dds__LowCmd_
        self.message_type = LowCmd_
        # 进入执行模式后才创建rt/arm_sdk发布者
        self.publisher_type = ChannelPublisher
        self.crc = CRC()
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self._on_state, 10)
        self.arm_subscriber = ChannelSubscriber("rt/arm_sdk", LowCmd_)
        self.arm_subscriber.Init(self._on_arm_command, 10)
        self.loco = LocoClient()
        self.loco.SetTimeout(2.0)
        self.loco.Init()
        self.switcher = MotionSwitcherClient()
        self.switcher.SetTimeout(2.0)
        self.switcher.Init()
        self.robot_state = RobotStateClient()
        self.robot_state.SetTimeout(2.0)
        self.robot_state.Init()

    def _on_state(self, message):
        if len(message.motor_state) <= ARM_SLOTS[-1]:
            return
        motors = [message.motor_state[index] for index in ARM_SLOTS]
        snapshot = {
            "at": time.monotonic(), "mode_pr": int(message.mode_pr),
            "mode_machine": int(message.mode_machine),
            "motor_modes": tuple(int(m.mode) for m in motors),
            "q": tuple(float(m.q) for m in motors),
            "dq": tuple(float(m.dq) for m in motors),
        }
        with self.lock:
            self.latest = snapshot
        self.ready.set()

    def _on_arm_command(self, _message):
        with self.lock:
            self.other_commands += 1

    def snapshot(self):
        with self.lock:
            return self.latest

    def observed_commands(self):
        with self.lock:
            return self.other_commands

    def check_services(self):
        control = _read_control_state(self.loco, self.switcher)
        arm = _read_arm_service(self.robot_state)
        if control != {"motion_service": "ai", "fsm_id": 501}:
            raise RuntimeError("运控状态不符合本机候选条件：{}".format(control))
        if arm is None or arm["status"] != 0:
            raise RuntimeError("g1_arm_example 服务未确认关闭：{}".format(arm))
        return control, arm

    def start_publisher(self):
        self.publisher = self.publisher_type("rt/arm_sdk", self.message_type)
        self.publisher.Init()

    def write(self, positions, hold_kp, hold_kd, teach_kp, teach_kd, side, weight):
        command = self.command_type()
        selected = LEFT if side == "left" else RIGHT
        for offset, index in enumerate(ARM_SLOTS):
            motor = command.motor_cmd[index]
            motor.q = float(positions[offset])
            motor.dq = motor.tau = 0.0
            # 按关节槽位选择示教侧或保持侧的增益（kp：刚度系数、kd：阻尼系数）
            motor.kp = float(teach_kp if index in selected else hold_kp)
            motor.kd = float(teach_kd if index in selected else hold_kd)
        # 槽位 29 是控制权重，不是物理关节。
        command.motor_cmd[WEIGHT_SLOT].q = float(weight)
        command.crc = self.crc.Crc(command)
        if self.publisher.Write(command) is not True:
            raise RuntimeError("rt/arm_sdk 命令发送失败")

    def close(self):
        errors = []
        for name in ("publisher", "arm_subscriber", "subscriber"):
            resource = getattr(self, name, None)
            if resource is not None:
                try:
                    resource.Close()
                except Exception as error:
                    errors.append("{}：{}".format(name, error))
                finally:
                    setattr(self, name, None)
        if errors:
            raise RuntimeError("关闭通信失败：" + "；".join(errors))


def teach(interface, side, seconds, output, profile_path, execute=False, target_stage="hold"):
    """只在显式执行时创建 DDS 发布者；输出为未验收的状态录制。"""
    if side not in ("left", "right") or not 1 <= seconds <= 30:
        raise ValueError("只支持 left/right 单臂示教，时长为 1–30 秒")
    if target_stage not in ("hold", "soften", "record"):
        raise ValueError("示教阶段必须是 hold、soften 或 record")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请换新文件名")
    profile = load_profile(profile_path, target_stage)
    if not execute:
        return {"execute": False, "side": side, "seconds": seconds,
                "profile": profile, "output": str(output), "stage": target_stage,
                "message": "只核对参数；没有连接机器人或发送命令"}

    port = None
    stop = Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    # 最后尝试发送的参数；发送报错也可能已送达，收尾仍需使用这组值。
    last_weight = 0.0
    last_kp = profile["hold_kp"]
    last_kd = profile["hold_kd"]
    samples = []
    stage = "初始化"
    release_sent = True
    primary_error = None
    cleanup_errors = []
    start_q = modes = None
    trial_metrics = {}

    def observe(snapshot, targets, name):
        metrics = trial_metrics.setdefault(name, {"samples": 0,
            "max_speed_rad_s": 0.0, "max_start_offset_rad": 0.0,
            "max_target_error_rad": 0.0})
        metrics["samples"] += 1
        for key, value in (
                ("max_speed_rad_s", max(abs(x) for x in snapshot["dq"])),
                ("max_start_offset_rad", max(abs(a - b) for a, b in zip(snapshot["q"], start_q))),
                ("max_target_error_rad", max(abs(a - b) for a, b in zip(snapshot["q"], targets)))):
            metrics[key] = max(metrics[key], value)

    def request_stop(_signum, _frame):
        stop.set()

    # 一次控制周期（读取状态 -> 检查 -> 发指令）
    def tick(teach_kp, teach_kd, weight, follow_measured=False):
        nonlocal last_weight, last_kp, last_kd
        if stop.is_set():
            raise RuntimeError("操作员已请求停止")
        snapshot = port.snapshot()
        validate_snapshot(snapshot, modes)
        # 接管/保持固定双臂目标；仅低刚度示教阶段更新示教侧目标。
        targets = list(start_q)
        if follow_measured:
            for index in selected:
                targets[index - ARM_SLOTS[0]] = snapshot["q"][index - ARM_SLOTS[0]]
        observe(snapshot, targets, stage)
        last_weight, last_kp, last_kd = weight, teach_kp, teach_kd
        port.write(targets, profile["hold_kp"], profile["hold_kd"],
                   teach_kp, teach_kd, side, weight)
        return snapshot

    # 按时间渐变一个阶段
    def phase(duration, weight_start, weight_end,
              kp_start, kp_end, kd_start, kd_end, capture=False, follow_measured=False):
        steps = max(1, round(duration / PERIOD))
        began = time.monotonic()
        for step in range(steps + 1):
            ratio = step / steps
            snapshot = tick(
                            kp_start + (kp_end - kp_start) * ratio,
                            kd_start + (kd_end - kd_start) * ratio,
                            weight_start + (weight_end - weight_start) * ratio,
                            follow_measured)
            if capture:
                samples.append({"t": round(time.monotonic() - capture_start, 6),
                                "mode_pr": snapshot["mode_pr"],
                                "mode_machine": snapshot["mode_machine"],
                                "q": list(snapshot["q"]), "dq": list(snapshot["dq"])})
            # 安排下一次循环（尽量维持指定hz）
            delay = began + (step + 1) * PERIOD - time.monotonic()
            if step < steps and delay > 0:
                time.sleep(delay)

    try:
        port = SdkArmPort(interface)
        if not port.ready.wait(5):
            raise RuntimeError("五秒内未收到 rt/lowstate")
        port.check_services()
        start = port.snapshot()
        if start is None or time.monotonic() - start["at"] > STATE_TIMEOUT:
            raise RuntimeError("初始关节状态已过期")
        modes = start["motor_modes"]
        if any(mode != 1 for mode in modes):
            raise RuntimeError("手臂电机模式不全为 1")
        # 核对状态时效、模式、关节数量及数值有效性。
        validate_snapshot(start, modes)
        print("执行前观测 rt/arm_sdk 3 秒……", flush=True)
        time.sleep(3)
        if port.observed_commands():
            raise RuntimeError("观测到其他手臂命令；不接管")
        port.check_services()
        start = port.snapshot()
        validate_snapshot(start, modes)
        start_q = start["q"]
        selected = LEFT if side == "left" else RIGHT
        # 注册停止信号处理器，然后创建发布者
        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        port.start_publisher()

        stage = "接管"
        phase(3.0, 0.0, 1.0,
              profile["hold_kp"], profile["hold_kp"],
              profile["hold_kd"], profile["hold_kd"])
        if target_stage == "hold":
            stage = "固定位置保持"
            phase(seconds, 1.0, 1.0,
                  profile["hold_kp"], profile["hold_kp"],
                  profile["hold_kd"], profile["hold_kd"])
        if target_stage in ("soften", "record"):
            # 操作员应已托住手臂；从此阶段才让示教侧目标跟随测量位置。
            stage = "降低示教刚度"
            phase(3.0, 1.0, 1.0,
                  profile["hold_kp"], profile["teach_kp"],
                  profile["hold_kd"], profile["teach_kd"], follow_measured=True)
        if target_stage == "record":
            stage = "录制"
            capture_start = time.monotonic()
            print("RECORDING_STARTED：托住示教手臂缓慢移动；请保持另一侧净空。", flush=True)
            phase(seconds, 1.0, 1.0,
                  profile["teach_kp"], profile["teach_kp"],
                  profile["teach_kd"], profile["teach_kd"], capture=True, follow_measured=True)
        stage = "结束"
    except BaseException as error:
        primary_error = error
    finally:
        if port is not None:
            try:
                if last_weight > 0:
                    # 只采一次释放目标；后续反馈用于检查，不能让目标随下沉移动。
                    snapshot = port.snapshot()
                    validate_snapshot(snapshot, modes)
                    release_q = tuple(snapshot["q"])
                    for step in range(101):
                        snapshot = port.snapshot()
                        validate_snapshot(snapshot, modes)
                        observe(snapshot, release_q, "释放")
                        ratio = min(step / 50, 1.0)
                        gain = last_kp + (profile["hold_kp"] - last_kp) * ratio
                        damping = last_kd + (profile["hold_kd"] - last_kd) * ratio
                        weight = last_weight if step <= 50 else last_weight * (1 - (step - 50) / 50)
                        port.write(release_q, profile["hold_kp"], profile["hold_kd"],
                                   gain, damping, side, weight)
                        if step < 100:
                            time.sleep(PERIOD)
                    print("已请求交还手臂权重；请现场确认手臂和机体状态。", flush=True)
            except Exception as error:
                release_sent = False
                cleanup_errors.append("释放失败：{}".format(error))
                print("自动释放未确认：{}；请现场操作员接管。".format(error), flush=True)
            finally:
                try:
                    port.close()
                except Exception as error:
                    cleanup_errors.append(str(error))
        for signum, previous in ((signal.SIGINT, previous_sigint),
                                 (signal.SIGTERM, previous_sigterm)):
            try:
                signal.signal(signum, previous)
            except Exception as error:
                cleanup_errors.append("恢复信号处理失败：{}".format(error))

    if cleanup_errors:
        message = "；".join(cleanup_errors) + "；未保存可用记录，请现场接管"
        if primary_error is not None:
            message = "示教失败：{}；{}".format(primary_error, message)
        raise RuntimeError(message) from primary_error
    if primary_error is not None:
        raise primary_error

    if not release_sent:
        raise RuntimeError("释放命令未完整发送；未保存可用记录")
    if stage != "结束":
        raise RuntimeError("示教未完整结束，未保存可用记录")
    if target_stage != "record":
        return {"stage": target_stage, "qualification": "unqualified_hardware_trial",
                "recording_written": False,
                "trial_metrics": trial_metrics,
                "message": "已发送释放命令；实际交接仍须现场确认"}
    if len(samples) < 2:
        raise RuntimeError("录制样本不足，未保存可用记录")
    payload = {
        "schema": "g1-arm7-state-recording-v1",
        "qualification": "raw_observation_not_executable",
        "target_profile": PROFILE,
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "interface": interface, "requested_seconds": seconds,
        "arm_slots": list(ARM_SLOTS), "joint_names": list(JOINT_NAMES),
        "teaching_side": side, "teaching_profile": profile,
        "trial_metrics": trial_metrics,
        "messages_received": len(samples), "invalid_messages": 0,
        "samples": samples,
    }
    with output.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, separators=(",", ":"))
        stream.write("\n")
    return {"path": str(output), "samples": len(samples),
            "qualification": payload["qualification"]}
