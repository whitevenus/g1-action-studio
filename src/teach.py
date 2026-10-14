"""Arm7 示教：回车录制、保持终点、恢复刚度、避碰回位后释放。"""

import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, Lock, Thread
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor

from .constants import (
    ARM_COMMAND_TOPIC, ARM_SLOTS, DT, EXPECTED_MODE, JOINT_NAMES, LEFT, LOWSTATE_TOPIC, PROFILE,
    RECORDING_SCHEMA, RIGHT, STATE_TIMEOUT, WEIGHT_SLOT,
)
from .config import load_teach_profile as load_profile, write_json
from .unitree_binding import (arm_snapshot, close_channels, create_service_clients,
                             read_arm_service, read_control_state)


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
    if (snapshot["mode_pr"], snapshot["mode_machine"]) != EXPECTED_MODE:
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
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
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
        self.subscriber = ChannelSubscriber(LOWSTATE_TOPIC, LowState_)
        self.subscriber.Init(self._on_state, 10)
        self.arm_subscriber = ChannelSubscriber(ARM_COMMAND_TOPIC, LowCmd_)
        self.arm_subscriber.Init(self._on_arm_command, 10)
        self.loco, self.switcher, self.robot_state = create_service_clients()

    def _on_state(self, message):
        if len(message.motor_state) <= ARM_SLOTS[-1]:
            return
        snapshot = arm_snapshot(message, time)
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
        control = read_control_state(self.loco, self.switcher)
        arm = read_arm_service(self.robot_state)
        if control != {"motion_service": "ai", "fsm_id": 501}:
            raise RuntimeError("运控状态不符合本机候选条件：{}".format(control))
        if arm is None or arm["status"] != 0:
            raise RuntimeError("g1_arm_example 服务未确认关闭：{}".format(arm))
        return control, arm

    def start_publisher(self):
        self.publisher = self.publisher_type(ARM_COMMAND_TOPIC, self.message_type)
        self.publisher.Init()

    def write(self, positions, hold_kp, hold_kd, teach_kp, teach_kd, side, weight):
        command = self.command_type()
        command.mode_pr, command.mode_machine = EXPECTED_MODE
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
        errors = close_channels(self)
        if errors:
            raise RuntimeError("关闭通信失败：" + "；".join(errors))


def teach(interface, side, seconds, output, profile_path, execute=False, target_stage="record"):
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
    if target_stage == "record":
        return record_session(interface, side, output, profile)

    port = None
    stop = Event()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    # 最后尝试发送的参数；发送报错也可能已送达，收尾仍需使用这组值。
    last_weight = 0.0
    last_kp = profile["hold_kp"]
    last_kd = profile["hold_kd"]
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
              kp_start, kp_end, kd_start, kd_end, follow_measured=False):
        steps = max(1, round(duration / DT))
        began = time.monotonic()
        for step in range(steps + 1):
            ratio = step / steps
            snapshot = tick(
                            kp_start + (kp_end - kp_start) * ratio,
                            kd_start + (kd_end - kd_start) * ratio,
                            weight_start + (weight_end - weight_start) * ratio,
                            follow_measured)
            # 安排下一次循环（尽量维持指定hz）
            delay = began + (step + 1) * DT - time.monotonic()
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
        if target_stage == "soften":
            # 操作员应已托住手臂；从此阶段才让示教侧目标跟随测量位置。
            stage = "降低示教刚度"
            phase(3.0, 1.0, 1.0,
                  profile["hold_kp"], profile["teach_kp"],
                  profile["hold_kd"], profile["teach_kd"], follow_measured=True)
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
                            time.sleep(DT)
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
    return {"stage": target_stage, "qualification": "unqualified_hardware_trial",
            "recording_written": False, "trial_metrics": trial_metrics,
            "message": "已发送释放命令；实际交接仍须现场确认"}


def read_record_keys(events, stop):
    """键盘线程只产生事件，不读取反馈或发送控制指令。"""
    try:
        for prompt in ("摆好起点后按回车开始录制：", "完成动作后按回车结束录制："):
            if stop.is_set():
                return
            input(prompt)
            events.put("enter")
    except (EOFError, OSError) as error:
        events.put(error)


def plan_return(start, end, side, body_q):
    from .collision import CollisionModel, find_path, smooth_path
    model = CollisionModel(body_q)
    motion = smooth_path(find_path(model, start, end, side), 0.5, 2.0)
    model.check_motion(motion, 0.02)
    return motion


def record_session(interface, side, output, profile):
    """回车录制；终点保持、恢复刚度、避碰回位、稳定后才释放。"""
    from .trajectory import interpolate_positions, smoothstep
    from .constants import DEFAULT_MODEL
    from .replay import load_limits
    limits = load_limits(DEFAULT_MODEL)
    stop, events = Event(), Queue()
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    port = None
    baseline = modes = target = planned_body = None
    selected = range(7) if side == "left" else range(7, 14)
    samples = []
    last_tick = None
    weight = 0.0
    stage = "准备"
    pool = None

    def tick(kp, kd, w, follow=False, capture=False):
        nonlocal target, weight, last_tick
        if stop.is_set():
            raise RuntimeError("操作员已请求停止")
        now = time.monotonic()
        if last_tick is not None and now - last_tick > 0.1:
            raise RuntimeError("示教控制周期超时")
        state = port.snapshot()
        validate_snapshot(state, modes)
        if any(not low <= q <= high for q, (low, high) in zip(state["q"], limits)):
            raise RuntimeError("反馈角度超出关节范围")
        if planned_body is not None:
            body = state.get("body_q")
            if (body is None or len(body) != 15 or any(not math.isfinite(v) for v in body)
                    or max(abs(a - b) for a, b in zip(body, planned_body)) > 0.02):
                raise RuntimeError("腿/腰姿态偏离回位规划")
        if follow:
            target = list(baseline)
            for i in selected:
                target[i] = state["q"][i]
        if max(abs(a - b) for a, b in zip(state["q"], target)) > 0.15:
            raise RuntimeError("示教保持或回位跟踪误差超限")
        weight, last_tick = w, now
        port.write(target, profile["hold_kp"], profile["hold_kd"], kp, kd, side, w)
        if capture:
            samples.append({"t": round(now - capture_start, 6), "q": list(state["q"]),
                            "dq": list(state["dq"]), "mode_pr": state["mode_pr"],
                            "mode_machine": state["mode_machine"]})
        return state

    def ramp(seconds, w0, w1, kp0, kp1, kd0, kd1, follow=False):
        began = time.monotonic()
        steps = round(seconds / DT)
        for i in range(steps + 1):
            blend = smoothstep(i / steps)
            tick(kp0 + (kp1-kp0)*blend, kd0 + (kd1-kd0)*blend,
                 w0 + (w1-w0)*blend, follow=follow)
            if i < steps:
                time.sleep(max(0, began + (i+1)*DT - time.monotonic()))

    try:
        port = SdkArmPort(interface)
        if not port.ready.wait(5):
            raise RuntimeError("等待初始状态超时")
        port.check_services()
        state = port.snapshot()
        modes = state["motor_modes"] if state else ()
        validate_snapshot(state, modes)
        if modes != (1,) * 14:
            raise RuntimeError("手臂电机模式不符合条件")
        time.sleep(3)
        if port.observed_commands():
            raise RuntimeError("观测到其他手臂命令；不接管")
        port.check_services()
        state = port.snapshot()
        validate_snapshot(state, modes)
        baseline = tuple(state["q"])
        target = list(baseline)
        if max(map(abs, state["dq"])) > 0.1:
            raise RuntimeError("初始手臂尚未静止")
        # 初始姿态须在模型范围内且无本体碰撞，才能开始接管。
        if state.get("body_q") is None:
            raise RuntimeError("避碰示教需要腿/腰姿态反馈")
        plan_return(baseline, baseline, side, state["body_q"])
        for sig in previous:
            signal.signal(sig, lambda *_: stop.set())
        port.start_publisher()
        ramp(3, 0, 1, profile["hold_kp"], profile["hold_kp"], profile["hold_kd"], profile["hold_kd"])
        ramp(3, 1, 1, profile["hold_kp"], profile["teach_kp"], profile["hold_kd"], profile["teach_kd"], True)
        Thread(target=read_record_keys, args=(events, stop), daemon=True).start()
        recording = False
        stage = "等待开始"
        while True:
            try:
                event = events.get_nowait()
            except Empty:
                event = None
            if isinstance(event, Exception):
                raise RuntimeError("录制输入已关闭") from event
            if event == "enter":
                if recording:
                    tick(profile["teach_kp"], profile["teach_kd"], 1, True, True)
                    break
                recording = True
                capture_start = time.monotonic()
                stage = "录制"
                print("RECORDING_STARTED", flush=True)
            tick(profile["teach_kp"], profile["teach_kd"], 1, True, recording)
            time.sleep(DT)
        # 最后一帧已冻结 target；恢复增益时不再追随手动移动。
        stage = "恢复刚度"
        ramp(1, 1, 1, profile["teach_kp"], profile["hold_kp"], profile["teach_kd"], profile["hold_kd"])
        state = port.snapshot()
        planned_body = state.get("body_q")
        if planned_body is None:
            raise RuntimeError("避碰回位需要腿/腰姿态反馈")
        stage = "规划回位"
        pool = ThreadPoolExecutor(max_workers=1)
        future = pool.submit(plan_return, list(target), baseline, side, list(planned_body))
        while not future.done():
            tick(profile["hold_kp"], profile["hold_kd"], 1)
            time.sleep(DT)
        motion = future.result()
        stage = "回位"
        began = time.monotonic()
        while True:
            elapsed = min(motion[-1]["t"], time.monotonic() - began)
            # 返回路径只含 q/t，直接使用公共位置插值。
            index = min(int(elapsed / DT), len(motion)-1)
            after = min(index+1, len(motion)-1)
            target = interpolate_positions(motion[index]["q"], motion[after]["q"],
                                           min(1, max(0, elapsed/DT-index)))
            tick(profile["hold_kp"], profile["hold_kd"], 1)
            if elapsed >= motion[-1]["t"]:
                break
            time.sleep(DT)
        target = list(baseline)
        stage = "等待回位稳定"
        began, stable = time.monotonic(), None
        while True:
            state = tick(profile["hold_kp"], profile["hold_kd"], 1)
            now = time.monotonic()
            if (max(abs(a-b) for a,b in zip(state["q"], baseline)) <= 0.05
                    and max(map(abs, state["dq"])) <= 0.1):
                stable = now if stable is None else stable
                if now-stable >= 0.1:
                    break
            else:
                stable = None
            if now-began > 5:
                raise RuntimeError("实际回位等待超时")
            time.sleep(DT)
        stage = "释放"
        ramp(2, 1, 0, profile["hold_kp"], profile["hold_kp"], profile["hold_kd"], profile["hold_kd"])
    except BaseException as error:
        if weight > 0:
            raise RuntimeError("示教在{}阶段停止：{}；未自动释放，请现场接管".format(stage, error)) from error
        raise
    finally:
        stop.set()
        if pool is not None:
            pool.shutdown(wait=False)
        try:
            if port is not None:
                port.close()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)
    if len(samples) < 3:
        raise RuntimeError("录制至少需要三帧；已回位释放，未保存录制")
    payload = {"schema": RECORDING_SCHEMA, "qualification": "raw_observation_not_executable",
               "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS), "joint_names": list(JOINT_NAMES),
               "recorded_at_utc": datetime.now(timezone.utc).isoformat(), "teaching_side": side,
               "teaching_profile": profile, "baseline_q": list(baseline),
               "messages_received": len(samples), "invalid_messages": 0, "samples": samples}
    write_json(payload, output)
    return {"path": str(output), "samples": len(samples), "release_sent": True,
            "duration_s": samples[-1]["t"], "qualification": payload["qualification"]}
