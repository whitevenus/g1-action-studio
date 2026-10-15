"""Arm7 示教：回车录制、保持终点、恢复刚度、避碰回位后释放。"""

import math
import signal
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, Thread
from queue import Queue, Empty
from concurrent.futures import ThreadPoolExecutor

from .constants import (
    ARM_SLOTS, DT, EXPECTED_MODE, EXPECTED_MOTOR_MODES, JOINT_NAMES, PROFILE,
    RECORDING_SCHEMA, STATE_TIMEOUT, JOINT_COUNT, SIDE_OFFSETS,
)
from .config import load_teach_profile as load_profile, write_json
from .unitree_binding import UnitreeReplaySDK
from .replay_sdk import ReplaySdkPort


def validate_snapshot(snapshot, baseline_modes=EXPECTED_MOTOR_MODES) -> dict:
    """校验并返回有效状态；失败时抛异常，不返回 None。"""
    if snapshot is None:
        raise RuntimeError("rt/lowstate 已超时")
    if (len(snapshot["q"]) != JOINT_COUNT
            or len(snapshot["dq"]) != JOINT_COUNT
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
    return snapshot


class SdkArmPort(ReplaySdkPort):
    """示教与回放共用手臂/Dex3-1通信和资源清理。"""

    def __init__(self, interface):
        self.publisher = self.arm_subscriber = self.subscriber = None
        try:
            self._initialize(interface)
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup_error:
                raise RuntimeError("初始化失败：{}；关闭失败：{}".format(error, cleanup_error)) from error
            raise

    def _initialize(self, interface):
        sdk = UnitreeReplaySDK()
        sdk.initialize_network(interface, allow_network=True)
        sdk.initialize_service_clients()
        super().__init__(sdk, time)

    def write(self, positions, hold_kp, hold_kd, teach_kp, teach_kd, side, weight,
              hand_kp=None, hand_kd=None):
        if super().write(positions, hold_kp, hold_kd, weight, side=side,
                         selected_kp=teach_kp, selected_kd=teach_kd,
                         hand_kp=hand_kp, hand_kd=hand_kd) is not True:
            raise RuntimeError("rt/arm_sdk 命令发送失败")


def teach(interface, side, seconds, output, profile_path, execute=False, target_stage="record", *,
          takeover_seconds=3.0, soften_seconds=3.0):
    """只在显式执行时创建 DDS 发布者；输出为未验收的示教录制。"""
    if side not in ("left", "right") or not 1 <= seconds <= 30:
        raise ValueError("只支持 left/right 单臂示教，时长为 1–30 秒")
    if target_stage not in ("hold", "soften", "record"):
        raise ValueError("示教阶段必须是 hold、soften 或 record")
    for name, value in (("takeover_seconds", takeover_seconds), ("soften_seconds", soften_seconds)):
        if type(value) not in (int, float) or not math.isfinite(value) or not DT <= value <= 30:
            raise ValueError(f"{name} 必须为 {DT}–30 秒之间的有限数")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请换新文件名")
    profile = load_profile(profile_path, target_stage)
    if not execute:
        return {"execute": False, "side": side, "seconds": seconds,
                "profile": profile, "output": str(output), "stage": target_stage,
                "takeover_seconds": takeover_seconds, "soften_seconds": soften_seconds,
                "message": "只核对参数；没有连接机器人或发送命令"}
    return run_session(interface, side, output, profile, target_stage, seconds,
                       takeover_seconds, soften_seconds)


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
    from .collision import CollisionModel, find_path, parameterize_path
    model = CollisionModel(body_q)
    # 时间参数化沿已验证的路径线段采样，不改变空间路径。
    return parameterize_path(find_path(model, start, end, side), 0.5, 2.0)


def run_session(interface, side, output, profile, target_stage, seconds, takeover_seconds, soften_seconds):
    """三个阶段共用初始化、控制周期和渐变；完整录制额外执行避碰回位。"""
    from .trajectory import interpolate_positions, smoothstep
    from .constants import DEFAULT_MODEL
    from .replay import load_limits
    from .collision import CollisionModel
    recording_session = target_stage == "record"
    limits = load_limits(DEFAULT_MODEL) if recording_session else None
    stop, events = Event(), Queue()
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    port = None
    baseline = target = planned_body = None
    modes = EXPECTED_MOTOR_MODES
    selected = SIDE_OFFSETS[side]
    samples = []
    last_tick = None
    weight = 0.0
    control_attempted = False
    stage = "准备"
    pool = None
    primary_error = None
    cleanup_errors = []
    trial_metrics = {}
    last_kp, last_kd = profile["hold_kp"], profile["hold_kd"]
    last_hand = (profile["hand_hold_kp"], profile["hand_hold_kd"])

    def observe(state):
        metrics = trial_metrics.setdefault(stage, {"samples": 0,
            "max_speed_rad_s": 0.0, "max_start_offset_rad": 0.0,
            "max_target_error_rad": 0.0})
        metrics["samples"] += 1
        for key, value in (
                ("max_speed_rad_s", max(map(abs, state["dq"]))),
                ("max_start_offset_rad", max(abs(a-b) for a,b in zip(state["q"], baseline))),
                ("max_target_error_rad", max(abs(a-b) for a,b in zip(state["q"], target)))):
            metrics[key] = max(metrics[key], value)

    def tick(kp, kd, w, follow=False, capture=False, hand_gains=None, stopping=False):
        nonlocal target, weight, last_tick, control_attempted, last_kp, last_kd, last_hand
        if stop.is_set() and not stopping:
            raise RuntimeError("操作员已请求停止")
        now = time.monotonic()
        if recording_session and last_tick is not None and now - last_tick > 0.1:
            raise RuntimeError("示教控制周期超时")
        state = validate_snapshot(port.snapshot(), modes)
        if limits is not None and any(not low <= q <= high for q, (low, high) in zip(state["q"], limits)):
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
        if recording_session and max(abs(a - b) for a, b in zip(state["q"], target)) > 0.15:
            raise RuntimeError("示教保持或回位跟踪误差超限")
        weight, last_tick = w, now
        if hand_gains is None:
            hand_gains = (profile["hand_teach_kp"], profile["hand_teach_kd"]) if follow else (profile["hand_hold_kp"], profile["hand_hold_kd"])
        if not recording_session:
            observe(state)
        last_kp, last_kd, last_hand = kp, kd, hand_gains
        control_attempted = True
        port.write(target, profile["hold_kp"], profile["hold_kd"], kp, kd, side, w, *hand_gains)
        if capture:
            samples.append({"t": round(now - capture_start, 6), "q": list(state["q"]),
                            "dq": list(state["dq"]), "mode_pr": state["mode_pr"],
                            "mode_machine": state["mode_machine"]})
        return state

    def ramp(seconds, w0, w1, kp0, kp1, kd0, kd1, follow=False, hand_start=None, hand_end=None):
        hand_start = hand_start or (profile["hand_hold_kp"], profile["hand_hold_kd"])
        hand_end = hand_end or hand_start
        began = time.monotonic()
        steps = max(1, round(seconds / DT))
        for i in range(steps + 1):
            blend = smoothstep(i / steps)
            tick(kp0 + (kp1-kp0)*blend, kd0 + (kd1-kd0)*blend,
                 w0 + (w1-w0)*blend, follow=follow,
                 hand_gains=tuple(a+(b-a)*blend for a,b in zip(hand_start,hand_end)))
            if i < steps:
                time.sleep(max(0, began + (i+1)*DT - time.monotonic()))

    try:
        port = SdkArmPort(interface)
        if not port.ready.wait(5):
            raise RuntimeError("等待初始状态超时")
        time.sleep(3)
        if port.observed_commands():
            raise RuntimeError("观测到其他手臂命令；不接管")
        port.check_services()
        state = validate_snapshot(port.snapshot(), modes)
        baseline = tuple(state["q"])
        target = list(baseline)
        if recording_session:
            if max(map(abs, state["dq"])) > 0.1:
                raise RuntimeError("初始手臂尚未静止")
            # 初始姿态须在模型范围内且无本体碰撞，才能开始接管。
            if state.get("body_q") is None:
                raise RuntimeError("避碰示教需要腿/腰姿态反馈")
            check = CollisionModel(state["body_q"]).clearance(baseline, 0.02)
            if not check["valid"]:
                raise ValueError("初始姿态碰撞或余量不足：{}".format(check))
        for sig in previous:
            signal.signal(sig, lambda *_: stop.set())
        port.hand_profile = profile
        port.start_publisher()
        stage = "接管"
        ramp(takeover_seconds, 0, 1, profile["hold_kp"], profile["hold_kp"], profile["hold_kd"], profile["hold_kd"])
        if target_stage == "hold":
            stage = "固定位置保持"
            ramp(seconds, 1, 1, profile["hold_kp"], profile["hold_kp"],
                 profile["hold_kd"], profile["hold_kd"])
        else:
            stage = "降低示教刚度"
            ramp(soften_seconds, 1, 1, profile["hold_kp"], profile["teach_kp"], profile["hold_kd"], profile["teach_kd"], True,
                 hand_end=(profile["hand_teach_kp"], profile["hand_teach_kd"]))
        if not recording_session:
            return {"stage": target_stage, "qualification": "unqualified_hardware_trial",
                    "recording_written": False, "trial_metrics": trial_metrics,
                    "message": "已发送释放命令；实际交接仍须现场确认"}
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
        ramp(1, 1, 1, profile["teach_kp"], profile["hold_kp"], profile["teach_kd"], profile["hold_kd"],
             hand_start=(profile["hand_teach_kp"], profile["hand_teach_kd"]),
             hand_end=(profile["hand_hold_kp"], profile["hand_hold_kd"]))
        state = validate_snapshot(port.snapshot(), modes)
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
        primary_error = error
    finally:
        # 调试阶段保留原有的原位恢复增益/释放；完整录制必须先避碰回位。
        if not recording_session and weight > 0:
            try:
                target = list(validate_snapshot(port.snapshot(), modes)["q"])
                stage = "释放"
                kp0, kd0, hand0, w0 = last_kp, last_kd, last_hand, weight
                for step in range(101):
                    blend = min(step / 50, 1.0)
                    tick(kp0+(profile["hold_kp"]-kp0)*blend,
                         kd0+(profile["hold_kd"]-kd0)*blend,
                         w0 if step <= 50 else w0*(1-(step-50)/50),
                         hand_gains=tuple(a+(b-a)*blend for a,b in zip(hand0,
                             (profile["hand_hold_kp"], profile["hand_hold_kd"]))),
                         stopping=True)
                    if step < 100:
                        time.sleep(DT)
            except Exception as error:
                cleanup_errors.append(f"释放失败：{error}")
        stop.set()
        if pool is not None:
            pool.shutdown(wait=False)
        if port is not None:
            try:
                port.close()
            except Exception as error:
                cleanup_errors.append(str(error))
        for sig, handler in previous.items():
            try:
                signal.signal(sig, handler)
            except Exception as error:
                cleanup_errors.append(f"恢复信号处理失败：{error}")
        if cleanup_errors:
            message = "；".join(cleanup_errors) + "；未保存可用记录，请现场接管"
            if primary_error is not None:
                message = f"示教失败：{primary_error}；{message}"
            raise RuntimeError(message) from primary_error
        if primary_error is not None:
            if recording_session and control_attempted:
                raise RuntimeError(f"示教在{stage}阶段停止：{primary_error}；未自动释放，请现场接管") from primary_error
            raise primary_error
    if len(samples) < 3:
        raise RuntimeError("录制至少需要三帧；已回位释放，未保存录制")
    payload = {"schema": RECORDING_SCHEMA, "qualification": "raw_observation_not_executable",
               "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS), "joint_names": list(JOINT_NAMES),
               "recorded_at_utc": datetime.now(timezone.utc).isoformat(), "teaching_side": side,
               "teaching_profile": profile, "baseline_q": list(baseline),
               "messages_received": len(samples), "invalid_messages": 0, "samples": samples}
    write_json(payload, output)
    return {"path": str(output), "samples": len(samples), "release_sent": True,
            "hand_release_sent": False,
            "hand_finish": "已发送初始手型保持目标；关闭发布器不证明手指已卸力或保持",
            "duration_s": samples[-1]["t"], "qualification": payload["qualification"]}
