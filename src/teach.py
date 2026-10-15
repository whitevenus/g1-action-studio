"""Arm7 示教：回车录制、保持终点、恢复刚度、避碰回位后释放。"""

import math
import signal
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Thread

from .config import load_teach_profile, write_json
from .constants import (
    ARM_SLOTS,
    COLLISION_MODEL,
    DT,
    JOINT_NAMES,
    PROFILE,
    RECORDING_SCHEMA,
    SIDE_OFFSETS,
    TEACH_SOFTEN_SECONDS,
    TEACH_TAKEOVER_SECONDS,
)
from .replay import load_limits
from .replay_sdk import ReplaySdkPort, validate_snapshot
from .trajectory import interpolate_positions, smoothstep
from .unitree_binding import create_sdk


def teach(interface, side, output, profile_path, execute=False):
    """核对参数；显式执行才连接机器人并进入完整示教流程。"""
    if side not in SIDE_OFFSETS:
        raise ValueError("示教侧必须为 left/right")
    output = Path(output)
    if output.exists():
        raise FileExistsError("输出文件已存在，请换新文件名")
    profile = load_teach_profile(profile_path)
    if not execute:
        return {
            "execute": False,
            "side": side,
            "profile": profile,
            "output": str(output),
            "message": "只核对参数；没有连接机器人或发送命令",
        }
    return run_session(interface, side, output, profile)


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


def run_session(interface, side, output, profile):
    """接管、降低刚度、回车录制、恢复刚度、避碰回位后释放。"""
    from .collision import CollisionModel

    limits = load_limits(COLLISION_MODEL)
    stop, events = Event(), Queue()
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    port = None
    baseline = target = planned_body = None
    selected = SIDE_OFFSETS[side]
    samples = []
    last_tick = None
    control_attempted = False
    stage = "准备"
    pool = None
    primary_error = None
    cleanup_errors = []
    hold_gains = tuple(profile[key] for key in ("hold_kp", "hold_kd", "hand_hold_kp", "hand_hold_kd"))
    teach_gains = tuple(profile[key] for key in ("teach_kp", "teach_kd", "hand_teach_kp", "hand_teach_kd"))

    def tick(gains=hold_gains, weight=1.0, *, follow=False, capture_start: float | None = None):
        nonlocal target, last_tick, control_attempted
        if stop.is_set():
            raise RuntimeError("操作员已请求停止")
        now = time.monotonic()
        if last_tick is not None and now - last_tick > 0.1:
            raise RuntimeError("示教控制周期超时")
        state = validate_snapshot(port.snapshot(), time.monotonic())
        if any(not low <= q <= high for q, (low, high) in zip(state["q"], limits)):
            raise RuntimeError("反馈角度超出关节范围")
        if planned_body is not None:
            body = state.get("body_q")
            if (
                body is None
                or len(body) != 15
                or any(not math.isfinite(v) for v in body)
                or max(abs(a - b) for a, b in zip(body, planned_body)) > 0.02
            ):
                raise RuntimeError("腿/腰姿态偏离回位规划")
        if follow:
            target = list(baseline)
            for i in selected:
                target[i] = state["q"][i]
        if max(abs(a - b) for a, b in zip(state["q"], target)) > 0.15:
            raise RuntimeError("示教保持或回位跟踪误差超限")
        last_tick = now
        kp, kd, hand_kp, hand_kd = gains
        control_attempted = True
        if (
            port.write(
                target,
                profile["hold_kp"],
                profile["hold_kd"],
                weight,
                side=side,
                selected_kp=kp,
                selected_kd=kd,
                hand_kp=hand_kp,
                hand_kd=hand_kd,
            )
            is not True
        ):
            raise RuntimeError("手臂命令发送失败；部分命令可能已送达")
        if capture_start is not None:
            samples.append(
                {
                    "t": round(now - capture_start, 6),
                    "q": list(state["q"]),
                    "dq": list(state["dq"]),
                    "mode_pr": state["mode_pr"],
                    "mode_machine": state["mode_machine"],
                }
            )
        return state

    def ramp(seconds, w0, w1, start_gains=hold_gains, end_gains=hold_gains, *, follow=False):
        began = time.monotonic()
        steps = max(1, round(seconds / DT))
        for i in range(steps + 1):
            blend = smoothstep(i / steps)
            gains = tuple(a + (b - a) * blend for a, b in zip(start_gains, end_gains))
            tick(gains, w0 + (w1 - w0) * blend, follow=follow)
            if i < steps:
                time.sleep(max(0, began + (i + 1) * DT - time.monotonic()))

    def record():
        """输入线程只通知两次回车；本循环负责录制期间的保持和采样。"""
        nonlocal stage
        capture_start = None
        while True:
            try:
                event = events.get_nowait()
            except Empty:
                event = None
            if isinstance(event, Exception):
                raise RuntimeError("录制输入已关闭") from event
            if event == "enter":
                if capture_start is not None:
                    tick(teach_gains, follow=True, capture_start=capture_start)
                    return
                capture_start = time.monotonic()
                stage = "录制"
                print("RECORDING_STARTED", flush=True)
            tick(teach_gains, follow=True, capture_start=capture_start)
            time.sleep(DT)

    try:
        port = ReplaySdkPort(create_sdk(interface), time)
        if not port.wait_ready(5, stop):
            raise RuntimeError("等待初始状态超时")
        time.sleep(3)
        if port.observed_commands():
            raise RuntimeError("观测到其他手臂或手指命令；不接管")
        port.check_services()
        state = validate_snapshot(port.snapshot(), time.monotonic())
        baseline = tuple(state["q"])
        target = list(baseline)
        if max(map(abs, state["dq"])) > 0.1:
            raise RuntimeError("初始手臂或手指尚未静止")
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
        ramp(TEACH_TAKEOVER_SECONDS, 0, 1)
        stage = "降低示教刚度"
        ramp(TEACH_SOFTEN_SECONDS, 1, 1, hold_gains, teach_gains, follow=True)
        Thread(target=read_record_keys, args=(events, stop), daemon=True).start()
        stage = "等待开始"
        record()
        # 最后一帧已冻结 target；恢复增益时不再追随手动移动。
        stage = "恢复刚度"
        ramp(1, 1, 1, teach_gains, hold_gains)
        state = validate_snapshot(port.snapshot(), time.monotonic())
        planned_body = state.get("body_q")
        if planned_body is None:
            raise RuntimeError("避碰回位需要腿/腰姿态反馈")
        stage = "规划回位"
        pool = ThreadPoolExecutor(max_workers=1)
        future = pool.submit(plan_return, list(target), baseline, side, list(planned_body))
        while not future.done():
            tick()
            time.sleep(DT)
        motion = future.result()
        stage = "回位"
        began = time.monotonic()
        while True:
            elapsed = min(motion[-1]["t"], time.monotonic() - began)
            # 返回路径只含 q/t，直接使用公共位置插值。
            index = min(int(elapsed / DT), len(motion) - 1)
            after = min(index + 1, len(motion) - 1)
            target = interpolate_positions(motion[index]["q"], motion[after]["q"], min(1, max(0, elapsed / DT - index)))
            tick()
            if elapsed >= motion[-1]["t"]:
                break
            time.sleep(DT)
        target = list(baseline)
        stage = "等待回位稳定"
        began, stable = time.monotonic(), None
        while True:
            state = tick()
            now = time.monotonic()
            if max(abs(a - b) for a, b in zip(state["q"], baseline)) <= 0.05 and max(map(abs, state["dq"])) <= 0.1:
                stable = now if stable is None else stable
                if now - stable >= 0.1:
                    break
            else:
                stable = None
            if now - began > 5:
                raise RuntimeError("实际回位等待超时")
            time.sleep(DT)
        stage = "释放"
        ramp(2, 1, 0)
    except BaseException as error:
        primary_error = error
    finally:
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
            if control_attempted:
                raise RuntimeError(f"示教在{stage}阶段停止：{primary_error}；未自动释放，请现场接管") from primary_error
            raise primary_error
    if len(samples) < 3:
        raise RuntimeError("录制至少需要三帧；已回位释放，未保存录制")
    payload = {
        "schema": RECORDING_SCHEMA,
        "qualification": "raw_observation_not_executable",
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "teaching_side": side,
        "teaching_profile": profile,
        "baseline_q": list(baseline),
        "samples": samples,
    }
    write_json(payload, output)
    return {
        "path": str(output),
        "samples": len(samples),
        "release_sent": True,
        "hand_release_sent": False,
        "hand_finish": "已发送初始手型保持目标；关闭发布器不证明手指已卸力或保持",
        "duration_s": samples[-1]["t"],
        "qualification": payload["qualification"],
    }
