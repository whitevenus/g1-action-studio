"""可注入端口和时钟的反馈回放执行器；没有自动加载真机 SDK 的入口。"""

import math

from .config import document_layout, normalize_document, validate_replay_profile
from .constants import (
    ARM_SLOTS,
    DT,
    PROFILE,
    get_layout,
)
from .replay import sample_schedule, screen_schedule
from .replay_sdk import validate_snapshot
from .trajectory import frame_digest


class Cancelled(RuntimeError):
    pass


def validate_plan(plan):
    layout = document_layout(plan, "plan")
    if (
        not isinstance(plan, dict)
        or plan.get("hardware_enabled") is not False
        or plan.get("qualification") != "offline_only_not_executable"
        or plan.get("status") != "offline_numerical_pass"
    ):
        raise ValueError("仅接受数值通过的离线 Arm7 回放计划")
    frames = plan["frames"]
    if len(frames) > 30001 or plan["schedule_sha256"] != frame_digest(frames):
        raise ValueError("回放序列长度或摘要无效")
    limits = plan["model"]["limits_rad"]
    if len(limits) != layout.joint_count or any(
        len(pair) != 2 or any(type(x) not in (int, float) or not math.isfinite(x) for x in pair) or pair[0] >= pair[1]
        for pair in limits
    ):
        raise ValueError("回放计划关节范围无效")
    screening = screen_schedule(
        frames,
        limits,
        plan["thresholds"]["max_velocity_rad_s"],
        plan["thresholds"]["max_acceleration_rad_s2"],
        layout=layout,
    )
    if screening["violations"]:
        raise ValueError("回放计划重新筛查失败")
    expected = ["takeover", "enter", "replay", "return", "release"]
    phases = []
    previous_weight = 0.0
    for frame in frames:
        phase, weight = frame["phase"], frame["weight"]
        if type(weight) not in (int, float) or not math.isfinite(weight) or not 0 <= weight <= 1:
            raise ValueError("回放权重无效")
        if not phases or phases[-1] != phase:
            phases.append(phase)
        if phase == "takeover" and (weight < previous_weight or frame["q"] != frames[0]["q"]):
            raise ValueError("接管须保持原位并递增权重")
        if phase in ("enter", "replay", "return") and weight != 1:
            raise ValueError("运动阶段权重须为 1")
        if phase == "release" and (weight > previous_weight or frame["q"] != frames[0]["q"]):
            raise ValueError("释放须保持基线并递减权重")
        previous_weight = weight
    if (
        phases != expected
        or frames[0]["weight"] != 0
        or frames[-1]["weight"] != 0
        or frames[-1]["q"] != frames[0]["q"]
        or frames[0]["q"] != plan["baseline"]["q"]
    ):
        raise ValueError("回放阶段或起止姿态无效")
    return frames, limits


def execute_replay(plan, profile, port_factory, clock, stop):
    """端口负责 SDK 边界；运行结果只报告发送和反馈，不宣称控制权已交接。"""
    profile = validate_replay_profile(profile)
    plan = normalize_document(plan, "plan")
    frames, limits = validate_plan(plan)
    layout = document_layout(plan, "plan")
    if get_layout(profile.get("target_profile", PROFILE)) != layout:
        raise ValueError("回放参数与计划布局不匹配")
    times = [frame["t"] for frame in frames]
    release_index = next(i for i, frame in enumerate(frames) if frame["phase"] == "release")
    return_end = times[release_index - 1]
    baseline = tuple(frames[0]["q"])
    port = None
    stage = "preflight"
    logs, errors, cleanup_errors = [], [], []
    status = "failed"
    last_q = baseline
    last_weight = attempted_weight = 0.0
    last_send_at = None
    last_tick_at = None
    last_state_at = None
    recovery_release_sent = release_sent = False
    control_attempted = False
    peak_error = 0.0
    began = clock.monotonic()

    def feedback():
        nonlocal last_state_at
        state = validate_snapshot(port.snapshot(), clock.monotonic(), profile["state_timeout_s"], layout=layout)
        at = state["at"]
        if last_state_at is not None and at < last_state_at:
            raise RuntimeError("状态时间戳倒退")
        if any(q < low or q > high for q, (low, high) in zip(state["q"], limits)):
            raise RuntimeError("反馈角度超出模型范围")
        if plan.get("body_pose") is not None:
            body = state.get("body_q")
            if (
                not isinstance(body, (tuple, list))
                or len(body) != ARM_SLOTS[0]
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in body)
            ):
                raise RuntimeError("缺少有效的腿/腰姿态反馈")
            if max(abs(a - b) for a, b in zip(body, plan["body_pose"])) > 0.02:
                raise RuntimeError("腿/腰姿态已偏离本次碰撞规划；停止回放")
        if "max_speed_rad_s" in profile and max(map(abs, state["dq"])) > profile["max_speed_rad_s"]:
            raise RuntimeError("反馈速度超限")
        if (
            "max_offset_rad" in profile
            and max(abs(q - initial) for q, initial in zip(state["q"], baseline)) > profile["max_offset_rad"]
        ):
            raise RuntimeError("反馈偏移超限")
        last_state_at = at
        return state

    def send(q, weight, phase, *, recovery=False):
        nonlocal last_q, last_weight, attempted_weight, last_send_at, last_tick_at, control_attempted, peak_error, stage
        stage = phase
        now = clock.monotonic()
        if not recovery and stop.is_set():
            raise Cancelled("已请求取消回放")
        if last_tick_at is not None and not 0 <= now - last_tick_at <= profile["max_tick_gap_s"] + 1e-9:
            raise RuntimeError("控制周期超时")
        last_tick_at = now
        state = feedback()
        error = max(abs(actual - target) for actual, target in zip(state["q"], last_q))
        peak_error = max(peak_error, error)
        if error > profile["max_tracking_error_rad"]:
            raise RuntimeError("跟踪误差超限：{:.6f} rad".format(error))
        entry = {
            "t": round(now - began, 6),
            "phase": phase,
            "state_at": state["at"],
            "q": list(state["q"]),
            "dq": list(state["dq"]),
            "target_q": list(q),
            "weight": weight,
            "tracking_error_rad": error,
            "write_confirmed": False,
        }
        logs.append(entry)
        # 发送失败也可能已送达，因此先记录尝试权重。
        attempted_weight = weight
        control_attempted = True  # 手部没有权重，首个零手臂权重周期也可能已控制手指。
        result = port.write(q, profile["hold_kp"], profile["hold_kd"], weight)
        if result is not True:
            raise RuntimeError("SDK Write 未确认成功")
        entry["write_confirmed"] = True
        last_q, last_weight, last_send_at = tuple(q), weight, now
        return state

    def run_segment(start, end):
        segment_started = clock.monotonic()
        while True:
            elapsed = min(end, start + clock.monotonic() - segment_started)
            if elapsed >= end - 1e-9:
                elapsed = end
            point = sample_schedule(frames, elapsed, times)
            send(point["q"], point["weight"], point["phase"])
            if elapsed >= end - 1e-9:
                break
            clock.sleep(min(DT, end - elapsed))

    try:
        if stop.is_set():
            raise Cancelled("启动前已取消")
        port = port_factory()
        port.check_services()
        if not port.wait_ready(5.0, stop):
            if stop.is_set():
                raise Cancelled("等待初始反馈期间已取消")
            raise RuntimeError("等待初始反馈超时，状态缺失")
        state = feedback()
        if max(abs(a - b) for a, b in zip(state["q"], baseline)) > profile["initial_pose_tolerance_rad"]:
            raise RuntimeError("当前姿态与规划基线不匹配；须重新规划")
        if max(map(abs, state["dq"])) > profile["return_speed_rad_s"]:
            raise RuntimeError("初始手臂或手指尚未静止")
        # 只读观察；零条命令仍不能证明独占控制权。
        observe_started = clock.monotonic()
        while clock.monotonic() - observe_started < 3 - 1e-9:
            if stop.is_set():
                raise Cancelled("预检期间已取消")
            feedback()
            clock.sleep(DT)
        if port.observed_commands():
            raise RuntimeError("观测到其他手臂或手指命令")
        port.check_services()
        state = feedback()
        if max(abs(a - b) for a, b in zip(state["q"], baseline)) > profile["initial_pose_tolerance_rad"]:
            raise RuntimeError("接管前姿态已偏离基线")
        if max(map(abs, state["dq"])) > profile["return_speed_rad_s"]:
            raise RuntimeError("接管前手臂或手指尚未静止")
        port.hand_profile = profile
        port.start_publisher()
        stage = "playback"
        run_segment(0, return_end)
        # 回位实际达到容差后才开始正常释放；要求连续 0.1 秒稳定。
        stage = "settle"
        settling = clock.monotonic()
        stable_since = None
        while True:
            clock.sleep(DT)
            state = send(baseline, 1.0, "settle")
            now = clock.monotonic()
            if (
                max(abs(a - b) for a, b in zip(state["q"], baseline)) <= profile["return_tolerance_rad"]
                and max(map(abs, state["dq"])) <= profile["return_speed_rad_s"]
                and state["at"] > last_send_at - DT - 1e-9
            ):
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 0.1 - 1e-9:
                    break
            else:
                stable_since = None
            if now - settling >= profile["settle_timeout_s"]:
                raise RuntimeError("实际回位等待超时")
        stage = "release"
        run_segment(return_end, times[-1])
        release_sent = True
        # 零权重后的新反馈只证明观测仍有效，不证明控制权归属。
        clock.sleep(DT)
        state = feedback()
        if state["at"] <= last_send_at:
            raise RuntimeError("释放后未收到新反馈")
        status = "completed"
    except (Exception, KeyboardInterrupt) as error:
        status = "cancelled" if isinstance(error, (Cancelled, KeyboardInterrupt)) else "failed"
        errors.append({"stage": stage, "message": str(error) or type(error).__name__})
        if port is not None and max(last_weight, attempted_weight) > 0:
            try:
                # 有效反馈下固定当前姿态释放，不试图沿原轨迹自动回位。
                state = feedback()
                last_q = tuple(state["q"])
                weight = max(last_weight, attempted_weight)
                last_tick_at = clock.monotonic()
                steps = math.ceil(profile["recovery_release_s"] / DT)
                for step in range(steps + 1):
                    send(last_q, weight * (1 - step / steps), "recovery_release", recovery=True)
                    if step < steps:
                        clock.sleep(DT)
                recovery_release_sent = True
            except (Exception, KeyboardInterrupt) as recovery_error:
                cleanup_errors.append("恢复释放失败：{}".format(recovery_error))
    finally:
        if port is not None:
            try:
                port.close()
            except Exception as error:
                cleanup_errors.append("关闭失败：{}".format(error))
                if status == "completed":
                    status = "failed"
    return {
        "schema": "g1-arm7-feedback-replay-report-v1",
        "status": status,
        "qualification": "unqualified_executor_trial",
        "control_owner": "unknown",
        "schedule_sha256": plan["schedule_sha256"],
        "profile": profile,
        "release_sent": release_sent,
        "recovery_release_sent": recovery_release_sent,
        "hand_release_sent": False,
        "target_profile": layout.profile_id,
        "controlled_joint_count": layout.joint_count,
        "hand_control_enabled": layout.hand_model is not None,
        "collision_hand_model": "dex3-1",
        "hand_collision_policy": "measured_joints" if layout.hand_model else "full_range_envelope",
        "hand_finish": "手部无权重释放接口；关闭发布器不证明手指已卸力或保持"
        if layout.hand_model
        else "手部控制未启用",
        "handover_confirmed": False,
        "requires_operator": bool(cleanup_errors) or (bool(errors) and control_attempted),
        "errors": errors,
        "cleanup_errors": cleanup_errors,
        "samples": logs,
        "metrics": {
            "command_attempts": len(logs),
            "confirmed_writes": sum(x["write_confirmed"] for x in logs),
            "max_tracking_error_rad": peak_error,
        },
    }
