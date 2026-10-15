"""录制、单个和组合动作的回放入口；execute=True 才初始化 DDS。"""

import signal
import time
from pathlib import Path
from threading import Event

from .constants import ARM_SLOTS, EXPECTED_MOTOR_MODES, PROFILE
from .author import load_action
from .executor import execute_replay
from .config import dump_json, load_replay_profile
from .replay import create_plan
from .replay_sdk import ReplaySdkPort
from .teach import validate_snapshot


def _make_sdk(interface):
    from .unitree_binding import UnitreeReplaySDK
    sdk = UnitreeReplaySDK()
    sdk.initialize_network(interface, allow_network=True)
    sdk.initialize_service_clients()
    return sdk


def _baseline(q, provenance, body_q=None):
    return {"schema": "g1-arm7-replay-baseline-v1", "target_profile": PROFILE,
            "arm_slots": list(ARM_SLOTS), "q": list(q), "provenance": provenance,
            **({"body_q": list(body_q)} if body_q is not None else {})}


def _plan(action, baseline, profile, model, speed_scale, side, error_prefix="离线轨迹"):
    plan = create_plan(
        action, baseline, model,
        profile["max_target_velocity_rad_s"], profile["max_target_acceleration_rad_s2"],
        speed_scale=speed_scale, side=side, collision_check=True,
    )
    if plan["status"] != "offline_numerical_pass":
        raise ValueError("{}筛查失败：{}".format(error_prefix, plan["violations"]))
    return plan


def _execute(interface, action, profile, model, speed_scale, side, stop):
    port = ReplaySdkPort(_make_sdk(interface), time)
    try:
        if not port.wait_ready(5.0, stop):
            raise RuntimeError("等待初始状态超时或操作员已请求停止")
        state = validate_snapshot(port.snapshot(), EXPECTED_MOTOR_MODES)
        # 接管和最终回位均使用本次反馈，另一侧全程保持此位置。
        if state.get("body_q") is None:
            raise RuntimeError("避碰回放需要腿/腰的当前姿态反馈")
        baseline = _baseline(state["q"], "本次执行前 rt/lowstate 实测姿态", state.get("body_q"))
        plan = _plan(action, baseline, profile, model, speed_scale, side,
                     error_prefix="当前姿态生成的回放计划")
        print("现场规划完成：{} 臂单次回放，目标序列 {:.2f} 秒；先执行接管前预检。".format(
            side, plan["metrics"]["duration_s"]), flush=True)
        report = execute_replay(plan, profile, lambda: port, time, stop)
        report["plan"] = plan
        return report
    except BaseException as error:
        # 执行器负责正常/异常收尾；这里补上规划失败时的资源清理。
        try:
            port.close()
        except Exception as cleanup_error:
            raise RuntimeError("{}；关闭失败：{}".format(error, cleanup_error)) from error
        raise


def replay(interface, source, profile_path, model, output, side=None,
           speed_scale=0.5, execute=False):
    sources = [Path(p) for p in source] if isinstance(source, (list, tuple)) else [Path(source)]
    if not sources:
        raise ValueError("至少需要一个动作")
    actions = [load_action(path) for path in sources]
    if execute and any(action["synthetic"] for action in actions):
        raise ValueError("合成示例只供离线学习，不允许真机执行")
    side = side or next((action["side"] for action in actions if action["kind"] == "sequence"), "left")
    if side not in ("left", "right"):
        raise ValueError("回放侧必须为 left 或 right")
    if any(action["kind"] == "sequence" and side != action["side"] for action in actions):
        raise ValueError("组合动作只能按制作时的回放侧执行")
    profile = load_replay_profile(profile_path)
    # 预览用首帧；执行只根据当前反馈规划一次，在发布命令前自动检查。
    baseline = _baseline(actions[0]["frames"][0]["q"], "录制首帧，仅供离线预览")
    plan = None if execute else _plan(actions, baseline, profile, model, speed_scale, side)
    stop = Event()
    # 提前独占新报告路径，避免发送指令后才发现输出文件不能创建。
    with Path(output).open("x", encoding="utf-8") as stream:
        if execute:
            previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
            try:
                for signum in previous:
                    signal.signal(signum, lambda _signum, _frame: stop.set())
                report = _execute(interface, actions, profile, model, speed_scale, side, stop)
            except (Exception, KeyboardInterrupt) as error:
                report = {"status": "failed", "qualification": "unqualified_executor_trial",
                          "errors": [{"message": str(error) or type(error).__name__}],
                          "release_sent": False, "handover_confirmed": False}
            finally:
                for signum, handler in previous.items():
                    signal.signal(signum, handler)
        else:
            report = {"status": "preview", "qualification": "offline_only_not_executable", "plan": plan}
        report.update(hardware_enabled=execute, side=side, speed_scale=speed_scale,
                      source_files=[path.name for path in sources], execution_profile=profile)
        dump_json(report, stream)
    return {"path": str(output), "status": report["status"], "hardware_enabled": execute,
            "qualification": report["qualification"], "side": side,
            "speed_scale": speed_scale,
            "metrics": report.get("metrics", report.get("plan", {}).get("metrics", {})),
            "release_sent": report.get("release_sent", False),
            "hand_release_sent": report.get("hand_release_sent", False),
            "recovery_release_sent": report.get("recovery_release_sent", False),
            "requires_operator": report.get("requires_operator", False),
            "handover_confirmed": report.get("handover_confirmed", False),
            "plan_metrics": report.get("plan", {}).get("metrics", {}),
            "errors": report.get("errors", []), "cleanup_errors": report.get("cleanup_errors", [])}
