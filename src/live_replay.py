"""单次单臂回放入口；只有 execute=True 才加载 SDK 并初始化 DDS。"""

import json
import signal
import time
from pathlib import Path
from threading import Event

from . import ARM_SLOTS, PROFILE
from .author import build_draft
from .executor import execute_replay, validate_profile
from .replay import create_plan
from .replay_sdk import ReplaySdkPort
from .teach import load_profile, validate_snapshot


def _make_sdk(interface):
    from .unitree_binding import UnitreeReplaySDK
    sdk = UnitreeReplaySDK()
    sdk.initialize_network(interface, allow_network=True)
    sdk.initialize_service_clients()
    return sdk


def _baseline(q, provenance):
    return {"schema": "g1-arm7-replay-baseline-v1", "target_profile": PROFILE,
            "arm_slots": list(ARM_SLOTS), "q": list(q), "provenance": provenance}


def _execute(interface, draft, profile, model, speed_scale, side, stop):
    port = ReplaySdkPort(_make_sdk(interface), time)
    try:
        if not port.wait_ready(5.0, stop):
            raise RuntimeError("等待初始状态超时或操作员已请求停止")
        state = port.snapshot()
        validate_snapshot(state, (1,) * 14)
        # 接管和最终回位均使用本次反馈，另一侧全程保持此位置。
        baseline = _baseline(state["q"], "本次执行前 rt/lowstate 实测姿态")
        plan = create_plan(draft, baseline, model,
                           profile["max_target_velocity_rad_s"], profile["max_target_acceleration_rad_s2"],
                           speed_scale=speed_scale, side=side)
        if plan["status"] != "offline_numerical_pass":
            raise ValueError("当前姿态生成的回放计划筛查失败：{}".format(plan["violations"]))
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


def replay(interface, source, profile_path, model, output, side="left",
           speed_scale=0.5, execute=False):
    source = Path(source)
    draft = build_draft(source, None, None, "trial." + side + "_replay", 1,
                        "单臂示教回放试验", allow_synthetic=not execute)
    if (draft["observed_mode_pr"], draft["observed_mode_machine"]) != (0, 5):
        raise ValueError("录制模式不符合 0/5")
    load_profile(profile_path, "hold")
    profile = validate_profile(json.loads(Path(profile_path).read_text(encoding="utf-8-sig")))
    # 先做离线筛查，拒绝不可用的轨迹，再允许导入真机 SDK。
    plan = create_plan(draft, _baseline(draft["frames"][0]["q"], "录制首帧，仅供离线预览"), model,
                       profile["max_target_velocity_rad_s"], profile["max_target_acceleration_rad_s2"],
                       speed_scale=speed_scale, side=side)
    if plan["status"] != "offline_numerical_pass":
        raise ValueError("离线轨迹筛查失败：{}".format(plan["violations"]))
    stop = Event()
    # 提前独占新报告路径，避免发送指令后才发现输出文件不能创建。
    with Path(output).open("x", encoding="utf-8") as stream:
        if execute:
            previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
            try:
                for signum in previous:
                    signal.signal(signum, lambda _signum, _frame: stop.set())
                report = _execute(interface, draft, profile, model, speed_scale, side, stop)
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
                      source_file=source.name, execution_profile=profile)
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    return {"path": str(output), "status": report["status"], "hardware_enabled": execute,
            "qualification": report["qualification"], "side": side,
            "speed_scale": speed_scale,
            "metrics": report.get("metrics", report.get("plan", {}).get("metrics", {})),
            "release_sent": report.get("release_sent", False),
            "recovery_release_sent": report.get("recovery_release_sent", False),
            "requires_operator": report.get("requires_operator", False),
            "handover_confirmed": report.get("handover_confirmed", False),
            "plan_metrics": report.get("plan", {}).get("metrics", {}),
            "errors": report.get("errors", []), "cleanup_errors": report.get("cleanup_errors", [])}
