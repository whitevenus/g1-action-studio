"""开发验证入口，与日常录制/回放命令分开。"""

import argparse
import json
import sys
from pathlib import Path

from .author import write_draft
from .replay import DEFAULT_MODEL, plan_replay


def main(argv=None):
    parser = argparse.ArgumentParser(prog="g1-action-studio-dev", description="离线开发验证；不发送真机指令")
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan-replay", help="生成完整离线序列并筛查数值")
    plan.add_argument("draft")
    plan.add_argument("--baseline", required=True)
    plan.add_argument("--model", default=str(DEFAULT_MODEL))
    plan.add_argument("--max-velocity", type=float, required=True)
    plan.add_argument("--max-acceleration", type=float, required=True)
    plan.add_argument("--takeover-seconds", type=float, default=3.0)
    plan.add_argument("--enter-seconds", type=float, default=5.0)
    plan.add_argument("--return-seconds", type=float, default=5.0)
    plan.add_argument("--release-seconds", type=float, default=2.0)
    plan.add_argument("--output", required=True)
    simulation = commands.add_parser("simulate-replay", help="用内存 SDK 验证正常、取消及故障流程")
    simulation.add_argument("plan")
    simulation.add_argument("--scenario", choices=("normal", "tracking-error", "state-timeout", "mode-change",
                             "send-failure", "cancel", "release-failure"), default="normal")
    simulation.add_argument("--profile")
    simulation.add_argument("--output", required=True)
    audit = commands.add_parser("audit-unitree-replay", help="真实 SDK 离线报文验证，阻断网络")
    audit.add_argument("plan")
    audit.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        if Path(args.output).exists():
            raise FileExistsError("输出文件已存在，请换新路径")
        if args.command == "plan-replay":
            report = plan_replay(args.draft, args.baseline, args.model, args.max_velocity,
                                 args.max_acceleration, args.takeover_seconds, args.enter_seconds,
                                 args.return_seconds, args.release_seconds)
        else:
            plan = json.loads(Path(args.plan).read_text(encoding="utf-8-sig"))
            if args.command == "simulate-replay":
                from .simulated_sdk import simulate_replay
                profile = None if args.profile is None else json.loads(Path(args.profile).read_text(encoding="utf-8-sig"))
                report = simulate_replay(plan, args.scenario, profile)
            else:
                from .unitree_audit import audit_unitree_replay
                report = audit_unitree_replay(plan)
        write_draft(report, args.output)
    except (OSError, ValueError, RuntimeError, ImportError, KeyError, TypeError) as error:
        print("失败：{}".format(error), file=sys.stderr)
        return 1
    summary = {key: report[key] for key in ("status", "qualification", "hardware_enabled") if key in report}
    summary["path"] = args.output
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if report["status"] in ("offline_numerical_pass", "completed", "passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
