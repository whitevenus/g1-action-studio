"""G1 动作预检、录制、示教和回放的命令行入口。"""

import argparse
import json
import sys

from .constants import DEFAULT_MODEL
from .author import audit_draft, build_draft
from .config import read_json, write_json
from . import __version__


def main(argv=None):
    parser = argparse.ArgumentParser(prog="g1-action-studio", description="G1 Arm7 动作制作工具；teach/replay --execute 会发送手臂命令")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)

    capture = commands.add_parser("record", help="只读记录 rt/lowstate")
    capture.add_argument("interface", help="DDS 网卡，例如 eth0")
    capture.add_argument("--seconds", type=int, required=True)
    capture.add_argument("--output", required=True)

    teach = commands.add_parser("teach", help="Arm7 手把手示教候选；默认只检查参数")
    teach.add_argument("interface", help="DDS 网卡，例如 eth0")
    teach.add_argument("--side", choices=("left", "right"), required=True)
    teach.add_argument("--seconds", type=int, required=True, help="hold 的固定保持时长，或 record 的录制时长")
    teach.add_argument("--output", required=True)
    teach.add_argument("--profile", required=True, help="本机候选/验收参数 JSON；hold 只需保持增益")
    teach.add_argument("--stage", choices=("hold", "soften", "record"), default="hold",
                       help="默认只试验原位保持；record 才会录制手摆动作")
    teach.add_argument("--execute", action="store_true", help="显式开启真机控制与录制")

    check = commands.add_parser("preflight", help="只读保存运控与状态消息观测报告")
    check.add_argument("interface", help="DDS 网卡，例如 eth0")
    check.add_argument("--seconds", type=int, default=10)
    check.add_argument("--output", required=True)

    build = commands.add_parser("build", help="从状态记录制作未验收轨迹草稿")
    build.add_argument("source", help="本工具录制或现有预置动作只读记录 JSON")
    build.add_argument("--start", type=float, required=True, help="源记录中的开始秒数")
    build.add_argument("--end", type=float, required=True, help="源记录中的结束秒数")
    build.add_argument("--code", required=True, help="语义动作代码")
    build.add_argument("--version", type=int, required=True)
    build.add_argument("--name", required=True)
    build.add_argument("--output", required=True)

    audit = commands.add_parser("audit", help="离线核对草稿完整性与轨迹指标")
    audit.add_argument("draft")

    playback = commands.add_parser("replay", help="单臂示教回放；默认离线预览，--execute 才连接真机")
    playback.add_argument("interface", help="DDS 网卡，例如 eth0")
    playback.add_argument("source", help="原始状态录制 JSON")
    playback.add_argument("--side", choices=("left", "right"), default="left")
    playback.add_argument("--profile", required=True, help="回放候选参数 JSON")
    playback.add_argument("--model", default=str(DEFAULT_MODEL), help="本机配置对应的关节范围 XML")
    playback.add_argument("--speed-scale", type=float, default=0.5, help="平均播放速度比例，默认 0.5：约两倍时长，两端渐变")
    playback.add_argument("--output", required=True, help="新报告文件；包含本次规划和执行反馈")
    playback.add_argument("--execute", action="store_true", help="显式启用单次真机回放")

    transition = commands.add_parser("transition", help="离线生成动作 A → 避碰过渡 → 动作 B；不连接真机")
    transition.add_argument("before", help="动作 A 的录制或草稿 JSON")
    transition.add_argument("after", help="动作 B 的录制或草稿 JSON")
    transition.add_argument("--side", choices=("left", "right"), default="left")
    transition.add_argument("--speed-scale", type=float, default=0.5)
    transition.add_argument("--clearance", type=float, default=0.02, help="非安装区避碰余量，单位米")
    transition.add_argument("--max-velocity", type=float, default=0.5)
    transition.add_argument("--max-acceleration", type=float, default=2.0)
    transition.add_argument("--body-pose", help="固定腿/腰姿态 JSON：J0–J14 的 15 个弧度值；默认参考零位")
    transition.add_argument("--seed", type=int, default=0)
    transition.add_argument("--output", required=True)

    args = parser.parse_args(argv)
    try:
        if args.command == "record":
            from .record import record
            summary = record(args.interface, args.seconds, args.output)
        elif args.command == "teach":
            from .teach import teach
            summary = teach(args.interface, args.side, args.seconds,
                            args.output, args.profile, args.execute, args.stage)
        elif args.command == "preflight":
            from .preflight import preflight
            summary = preflight(args.interface, args.seconds, args.output)
        elif args.command == "build":
            draft = build_draft(args.source, args.start, args.end,
                                args.code, args.version, args.name)
            write_json(draft, args.output)
            summary = {"path": args.output, "qualification": draft["qualification"],
                       "trajectory_sha256": draft["trajectory_sha256"],
                       "metrics": draft["metrics"]}
        elif args.command == "audit":
            summary = {"qualification": "draft_unqualified_not_executable",
                       "metrics": audit_draft(args.draft)}
        elif args.command == "replay":
            from .live_replay import replay
            summary = replay(args.interface, args.source, args.profile, args.model,
                             args.output, args.side, args.speed_scale, args.execute)
        elif args.command == "transition":
            from .transition import plan_transition
            body_q = None if args.body_pose is None else read_json(args.body_pose)
            summary = plan_transition(args.before, args.after, args.output, args.side, args.speed_scale,
                                      args.clearance, args.max_velocity, args.max_acceleration, args.seed, body_q)
            summary["path"] = args.output
    except (OSError, ValueError, RuntimeError, ImportError, KeyError, TypeError) as error:
        print("失败：{}".format(error), file=sys.stderr)
        return 1
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.command == "preflight" and summary["errors"]:
        return 1
    if args.command == "replay" and summary["status"] not in ("preview", "completed"):
        return 1
    return 0
