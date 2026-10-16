"""G1 动作预检、示教、制作和回放的命令行入口。"""

import argparse
import sys

from . import __version__
from .author import load_action
from .config import dump_json, read_json, write_json


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="g1-action-studio",
        description="G1 Arm7 动作制作工具；profile 选择仅手臂或 Dex3-1，--execute 才发送所选命令",
    )
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)

    teach = commands.add_parser("teach", help="Arm7 手把手示教候选；默认只检查参数")
    teach.add_argument("interface", help="DDS 网卡，例如 eth0")
    teach.add_argument("--side", choices=("left", "right"), required=True)
    teach.add_argument("--output", required=True, help="保存动作文件")
    teach.add_argument("--profile", required=True, help="控制布局与保持/示教增益 JSON")
    teach.add_argument("--execute", action="store_true", help="显式开启真机控制与录制")

    check = commands.add_parser("preflight", help="只读保存运控与状态消息观测报告")
    check.add_argument("interface", help="DDS 网卡，例如 eth0")
    check.add_argument("--seconds", type=int, default=10)
    check.add_argument("--output", required=True)

    build = commands.add_parser("build", help="将整段示教录制转换并保存为统一动作文件")
    build.add_argument("source", help="teach 生成的示教录制 JSON")
    build.add_argument("--code", default="recorded.action", help="语义动作代码")
    build.add_argument("--version", type=int, default=1)
    build.add_argument("--name")
    build.add_argument("--output", required=True)

    playback = commands.add_parser("replay", help="单个或组合动作回放；默认预览，--execute 才连接真机")
    playback.add_argument("interface", help="DDS 网卡，例如 eth0")
    playback.add_argument("source", nargs="+", help="按播放顺序填写一个或多个录制/动作 JSON")
    playback.add_argument("--side", choices=("left", "right"), help="默认使用组合动作的回放侧；其他输入默认 left")
    playback.add_argument("--profile", required=True, help="回放候选参数 JSON")
    playback.add_argument(
        "--speed-scale", type=float, default=0.5, help="平均播放速度比例，默认 0.5：约两倍时长；单个动作平滑两端"
    )
    playback.add_argument("--output", required=True, help="新报告文件；包含本次规划和执行反馈")
    playback.add_argument("--execute", action="store_true", help="显式启用单次真机回放")

    transition = commands.add_parser("transition", help="生成动作 A → 避碰过渡 → 动作 B，可交给 replay 播放")
    transition.add_argument("before", help="动作 A 的录制或统一动作 JSON")
    transition.add_argument("after", help="动作 B 的录制或统一动作 JSON")
    transition.add_argument("--side", choices=("left", "right"), default="left")
    transition.add_argument("--speed-scale", type=float, default=1.0, help="默认保留原动作时长")
    transition.add_argument("--clearance", type=float, default=0.02, help="非安装区避碰余量，单位米")
    transition.add_argument("--max-velocity", type=float, default=0.5)
    transition.add_argument("--max-acceleration", type=float, default=2.0)
    transition.add_argument("--body-pose", help="固定腿/腰姿态 JSON：J0–J14 的 15 个弧度值；默认参考零位")
    transition.add_argument("--output", required=True)

    args = parser.parse_args(argv)
    try:
        if args.command == "teach":
            from .teach import teach

            summary = teach(args.interface, args.side, args.output, args.profile, args.execute)
        elif args.command == "preflight":
            from .preflight import preflight

            summary = preflight(args.interface, args.seconds, args.output)
        elif args.command == "build":
            action = load_action(args.source, code=args.code, version=args.version, name=args.name)
            write_json(action, args.output)
            summary = {
                "path": args.output,
                "kind": action["kind"],
                "trajectory_sha256": action["trajectory_sha256"],
                "metrics": action["metrics"],
            }
        elif args.command == "replay":
            from .live_replay import replay

            summary = replay(
                args.interface,
                args.source,
                profile_path=args.profile,
                output=args.output,
                side=args.side,
                speed_scale=args.speed_scale,
                execute=args.execute,
            )
        elif args.command == "transition":
            from .transition import plan_transition

            body_q = None if args.body_pose is None else read_json(args.body_pose)
            summary = plan_transition(
                args.before,
                args.after,
                args.output,
                args.side,
                args.speed_scale,
                args.clearance,
                args.max_velocity,
                args.max_acceleration,
                body_q,
            )
            summary["path"] = args.output
    except (OSError, ValueError, RuntimeError, ImportError, KeyError, TypeError) as error:
        print("失败：{}".format(error), file=sys.stderr)
        return 1
    dump_json(summary, sys.stdout, allow_nan=True)
    if args.command == "preflight" and summary["errors"]:
        return 1
    if args.command == "replay" and summary["status"] not in ("preview", "completed"):
        return 1
    return 0
