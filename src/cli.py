"""G1 动作预检、示教、制作和回放的命令行入口。"""

import argparse
import json
import sys

from .constants import DEFAULT_MODEL
from .author import load_action
from .config import read_json, write_json
from . import __version__


def main(argv=None):
    parser = argparse.ArgumentParser(prog="g1-action-studio", description="G1 Arm7 + Dex3-1 动作制作工具；teach/replay --execute 会发送手臂和手指命令")
    parser.add_argument("--version", action="version", version=__version__)
    commands = parser.add_subparsers(dest="command", required=True)

    teach = commands.add_parser("teach", help="Arm7 手把手示教候选；默认只检查参数")
    teach.add_argument("interface", help="DDS 网卡，例如 eth0")
    teach.add_argument("--side", choices=("left", "right"), required=True)
    teach.add_argument("--seconds", type=int, default=3, help="hold 的保持时长；record 用回车开始和结束")
    teach.add_argument("--takeover-seconds", type=float, default=3.0, help="手臂权重从0到1的接管时长，默认3秒")
    teach.add_argument("--soften-seconds", type=float, default=3.0, help="示教侧手臂和手指降低刚度的时长，默认3秒")
    teach.add_argument("--output", required=True)
    teach.add_argument("--profile", required=True, help="本机候选/验收参数 JSON；hold 只需保持增益")
    teach.add_argument("--stage", choices=("hold", "soften", "record"), default="record",
                       help="默认回车控制示教录制；hold/soften 用于单独调试")
    teach.add_argument("--execute", action="store_true", help="显式开启真机控制与录制")

    check = commands.add_parser("preflight", help="只读保存运控与状态消息观测报告")
    check.add_argument("interface", help="DDS 网卡，例如 eth0")
    check.add_argument("--seconds", type=int, default=10)
    check.add_argument("--output", required=True)

    build = commands.add_parser("build", help="裁剪录制并保存统一动作文件")
    build.add_argument("source", help="teach 生成的示教录制 JSON")
    build.add_argument("--start", type=float, help="源记录中的开始秒数")
    build.add_argument("--end", type=float, help="源记录中的结束秒数")
    build.add_argument("--code", default="recorded.action", help="语义动作代码")
    build.add_argument("--version", type=int, default=1)
    build.add_argument("--name")
    build.add_argument("--output", required=True)

    playback = commands.add_parser("replay", help="单个或组合动作回放；默认预览，--execute 才连接真机")
    playback.add_argument("interface", help="DDS 网卡，例如 eth0")
    playback.add_argument("source", nargs="+", help="按播放顺序填写一个或多个录制/动作 JSON")
    playback.add_argument("--side", choices=("left", "right"), help="默认使用组合动作的回放侧；其他输入默认 left")
    playback.add_argument("--profile", required=True, help="回放候选参数 JSON")
    playback.add_argument("--model", default=str(DEFAULT_MODEL), help="本机配置对应的关节范围 XML")
    playback.add_argument("--speed-scale", type=float, default=0.5, help="平均播放速度比例，默认 0.5：约两倍时长；单个动作平滑两端")
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
            summary = teach(args.interface, args.side, args.seconds,
                            args.output, args.profile, args.execute, args.stage,
                            takeover_seconds=args.takeover_seconds, soften_seconds=args.soften_seconds)
        elif args.command == "preflight":
            from .preflight import preflight
            summary = preflight(args.interface, args.seconds, args.output)
        elif args.command == "build":
            action = load_action(args.source, args.start, args.end,
                                args.code, args.version, args.name)
            write_json(action, args.output)
            summary = {"path": args.output, "kind": action["kind"],
                       "trajectory_sha256": action["trajectory_sha256"],
                       "metrics": action["metrics"]}
        elif args.command == "replay":
            from .live_replay import replay
            summary = replay(args.interface, args.source, args.profile, args.model,
                             args.output, args.side, args.speed_scale, args.execute)
        elif args.command == "transition":
            from .transition import plan_transition
            body_q = None if args.body_pose is None else read_json(args.body_pose)
            summary = plan_transition(args.before, args.after, args.output, args.side, args.speed_scale,
                                      args.clearance, args.max_velocity, args.max_acceleration, body_q)
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
