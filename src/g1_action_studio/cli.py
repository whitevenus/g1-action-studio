"""命令入口；文件读取与离线规划不依赖设备 SDK。"""

import argparse
import math
import signal
import sys
from threading import Event

from g1_action_studio.motion import load_action, load_start
from g1_action_studio.preview import run_preview


def positive_number(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是正的有限数")
    return number


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="g1-action-studio", description="G1 上肢动作离线预览")
    commands = parser.add_subparsers(dest="command", required=True)
    preview = commands.add_parser("preview", help="预览四种范围的动作及直连衔接、返回")
    preview.add_argument("actions", nargs="+", help="group + frames 格式的动作 JSON")
    preview.add_argument("--start", required=True, help="leg_q[12] + upper_q[17] 启动姿态 JSON")
    preview.add_argument("--speed", type=positive_number, default=1.0, help="动作速度倍率，默认1")
    preview.add_argument("--transition-seconds", type=positive_number, default=2.0, help="每个连接段时长，默认2秒")
    args = parser.parse_args(argv)
    cancel = Event()
    previous_handler = signal.signal(signal.SIGINT, lambda *_: cancel.set())
    try:
        actions = [load_action(path) for path in args.actions]
        start = load_start(args.start)
        run_preview(actions, start, args.speed, args.transition_seconds, cancel)
        return 130 if cancel.is_set() else 0
    except InterruptedError:
        return 130
    except (OSError, ValueError, RuntimeError, ImportError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous_handler)
