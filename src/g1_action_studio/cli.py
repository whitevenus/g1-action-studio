"""命令入口；文件读取与离线规划不依赖设备 SDK。"""

import argparse
import json
import math
import os
import select
import signal
import sys
from pathlib import Path
from threading import Condition, Event, Thread

from .model import GROUPS
from .motion import _keys, _number, load_action, load_start
from .preview import run_preview
from .replay import run_replay
from .teach import run_teach


def positive_number(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是正的有限数")
    return number


def load_gains(path):
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as source:
            gains = json.load(source)
        _keys(gains, ("arms", "waist"), "增益配置")
        for group, values in gains.items():
            _keys(values, ("hold_kp", "hold_kd", "teach_kp", "teach_kd"), group)
            for name, value in values.items():
                values[name] = _number(value, f"{group}.{name}")
                if values[name] < 0:
                    raise ValueError(f"{group}.{name} 必须非负")
        return gains
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from error


class _TerminalInput:
    def __init__(self, cancel):
        self.cancel = cancel
        self.stop = Event()
        self.error = None
        self._condition = Condition()
        self._prompt = None
        self._awaiting = False
        self._entered = False

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.thread = Thread(target=self._read, name="g1-terminal-input", daemon=False)
        self.thread.start()
        return self

    def _read(self):
        try:
            while not self.stop.is_set():
                with self._condition:
                    if self._prompt is not None:
                        # 只有此线程读取 stdin；提示前排空旧输入，避免读取竞争。
                        while not self.stop.is_set() and select.select([self.fd], [], [], 0)[0]:
                            if not os.read(self.fd, 4096):
                                self.cancel.set()
                                break
                        if self.cancel.is_set() or self.stop.is_set():
                            return
                        print(self._prompt, flush=True)
                        self._prompt = None
                        self._awaiting = True
                        self._condition.notify_all()
                ready, _, _ = select.select([self.fd], [], [], 0.05)
                if ready:
                    data = os.read(self.fd, 4096)
                    with self._condition:
                        if not data:
                            self.cancel.set()
                            self._condition.notify_all()
                            return
                        if self._awaiting and (b"\n" in data or b"\r" in data):
                            self._entered = True
                            self._awaiting = False
                            self._condition.notify_all()
        except Exception as error:
            self.error = error
            self.cancel.set()

    def wait_enter(self, prompt):
        with self._condition:
            self._entered = False
            self._prompt = prompt
            self._condition.notify_all()
            while not self._entered:
                if self.cancel.is_set():
                    raise InterruptedError("操作已取消")
                self._condition.wait(0.05)
            if self.cancel.is_set():
                raise InterruptedError("操作已取消")

    def __exit__(self, kind, error, traceback):
        self.stop.set()
        self.thread.join()
        if self.error is not None:
            errors = [error, self.error] if error is not None else [self.error]
            raise ExceptionGroup("终端输入失败", errors)


def _error_message(error):
    if isinstance(error, ExceptionGroup):
        return "；".join(_error_message(item) for item in error.exceptions)
    return str(error)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="g1-action-studio", description="G1 上肢动作示教、预览与回放")
    commands = parser.add_subparsers(dest="command", required=True)
    preview = commands.add_parser("preview", help="预览四种范围的动作及避碰衔接、返回")
    preview.add_argument("--start", required=True, help="leg_q[12] + upper_q[17] 启动姿态 JSON")
    replay = commands.add_parser("replay", help="一次接管连续回放并返回启动姿态、交回权重")
    teach = commands.add_parser("teach", help="两次回车录制所选关节，保存后避碰返回并交回")
    teach.add_argument("--group", choices=GROUPS, required=True, help="同步示教的关节范围")
    teach.add_argument("--output", required=True, help="新动作 JSON，不覆盖已有文件")
    for command in (replay, teach):
        command.add_argument("--interface", required=True, help="机器人通信网卡")
        command.add_argument("--config", required=True, help="arms 与 waist 两组增益 JSON")
        command.add_argument("--execute", action="store_true", required=True, help="明确执行设备控制")
    for command in (preview, replay):
        command.add_argument("actions", nargs="+", help="group + frames 格式的动作 JSON")
        command.add_argument("--speed", type=positive_number, default=1.0, help="动作速度倍率，默认1")
    for command in (preview, replay, teach):
        command.add_argument("--transition-seconds", type=positive_number, default=2.0, help="每个连接段时长，默认2秒")
    args = parser.parse_args(argv)
    cancel = Event()
    previous_handler = signal.signal(signal.SIGINT, lambda *_: cancel.set())
    try:
        if args.command == "teach":
            gains = load_gains(args.config)
            run_teach(args.interface, args.group, gains, args.output, args.transition_seconds, cancel)
            print("示教流程完成，动作已保存，零权重发送完成。")
        elif args.command == "preview":
            actions = [load_action(path) for path in args.actions]
            start = load_start(args.start)
            run_preview(actions, start, args.speed, args.transition_seconds, cancel)
        else:
            actions = [load_action(path) for path in args.actions]
            gains = load_gains(args.config)
            with _TerminalInput(cancel):
                run_replay(args.interface, actions, gains, args.speed, args.transition_seconds, cancel)
            print("回放流程完成，零权重发送完成。")
        return 130 if cancel.is_set() else 0
    except InterruptedError:
        return 130
    except Exception as error:
        print(f"错误：{_error_message(error)}", file=sys.stderr)
        return 1
    finally:
        signal.signal(signal.SIGINT, previous_handler)
