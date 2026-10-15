"""只订阅原始 Dex3-1 状态，通过手动活动单指核实 DDS 电机下标；不发送控制命令。"""

import argparse
import json
import math
import sys
import time
from pathlib import Path
from threading import Lock


class HandObservation:
    def __init__(self):
        self.lock = Lock()
        self.latest = None
        self.received_at = None
        self.invalid = 0
        self.error = None
        self.samples = 0
        self.minimum = None
        self.maximum = None

    def on_state(self, message):
        try:
            if len(message.motor_state) != 7:
                raise ValueError("motor_state 长度为 {}，预期 7".format(len(message.motor_state)))
            q = tuple(float(m.q) for m in message.motor_state)
            if not all(math.isfinite(value) for value in q):
                raise ValueError("q 包含 NaN 或无穷值")
        except (AttributeError, TypeError, ValueError, IndexError) as error:
            with self.lock:
                self.invalid += 1
                self.error = str(error)
            return
        with self.lock:
            self.latest = q
            self.received_at = time.monotonic()
            if self.minimum is not None:
                self.samples += 1
                self.minimum = [min(a, b) for a, b in zip(self.minimum, q)]
                self.maximum = [max(a, b) for a, b in zip(self.maximum, q)]

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                raise RuntimeError("尚未收到有效状态；最近解析错误：{}".format(self.error))
            age = time.monotonic() - self.received_at
            if age > 0.5:
                raise RuntimeError("状态已断流 {:.2f} 秒，不能继续验证".format(age))
            if self.error is not None:
                raise RuntimeError("收到无效状态：{}".format(self.error))
            return self.latest

    def begin(self):
        self.snapshot()
        with self.lock:
            self.samples = 0
            self.minimum = list(self.latest)
            self.maximum = list(self.latest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", required=True, help="接收 DDS 的网卡名，例如 eth0；用 ip -br addr 查看")
    parser.add_argument("--domain", type=int, default=0, help="DDS domain，默认 0")
    parser.add_argument("--side", required=True, choices=("left", "right"))
    parser.add_argument("--finger", required=True, choices=("thumb", "middle", "index"), help="本次手动活动的手指标签")
    parser.add_argument("--seconds", type=float, default=15, help="记录时长，默认 15 秒")
    parser.add_argument("--output", type=Path, help="可选 JSON 报告路径，不覆盖已有文件")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 300:
        parser.error("--seconds 必须在 1 到 300 之间")
    if args.output is not None and args.output.exists():
        parser.error("输出文件已存在，请换一个文件名")

    # 延迟导入，使 --help 可以在没有机器人 SDK 的电脑上运行。
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

    topic = "rt/dex3/{}/state".format(args.side)
    observation = HandObservation()
    subscriber = None
    try:
        ChannelFactoryInitialize(args.domain, args.interface)
        subscriber = ChannelSubscriber(topic, HandState_)
        print("只读订阅 {}；不发送指令、不切换服务。".format(topic), flush=True)
        subscriber.Init(observation.on_state, 1)
        deadline = time.monotonic() + 5
        while True:
            with observation.lock:
                received = observation.latest is not None
            if received:
                break
            if time.monotonic() >= deadline:
                raise RuntimeError("5 秒内未收到有效状态，请检查网卡、domain、SDK 和手部状态服务")
            time.sleep(0.05)

        print("保持手指静止，3 秒后开始。若手指有保持力，请勿强掰；脚本不会使其卸力。", flush=True)
        for remaining in (3, 2, 1):
            observation.snapshot()
            print("{}...".format(remaining), flush=True)
            time.sleep(1)
        observation.begin()
        print("开始：仅轻轻活动 {} 手的 {}，其余手指尽量静止。".format(args.side, args.finger), flush=True)
        print("下列数值按原始 DDS 下标 0..6 排列，单位 rad。", flush=True)
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            q = observation.snapshot()
            print("q: " + "  ".join("{}:{:+.4f}".format(i, value) for i, value in enumerate(q)), flush=True)
            time.sleep(0.5)
        observation.snapshot()
    finally:
        if subscriber is not None:
            original_error = sys.exc_info()[0] is not None
            try:
                subscriber.Close()
            except Exception as error:
                if not original_error:
                    raise
                print("关闭订阅失败：{}".format(error), file=sys.stderr)

    with observation.lock:
        if observation.samples == 0:
            raise RuntimeError("记录期间没有新的有效样本，不能生成验证报告")
        ranges = [high - low for low, high in zip(observation.minimum, observation.maximum)]
        report = {
            "topic": topic,
            "interface": args.interface,
            "domain": args.domain,
            "finger_label_by_operator": args.finger,
            "requested_seconds": args.seconds,
            "samples": observation.samples,
            "invalid_messages": observation.invalid,
            "minimum_q_rad": observation.minimum,
            "maximum_q_rad": observation.maximum,
            "range_q_rad": ranges,
        }
    print("\n原始下标     最小 q      最大 q      变化范围(rad)")
    for i in range(7):
        print(
            "{:8d}  {:+10.5f}  {:+10.5f}  {:12.5f}".format(
                i, observation.minimum[i], observation.maximum[i], ranges[i]
            )
        )
    ranking = sorted(range(7), key=lambda i: ranges[i], reverse=True)
    print("变化由大到小的下标：{}".format(ranking))
    print("请比较 middle/index 两次报告；无明显变化或多个手指联动时，不能确认映射。")
    if args.output is not None:
        with args.output.open("x", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        print("报告已保存：{}".format(args.output))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已中止并关闭订阅；本次未生成完整报告。", file=sys.stderr)
        sys.exit(130)
    except Exception as error:
        print("验证失败：{}".format(error), file=sys.stderr)
        sys.exit(1)
