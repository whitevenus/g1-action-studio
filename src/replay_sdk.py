"""依赖注入的 SDK 适配器；调用方必须显式提供 SDK，不导入真机库。"""

import math
from threading import Lock

from .constants import (
    ARM_COMMAND_TOPIC,
    ARM_SLOTS,
    DEX3_LAYOUT,
    DT,
    EXPECTED_MODE,
    EXPECTED_MOTOR_MODES,
    HAND_COMMAND_TOPICS,
    HAND_MOTOR_IDS,
    HAND_STATE_TOPICS,
    LOWSTATE_TOPIC,
    STATE_TIMEOUT,
    WEIGHT_SLOT,
)
from .trajectory import finite_vector


def validate_snapshot(snapshot, now, timeout=STATE_TIMEOUT, *, layout=DEX3_LAYOUT) -> dict:
    """示教与回放共用反馈检查；时钟由调用方传入。"""
    if not isinstance(snapshot, dict):
        raise RuntimeError("手臂或手指反馈状态缺失")
    at = snapshot.get("at")
    if type(at) not in (int, float) or not math.isfinite(at) or not 0 <= now - at <= timeout:
        raise RuntimeError("手臂或手指状态超时或时间戳无效")
    if (snapshot.get("mode_pr"), snapshot.get("mode_machine")) != EXPECTED_MODE:
        raise RuntimeError("机器人模式已改变")
    if tuple(snapshot.get("motor_modes", ())) != EXPECTED_MOTOR_MODES:
        raise RuntimeError("手臂电机模式已改变")
    for key in ("q", "dq"):
        values = snapshot.get(key)
        if (
            not isinstance(values, (tuple, list))
            or len(values) != layout.joint_count
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)
        ):
            raise RuntimeError("手臂或手指关节状态无效")
    return snapshot


class ReplaySdkPort:
    def __init__(self, sdk, clock, *, layout=DEX3_LAYOUT):
        self.layout = layout
        self.sdk, self.clock = sdk, clock
        self.publisher = self.subscriber = self.arm_subscriber = None
        self.lock = Lock()
        # 手臂关节+body 实时状态
        self.latest = None
        self.commands = 0
        # 灵巧手实时状态
        self.hand_states: dict[str, dict | None] = {side: None for side in HAND_MOTOR_IDS}
        self.hand_resources = []
        self.hand_publishers = {}
        self.hand_profile: dict | None = None
        try:
            self.subscriber = sdk.ChannelSubscriber(LOWSTATE_TOPIC, sdk.LowStateType)
            self.subscriber.Init(self._on_state, 10)
            self.arm_subscriber = sdk.ChannelSubscriber(ARM_COMMAND_TOPIC, sdk.LowCmdType)
            self.arm_subscriber.Init(self._on_command, 10)
            if layout.hand_model and hasattr(sdk, "initialize_hand_types"):
                sdk.initialize_hand_types()
            for side in HAND_MOTOR_IDS if layout.hand_model else ():
                subscriber = sdk.ChannelSubscriber(HAND_STATE_TOPICS[side], sdk.HandStateType)
                self.hand_resources.append(subscriber)
                subscriber.Init(lambda message, side=side: self._on_hand(side, message), 10)
                observer = sdk.ChannelSubscriber(HAND_COMMAND_TOPICS[side], sdk.HandCmdType)
                self.hand_resources.append(observer)
                observer.Init(self._on_command, 10)
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup_error:
                raise RuntimeError("初始化失败：{}；关闭失败：{}".format(error, cleanup_error)) from error
            raise

    def _on_state(self, message):
        try:
            motors = [message.motor_state[i] for i in ARM_SLOTS]
            state = {
                "at": self.clock.monotonic(),
                "mode_pr": int(message.mode_pr),
                "mode_machine": int(message.mode_machine),
                "motor_modes": tuple(int(m.mode) for m in motors),
                "q": tuple(float(m.q) for m in motors),
                "dq": tuple(float(m.dq) for m in motors),
            }
            try:
                state["body_q"] = tuple(float(message.motor_state[i].q) for i in range(ARM_SLOTS[0]))
            except (AttributeError, IndexError, TypeError, ValueError):
                state["body_q"] = None
        except (AttributeError, TypeError, ValueError, IndexError):
            # 不沿用旧反馈掩盖损坏消息。
            state = None
        with self.lock:
            self.latest = state

    def _on_hand(self, side, message):
        try:
            if len(message.motor_state) != 7:
                raise ValueError("Dex3-1 状态必须有7个电机")
            motors = [message.motor_state[i] for i in HAND_MOTOR_IDS[side]]
            q, dq = tuple(float(m.q) for m in motors), tuple(float(m.dq) for m in motors)
            if not all(math.isfinite(v) for v in q + dq):
                raise ValueError("手指反馈包含非有限数")
            state = {"at": self.clock.monotonic(), "q": q, "dq": dq}
        except (AttributeError, TypeError, ValueError, IndexError):
            state = None
        with self.lock:
            self.hand_states[side] = state

    def _on_command(self, _message):
        with self.lock:
            self.commands += 1

    def snapshot(self):
        with self.lock:
            if self.layout.hand_model is None:
                return None if self.latest is None else dict(self.latest)
            left, right = self.hand_states["left"], self.hand_states["right"]
            if self.latest is None or left is None or right is None:
                return None
            # 使用最旧的接收时间，手部断流不能被新鲜的手臂反馈掩盖。
            return {
                **self.latest,
                "at": min(self.latest["at"], left["at"], right["at"]),
                "q": self.latest["q"] + left["q"] + right["q"],
                "dq": self.latest["dq"] + left["dq"] + right["dq"],
            }

    def observed_commands(self):
        with self.lock:
            return self.commands

    def wait_ready(self, timeout, stop):
        """订阅回调异步到达，初始状态未收到前不能创建发布者。"""
        deadline = self.clock.monotonic() + timeout
        while self.snapshot() is None:
            if stop.is_set() or self.clock.monotonic() >= deadline:
                return False
            self.clock.sleep(DT)
        return True

    def check_services(self):
        control, arm = self.sdk.read_services()
        if control != {"motion_service": "ai", "fsm_id": 501} or not isinstance(arm, dict) or arm.get("status") != 0:
            raise RuntimeError("运控或手臂服务不符合候选条件")

    def start_publisher(self):
        if self.layout.hand_model and self.hand_profile is None:
            raise RuntimeError("缺少手部增益参数")
        self.publisher = self.sdk.ChannelPublisher(ARM_COMMAND_TOPIC, self.sdk.LowCmdType)
        self.publisher.Init()
        for side in HAND_MOTOR_IDS if self.layout.hand_model else ():
            publisher = self.sdk.ChannelPublisher(HAND_COMMAND_TOPICS[side], self.sdk.HandCmdType)
            self.hand_resources.append(publisher)
            publisher.Init()
            self.hand_publishers[side] = publisher

    def write(
        self, positions, kp, kd, weight, *, side=None, selected_kp=None, selected_kd=None, hand_kp=None, hand_kd=None
    ):
        positions = finite_vector(
            list(positions), size=self.layout.joint_count
        )  # 在任何通道发送前核对当前布局的完整目标。
        state = self.snapshot()
        publisher, profile = self.publisher, self.hand_profile
        if (
            state is None
            or publisher is None
            or (self.layout.hand_model and (profile is None or len(self.hand_publishers) != 2))
        ):
            raise RuntimeError("手臂/手指反馈或发布器尚未就绪")
        command = self.sdk.LowCmd()
        command.mode_pr = 0
        command.mode_machine = state["mode_machine"]
        for offset, slot in enumerate(ARM_SLOTS):
            motor = command.motor_cmd[slot]
            motor.mode = 1
            motor.q, motor.dq, motor.tau = float(positions[offset]), 0.0, 0.0
            selected = side == ("left" if offset < 7 else "right")
            motor.kp = float(selected_kp if selected else kp)
            motor.kd = float(selected_kd if selected else kd)
        command.motor_cmd[WEIGHT_SLOT].q = float(weight)
        command.crc = self.sdk.CRC().Crc(command)
        if publisher.Write(command) is not True:
            return False
        for hand_side, ids in HAND_MOTOR_IDS.items() if self.layout.hand_model else ():
            command = self.sdk.HandCmd()
            offset = 14 if hand_side == "left" else 21
            selected = hand_side == side and hand_kp is not None
            for index, motor_id in enumerate(ids):
                motor = command.motor_cmd[motor_id]
                # 官方Dex3-1模式编码：低4位电机ID，bit4..6为status=1。
                motor.mode = motor_id | (1 << 4)
                motor.q, motor.dq, motor.tau = float(positions[offset + index]), 0.0, 0.0
                motor.kp = float(hand_kp if selected else profile["hand_hold_kp"])
                motor.kd = float(hand_kd if selected else profile["hand_hold_kd"])
            if self.hand_publishers[hand_side].Write(command) is not True:
                raise RuntimeError("Dex3-1 {} 命令发送失败；部分命令可能已送达".format(hand_side))
        return True

    def close(self) -> None:
        """关闭本端口拥有的全部通道；单个失败不能阻止其余通道清理。"""
        errors = []
        for name in ("publisher", "arm_subscriber", "subscriber"):
            resource = getattr(self, name, None)
            if resource is not None:
                try:
                    resource.Close()
                except Exception as error:
                    errors.append("{}：{}".format(name, error))
                finally:
                    setattr(self, name, None)
        for resource in getattr(self, "hand_resources", []):
            try:
                resource.Close()
            except Exception as error:
                errors.append("手部通道：{}".format(error))
        self.hand_resources = []
        if errors:
            raise RuntimeError("；".join(errors))
