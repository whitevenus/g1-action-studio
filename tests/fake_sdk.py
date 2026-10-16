"""回归测试专用替身：内存消息、虚拟时钟与故障注入；不随项目安装。"""

import copy
import math
from threading import Event
from types import SimpleNamespace

from g1_action_studio.constants import (
    ARM_COMMAND_TOPIC,
    ARM_SLOTS,
    EXPECTED_MODE,
    EXPECTED_MOTOR_MODES,
    HAND_COMMAND_TOPICS,
    HAND_MOTOR_IDS,
    HAND_STATE_TOPICS,
    LOWSTATE_TOPIC,
)
from g1_action_studio.executor import execute_replay
from g1_action_studio.replay_sdk import ReplaySdkPort

SIMULATION_PROFILE = {
    "hold_kp": 12.0,
    "hold_kd": 1.2,
    "max_speed_rad_s": 1.0,
    "hand_hold_kp": 1.5,
    "hand_hold_kd": 0.2,
    "max_offset_rad": 1.5,
    "max_tracking_error_rad": 0.12,
    "initial_pose_tolerance_rad": 0.01,
    "return_tolerance_rad": 0.005,
    "return_speed_rad_s": 0.05,
    "state_timeout_s": 0.06,
    "max_tick_gap_s": 0.1,
    "settle_timeout_s": 2.0,
    "recovery_release_s": 1.0,
}
SCENARIOS = ("normal", "tracking-error", "state-timeout", "mode-change", "send-failure", "cancel", "release-failure")


class VirtualClock:
    def __init__(self):
        self.now = 0.0
        self.listeners = []

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        if not math.isfinite(duration) or duration < 0:
            raise ValueError("模拟等待时长无效")
        self.now += duration
        for listener in self.listeners:
            listener(duration)


class SimulatedSDK:
    """只模拟接口和一阶位置滞后；增益/CRC 不构成物理或协议验证。"""

    def __init__(self, clock, baseline, response_s=0.04):
        if not math.isfinite(response_s) or response_s <= 0:
            raise ValueError("模拟响应时间须为正的有限数")
        self.clock, self.response_s = clock, response_s
        self.q, self.dq = list(baseline), [0.0] * len(baseline)
        self.body_q = [0.0] * ARM_SLOTS[0]
        self.target = list(baseline)
        self.weight = 0.0
        self.subscribers, self.resources, self.writes = [], [], []
        self.publisher_started = False
        self.mode_pr, self.mode_machine = EXPECTED_MODE
        self.motor_modes = list(EXPECTED_MOTOR_MODES)
        self.services_ok, self.drop_states, self.stuck = True, False, False
        self.fail_next_write = self.fail_positive_writes = self.fail_zero_write = False
        self.close_failure = False
        self.before_step = lambda: None
        self.before_write = lambda command: None
        self.external_commands = 0
        self.hand_writes = []
        self.drop_hands = set()
        clock.listeners.append(self._advance)

    @staticmethod
    def LowCmd():
        return SimpleNamespace(
            mode_pr=0,
            mode_machine=0,
            crc=0,
            motor_cmd=[SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(35)],
        )

    LowStateType = SimpleNamespace
    LowCmdType = SimpleNamespace
    HandStateType = SimpleNamespace
    HandCmdType = SimpleNamespace

    @staticmethod
    def HandCmd():
        return SimpleNamespace(
            motor_cmd=[SimpleNamespace(mode=0, q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0) for _ in range(7)]
        )

    @staticmethod
    def CRC():
        return SimpleNamespace(Crc=lambda command: 1234)

    def read_services(self):
        return ({"motion_service": "ai", "fsm_id": 501 if self.services_ok else 801}, {"status": 0})

    def ChannelSubscriber(self, topic, message_type):
        sdk = self

        class Subscriber:
            closed = False

            def Init(self, callback, queue):
                self.topic, self.callback = topic, callback
                sdk.subscribers.append(self)
                if topic == LOWSTATE_TOPIC or topic in HAND_STATE_TOPICS.values():
                    sdk._publish_state()

            def Close(self):
                self.closed = True

        resource = Subscriber()
        self.resources.append(resource)
        return resource

    def ChannelPublisher(self, topic, message_type):
        sdk = self

        class Publisher:
            closed = False

            def Init(self):
                sdk.publisher_started = True

            def Write(self, command):
                if topic in HAND_COMMAND_TOPICS.values():
                    side = next(s for s, t in HAND_COMMAND_TOPICS.items() if t == topic)
                    offset = 14 if side == "left" else 21
                    sdk.target[offset : offset + 7] = [command.motor_cmd[i].q for i in HAND_MOTOR_IDS[side]]
                    sdk.hand_writes.append((side, copy.deepcopy(command)))
                    return True
                sdk.before_write(command)
                sdk.writes.append(copy.deepcopy(command))
                # 即使返回失败，也模拟命令可能已被接收。
                sdk.target[:14] = [command.motor_cmd[i].q for i in ARM_SLOTS]
                sdk.weight = command.motor_cmd[29].q
                if (
                    sdk.fail_next_write
                    or (sdk.fail_positive_writes and sdk.weight > 0)
                    or (sdk.fail_zero_write and sdk.weight == 0)
                ):
                    sdk.fail_next_write = False
                    return False
                return True

            def Close(self):
                self.closed = True
                if sdk.close_failure:
                    raise RuntimeError("模拟发布者关闭失败")

        resource = Publisher()
        self.resources.append(resource)
        return resource

    def _publish_state(self):
        if self.drop_states:
            return
        message = SimpleNamespace(
            mode_pr=self.mode_pr,
            mode_machine=self.mode_machine,
            motor_state=[SimpleNamespace(mode=1, q=0.0, dq=0.0) for _ in range(35)],
        )
        for i, q in enumerate(self.body_q):
            message.motor_state[i].q = q
        for offset, slot in enumerate(ARM_SLOTS):
            message.motor_state[slot] = SimpleNamespace(
                mode=self.motor_modes[offset], q=self.q[offset], dq=self.dq[offset]
            )
        for subscriber in self.subscribers:
            if not subscriber.closed and subscriber.topic == LOWSTATE_TOPIC:
                subscriber.callback(message)
        if len(self.q) == 14:
            return
        for side, ids in HAND_MOTOR_IDS.items():
            if side in self.drop_hands:
                continue
            hand = SimpleNamespace(motor_state=[SimpleNamespace(q=0.0, dq=0.0) for _ in range(7)])
            offset = 14 if side == "left" else 21
            for index, motor_id in enumerate(ids):
                hand.motor_state[motor_id].q = self.q[offset + index]
                hand.motor_state[motor_id].dq = self.dq[offset + index]
            for subscriber in self.subscribers:
                if not subscriber.closed and subscriber.topic == HAND_STATE_TOPICS[side]:
                    subscriber.callback(hand)

    def _advance(self, duration):
        self.before_step()
        if duration > 0:
            old = list(self.q)
            if not self.stuck:
                blend = (1 - math.exp(-duration / self.response_s)) * self.weight
                self.q = [
                    a + (b - a) * (blend if i < 14 else 1 - math.exp(-duration / self.response_s))
                    for i, (a, b) in enumerate(zip(self.q, self.target))
                ]
            self.dq = [(b - a) / duration for a, b in zip(old, self.q)]
        self._publish_state()
        if self.external_commands:
            for subscriber in self.subscribers:
                if not subscriber.closed and subscriber.topic == ARM_COMMAND_TOPIC:
                    subscriber.callback(self.LowCmd())


def simulate_replay(plan, scenario="normal", profile=None):
    if scenario not in SCENARIOS:
        raise ValueError("未知模拟场景")
    clock, stop = VirtualClock(), Event()
    sdk = SimulatedSDK(clock, plan["baseline"]["q"])
    fired = False

    def fault():
        nonlocal fired
        # 在已有正权重写入后注入，保证覆盖接管后的恢复路径。
        if not fired and sdk.publisher_started and clock.now >= 3.3:
            fired = True
            if scenario == "tracking-error":
                sdk.stuck = True
                sdk.q[0] += 0.3
            elif scenario == "state-timeout":
                sdk.drop_states = True
            elif scenario == "mode-change":
                sdk.mode_machine = 4
            elif scenario == "send-failure":
                sdk.fail_next_write = True
            elif scenario == "cancel":
                stop.set()

    def write_fault(command):
        if scenario == "release-failure" and clock.now > 3.3 and command.motor_cmd[29].q == 0:
            sdk.fail_zero_write = True

    sdk.before_step, sdk.before_write = fault, write_fault
    report = execute_replay(
        plan, SIMULATION_PROFILE if profile is None else profile, lambda: ReplaySdkPort(sdk, clock), clock, stop
    )
    report.update(
        qualification="offline_simulated_sdk_only",
        hardware_enabled=False,
        scenario=scenario,
        simulation_model="first_order_joint_lag_not_physics",
        resources_closed=all(resource.closed for resource in sdk.resources),
    )
    return report
