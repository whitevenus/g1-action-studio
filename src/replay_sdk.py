"""依赖注入的 SDK 适配器；调用方必须显式提供 SDK，不导入真机库。"""

from threading import Lock

from . import ARM_SLOTS, WEIGHT_SLOT
from .author import DT


class ReplaySdkPort:
    def __init__(self, sdk, clock):
        self.sdk, self.clock = sdk, clock
        self.publisher = self.subscriber = self.arm_subscriber = None
        self.lock = Lock()
        self.latest = None
        self.commands = 0
        try:
            self.subscriber = sdk.ChannelSubscriber("rt/lowstate", getattr(sdk, "LowStateType", sdk.LowState))
            self.subscriber.Init(self._on_state, 10)
            self.arm_subscriber = sdk.ChannelSubscriber("rt/arm_sdk", getattr(sdk, "LowCmdType", sdk.LowCmd))
            self.arm_subscriber.Init(self._on_command, 10)
        except BaseException as error:
            try:
                self.close()
            except Exception as cleanup_error:
                raise RuntimeError("初始化失败：{}；关闭失败：{}".format(error, cleanup_error)) from error
            raise

    def _on_state(self, message):
        try:
            motors = [message.motor_state[i] for i in ARM_SLOTS]
            state = {"at": self.clock.monotonic(), "mode_pr": int(message.mode_pr),
                     "mode_machine": int(message.mode_machine),
                     "motor_modes": tuple(int(m.mode) for m in motors),
                     "q": tuple(float(m.q) for m in motors),
                     "dq": tuple(float(m.dq) for m in motors)}
        except (AttributeError, TypeError, ValueError, IndexError):
            # 不沿用旧反馈掩盖损坏消息。
            state = None
        with self.lock:
            self.latest = state

    def _on_command(self, _message):
        with self.lock:
            self.commands += 1

    def snapshot(self):
        with self.lock:
            return self.latest

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
        if (control != {"motion_service": "ai", "fsm_id": 501}
                or not isinstance(arm, dict) or arm.get("status") != 0):
            raise RuntimeError("运控或手臂服务不符合候选条件")

    def start_publisher(self):
        self.publisher = self.sdk.ChannelPublisher("rt/arm_sdk", getattr(self.sdk, "LowCmdType", self.sdk.LowCmd))
        self.publisher.Init()

    def write(self, positions, kp, kd, weight):
        state = self.snapshot()
        command = self.sdk.LowCmd()
        command.mode_pr = 0
        command.mode_machine = state["mode_machine"]
        for offset, slot in enumerate(ARM_SLOTS):
            motor = command.motor_cmd[slot]
            motor.mode = 1
            motor.q, motor.dq, motor.tau = float(positions[offset]), 0.0, 0.0
            motor.kp, motor.kd = float(kp), float(kd)
        command.motor_cmd[WEIGHT_SLOT].q = float(weight)
        command.crc = self.sdk.CRC().Crc(command)
        return self.publisher.Write(command)

    def close(self):
        errors = []
        for name in ("publisher", "arm_subscriber", "subscriber"):
            resource = getattr(self, name)
            if resource is not None:
                try:
                    resource.Close()
                except Exception as error:
                    errors.append("{}：{}".format(name, error))
                finally:
                    setattr(self, name, None)
        if errors:
            raise RuntimeError("；".join(errors))
