"""一个反馈会话、一个发送线程，以及唯一的官方 HG 报文构造。"""

import time
from threading import Condition, Event, Lock, Thread

import numpy as np

from .model import UPPER_SLOTS
from .motion import StartPose

DT = 0.02
RAMP_SECONDS = 2.0

_factory_lock = Lock()
_factory_initialized = False


def build_command(q, dq, kp, kd, weight, crc):
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_

    command = unitree_hg_msg_dds__LowCmd_()
    for index, slot in enumerate(UPPER_SLOTS):
        motor = command.motor_cmd[slot]
        motor.q = float(q[index])
        motor.dq = float(dq[index])
        motor.kp = float(kp[index])
        motor.kd = float(kd[index])
        motor.tau = 0.0
    command.motor_cmd[29].q = float(weight)
    command.crc = crc.Crc(command)
    return command


def _causes(error):
    if isinstance(error, ExceptionGroup):
        return [cause for nested in error.exceptions for cause in _causes(nested)]
    return [error]


def _combined(errors):
    unique = []
    for error in errors:
        for cause in _causes(error):
            if not any(cause is previous for previous in unique):
                unique.append(cause)
    if len(unique) == 1:
        return unique[0]
    return ExceptionGroup("会话执行或资源清理失败", unique)


class RobotSession:
    def __init__(self, interface, gains, cancel):
        self.interface = interface
        self.cancel = cancel
        self.start_pose = None
        self._kp = np.array([gains["arms"]["hold_kp"]] * 14 + [gains["waist"]["hold_kp"]] * 3)
        self._kd = np.array([gains["arms"]["hold_kd"]] * 14 + [gains["waist"]["hold_kd"]] * 3)
        self._condition = Condition()
        self._close_lock = Lock()
        self._closing = Event()
        self._closed = False
        self._subscriber = None
        self._publisher = None
        self._sender = None
        self._feedback = None
        self._errors = []
        self._phase = "IDLE"
        self._phase_started = None
        self._target_q = None
        self._last_q = None
        self._last_weight = 0.0
        self._trajectory = None
        self._play_id = 0
        self._completed_play_id = 0

    def __enter__(self):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        global _factory_initialized
        try:
            with self._condition:
                self._check()
            with _factory_lock:
                if not _factory_initialized:
                    ChannelFactoryInitialize(0, self.interface)
                    _factory_initialized = True
            self._subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self._subscriber.Init(self._on_feedback, 0)
            with self._condition:
                self._wait_for(lambda: self.start_pose is not None)
            return self
        except Exception as error:
            self.__exit__(type(error), error, error.__traceback__)
            raise

    def __exit__(self, error_type, error, traceback):
        try:
            self.close()
        except Exception as cleanup_error:
            if error is not None:
                raise _combined([error, cleanup_error]) from None
            raise
        if error is None and self.cancel.is_set():
            raise InterruptedError("操作已取消")
        return False

    def _on_feedback(self, message):
        received = time.monotonic_ns()
        try:
            pose = StartPose(
                [message.motor_state[slot].q for slot in range(12)],
                [message.motor_state[slot].q for slot in UPPER_SLOTS],
            )
            with self._condition:
                self._feedback = received, pose
                if self.start_pose is None:
                    self.start_pose = pose
                self._condition.notify_all()
        except Exception as error:
            self._failed(error)

    def _failed(self, error):
        with self._condition:
            self._errors.append(error)
            self.cancel.set()
            self._condition.notify_all()

    def _check(self):
        if self._errors:
            raise _combined(self._errors)
        if self.cancel.is_set():
            raise InterruptedError("操作已取消")
        if self._closing.is_set() or self._closed:
            raise RuntimeError("会话已关闭")

    def _wait_for(self, complete):
        while not complete():
            self._check()
            self._condition.wait(DT)
        self._check()

    def acquire(self):
        from unitree_sdk2py.core.channel import ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        with self._condition:
            self._check()
            if self._phase != "IDLE" or self.start_pose is None:
                raise RuntimeError("会话尚未取得启动反馈，或已经接管")
        self._crc = CRC()
        self._publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self._publisher.Init()
        with self._condition:
            self._check()
            self._target_q = self.start_pose.upper_q.copy()
            self._phase = "ACQUIRE"
            self._phase_started = None
        sender = Thread(target=self._send_loop, name="g1-action-sender", daemon=False)
        sender.start()
        self._sender = sender
        with self._condition:
            self._wait_for(lambda: self._phase == "HOLD")

    def play(self, trajectory):
        with self._condition:
            self._check()
            if self._phase != "HOLD":
                raise RuntimeError("play 需要已经接管的会话")
            self._trajectory = trajectory
            self._play_id += 1
            requested = self._play_id
            self._phase = "PLAY"
            self._phase_started = None
            self._wait_for(lambda: self._completed_play_id >= requested)

    def _send_loop(self):
        zeros = np.zeros(17)
        try:
            while True:
                now = time.monotonic()
                with self._condition:
                    if (self.cancel.is_set() or self._closing.is_set()) and self._phase != "RELEASE":
                        if self._last_q is None:
                            break
                        self._phase = "RELEASE"
                        self._phase_started = now
                        release_q = self._last_q.copy()
                        release_weight = self._last_weight
                    if self._phase_started is None:
                        self._phase_started = now
                    phase = self._phase
                    elapsed = now - self._phase_started
                    trajectory = self._trajectory
                    play_id = self._play_id
                    q = self._target_q.copy()
                dq = zeros
                finished = False
                if phase == "ACQUIRE":
                    weight = min(1.0, elapsed / RAMP_SECONDS)
                    finished = weight == 1.0
                elif phase == "PLAY":
                    q, dq = trajectory.sample(elapsed)
                    weight = 1.0
                    finished = elapsed >= trajectory.duration
                elif phase == "RELEASE":
                    q = release_q
                    weight = release_weight * (1.0 - min(1.0, elapsed / RAMP_SECONDS))
                    finished = elapsed >= RAMP_SECONDS
                else:
                    weight = 1.0
                command = build_command(q, dq, self._kp, self._kd, weight, self._crc)
                if self._publisher.Write(command) is False:
                    raise RuntimeError("rt/arm_sdk Write 返回 False")
                with self._condition:
                    self._last_q = q.copy()
                    self._last_weight = weight
                    if phase == "RELEASE" and finished:
                        break
                    if finished and self._phase == phase:
                        self._target_q = q.copy()
                        self._phase = "HOLD"
                        if phase == "PLAY":
                            self._completed_play_id = play_id
                            self._trajectory = None
                        self._condition.notify_all()
                time.sleep(max(0.0, DT - (time.monotonic() - now)))
        except Exception as error:
            self._failed(error)
        finally:
            with self._condition:
                self._condition.notify_all()

    def close(self):
        with self._close_lock:
            if self._closed:
                return
            self._closing.set()
            if self._sender is not None:
                self._sender.join()
            for channel in (self._publisher, self._subscriber):
                if channel is not None:
                    try:
                        channel.Close()
                    except Exception as error:
                        with self._condition:
                            self._errors.append(error)
            with self._condition:
                self._closed = True
                self._phase = "CLOSED"
                self._condition.notify_all()
                errors = self._errors.copy()
            if errors:
                raise _combined(errors)
