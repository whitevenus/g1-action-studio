"""一个反馈会话、一个发送线程，以及唯一的官方 HG 报文构造。"""

import time
from threading import Condition, Event, Lock, Thread

import numpy as np

from .model import GROUPS, UPPER_SLOTS
from .motion import Action, StartPose

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
        self._kp = np.array([gains["arms"]["hold_kp"]] * 14 + [gains["waist"]["hold_kp"]] * 3, dtype=float)
        self._kd = np.array([gains["arms"]["hold_kd"]] * 14 + [gains["waist"]["hold_kd"]] * 3, dtype=float)
        self._teach_kp = np.array([gains["arms"]["teach_kp"]] * 14 + [gains["waist"]["teach_kp"]] * 3, dtype=float)
        self._teach_kd = np.array([gains["arms"]["teach_kd"]] * 14 + [gains["waist"]["teach_kd"]] * 3, dtype=float)
        self._last_kp = self._kp.copy()
        self._last_kd = self._kd.copy()
        self._gain_from_kp = self._kp.copy()
        self._gain_from_kd = self._kd.copy()
        self._gain_to_kp = self._kp.copy()
        self._gain_to_kd = self._kd.copy()
        self._gain_started = None
        self._gain_id = 0
        self._completed_gain_id = 0
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
        self._group = None
        self._selected = []
        self._recording = False
        self._record_epoch = 0
        self._record_started_ns = 0
        self._samples = []

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
        with self._condition:
            record_epoch = self._record_epoch
            recording = self._recording
        try:
            pose = StartPose(
                [message.motor_state[slot].q for slot in range(12)],
                [message.motor_state[slot].q for slot in UPPER_SLOTS],
            )
            with self._condition:
                self._feedback = received, pose
                if self.start_pose is None:
                    self.start_pose = pose
                if (
                    recording
                    and self._recording
                    and record_epoch == self._record_epoch
                    and received > self._record_started_ns
                ):
                    if self._samples and received <= self._samples[-1][0]:
                        raise ValueError("示教反馈接收时间必须严格递增")
                    self._samples.append((received, pose.upper_q[self._selected].copy()))
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

    def _change_gains(self, kp, kd):
        self._gain_from_kp = self._last_kp.copy()
        self._gain_from_kd = self._last_kd.copy()
        self._gain_to_kp = kp.copy()
        self._gain_to_kd = kd.copy()
        self._gain_started = None
        self._gain_id += 1
        return self._gain_id

    def follow(self, group):
        with self._condition:
            self._check()
            if self._phase != "HOLD":
                raise RuntimeError("follow 需要已经接管的固定目标")
            self._group = group
            self._selected = list(GROUPS[group])
            kp, kd = self._kp.copy(), self._kd.copy()
            kp[self._selected] = self._teach_kp[self._selected]
            kd[self._selected] = self._teach_kd[self._selected]
            requested = self._change_gains(kp, kd)
            self._phase = "FOLLOW"
            self._wait_for(lambda: self._completed_gain_id >= requested)

    def begin_recording(self):
        with self._condition:
            self._check()
            if self._phase != "FOLLOW" or self._recording:
                raise RuntimeError("录制需要已经跟随且尚未开始记录")
            self._samples = []
            self._record_epoch += 1
            self._record_started_ns = time.monotonic_ns()
            self._recording = True

    def end_recording(self):
        with self._condition:
            self._check()
            if self._phase != "FOLLOW" or not self._recording:
                raise RuntimeError("尚未开始录制")
            self._recording = False
            self._record_epoch += 1
            samples = self._samples.copy()
            selected_q = samples[-1][1] if samples else self._feedback[1].upper_q[self._selected]
            frozen = self.start_pose.upper_q.copy()
            frozen[self._selected] = selected_q
            self._target_q = frozen
            self._phase = "HOLD"
            self._change_gains(self._last_kp, self._last_kd)
            self._completed_gain_id = self._gain_id
            group = self._group
            self._condition.notify_all()
        if len(samples) < 2:
            raise ValueError("示教至少需要两个真实反馈样本")
        first = samples[0][0]
        action = Action(group, [(received - first) / 1_000_000_000 for received, _ in samples], [q for _, q in samples])
        return action, frozen.copy()

    def hold(self):
        with self._condition:
            self._check()
            if self._phase != "HOLD":
                raise RuntimeError("hold 需要已经冻结的目标")
            requested = self._change_gains(self._kp, self._kd)
            self._wait_for(lambda: self._completed_gain_id >= requested)

    def play(self, trajectory):
        with self._condition:
            self._check()
            if self._phase != "HOLD":
                raise RuntimeError("play 需要已经接管的会话")
            self._change_gains(self._kp, self._kd)
            self._completed_gain_id = self._gain_id
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
                        release_kp = self._last_kp.copy()
                        release_kd = self._last_kd.copy()
                    if self._phase_started is None:
                        self._phase_started = now
                    phase = self._phase
                    elapsed = now - self._phase_started
                    trajectory = self._trajectory
                    play_id = self._play_id
                    q = self._target_q.copy()
                    if phase == "FOLLOW":
                        q = self.start_pose.upper_q.copy()
                        q[self._selected] = self._feedback[1].upper_q[self._selected]
                    gain_id = self._gain_id
                    gain_finished = False
                    kp, kd = self._gain_to_kp.copy(), self._gain_to_kd.copy()
                    if gain_id > self._completed_gain_id:
                        if self._gain_started is None:
                            self._gain_started = now
                        fraction = min(1.0, (now - self._gain_started) / RAMP_SECONDS)
                        if fraction < 1.0:
                            kp = self._gain_from_kp + fraction * (kp - self._gain_from_kp)
                            kd = self._gain_from_kd + fraction * (kd - self._gain_from_kd)
                        gain_finished = fraction == 1.0
                dq = zeros
                finished = False
                if phase == "ACQUIRE":
                    kp, kd = self._kp, self._kd
                    weight = min(1.0, elapsed / RAMP_SECONDS)
                    finished = weight == 1.0
                elif phase == "PLAY":
                    kp, kd = self._kp, self._kd
                    q, dq = trajectory.sample(elapsed)
                    weight = 1.0
                    finished = elapsed >= trajectory.duration
                elif phase == "RELEASE":
                    q = release_q
                    fraction = min(1.0, elapsed / RAMP_SECONDS)
                    weight = release_weight * (1.0 - fraction)
                    kp = self._kp if fraction == 1.0 else release_kp + fraction * (self._kp - release_kp)
                    kd = self._kd if fraction == 1.0 else release_kd + fraction * (self._kd - release_kd)
                    finished = elapsed >= RAMP_SECONDS
                else:
                    weight = 1.0
                command = build_command(q, dq, kp, kd, weight, self._crc)
                if self._publisher.Write(command) is False:
                    raise RuntimeError("rt/arm_sdk Write 返回 False")
                with self._condition:
                    self._last_q = q.copy()
                    self._last_weight = weight
                    self._last_kp = kp.copy()
                    self._last_kd = kd.copy()
                    if gain_finished and gain_id == self._gain_id and phase != "RELEASE":
                        self._completed_gain_id = gain_id
                        self._condition.notify_all()
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
