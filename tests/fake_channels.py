"""只替换 SDK 通道；消息、CRC 与应用发送线程仍执行真实实现。"""

import copy
import time
from contextlib import ExitStack
from dataclasses import dataclass
from threading import Condition, Lock, current_thread
from unittest.mock import patch

import numpy as np

from g1_action_studio.model import UPPER_SLOTS

_REAL_MONOTONIC = time.monotonic
HOME = np.array([0, 0.2, 0, 0.8, 0, 0, 0] + [0, -0.2, 0, 0.8, 0, 0, 0] + [0, 0, 0], dtype=np.float64)


class Clock:
    def __init__(self, scale=40):
        self.scale = scale
        self._started = _REAL_MONOTONIC()
        self._offset = 0.0
        self._lock = Lock()

    def monotonic(self):
        with self._lock:
            return self._started + (_REAL_MONOTONIC() - self._started) * self.scale + self._offset

    def monotonic_ns(self):
        return int(self.monotonic() * 1_000_000_000)

    def advance(self, seconds):
        with self._lock:
            self._offset += seconds


@dataclass
class Write:
    command: object
    thread: object
    result: bool | None = None
    error: Exception | None = None


class Channels:
    def __init__(self, upper=HOME, legs=None, initial=True):
        self.upper = np.array(upper, copy=True)
        self.legs = np.zeros(12) if legs is None else np.array(legs, copy=True)
        self.initial = initial
        self.errors = {}
        self.calls = []
        self.records = []
        self.close_while_sender_alive = []
        self.on_write = None
        self.callback = None
        self._changed = Condition()
        self._patches = ExitStack()

    @property
    def commands(self):
        with self._changed:
            return [record.command for record in self.records]

    def _call(self, name, *arguments):
        with self._changed:
            if name.endswith("_close") and any(record.thread.is_alive() for record in self.records):
                self.close_while_sender_alive.append(name)
            self.calls.append((name, *arguments))
            self._changed.notify_all()
        if name in self.errors:
            raise self.errors[name]

    def emit(self, upper=None, legs=None, message=None):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_

        if message is None:
            message = unitree_hg_msg_dds__LowState_()
            for slot, q in zip(UPPER_SLOTS, self.upper if upper is None else upper):
                message.motor_state[slot].q = float(q)
            for slot, q in enumerate(self.legs if legs is None else legs):
                message.motor_state[slot].q = float(q)
        self.callback(message)

    def wait_for(self, predicate, timeout=2):
        with self._changed:
            return self._changed.wait_for(predicate, timeout)

    def __enter__(self):
        from unitree_sdk2py.core import channel

        owner = self

        def initialize(domain, interface):
            owner._call("factory", domain, interface)

        class Subscriber:
            def __init__(self, topic, message_type):
                owner._call("subscriber", topic, message_type)

            def Init(self, callback, queue_length):
                owner.callback = callback
                owner._call("subscriber_init", queue_length)
                if owner.initial:
                    owner.emit()

            def Close(self):
                owner._call("subscriber_close")

        class Publisher:
            def __init__(self, topic, message_type):
                owner._call("publisher", topic, message_type)

            def Init(self):
                owner._call("publisher_init")

            def Write(self, command):
                record = Write(copy.deepcopy(command), current_thread())
                with owner._changed:
                    owner.records.append(record)
                    owner._changed.notify_all()
                try:
                    result = True if owner.on_write is None else owner.on_write(record)
                    record.result = result
                    return result
                except Exception as error:
                    record.error = error
                    raise
                finally:
                    with owner._changed:
                        owner._changed.notify_all()

            def Close(self):
                owner._call("publisher_close")

        for name, replacement in (
            ("ChannelFactoryInitialize", initialize),
            ("ChannelSubscriber", Subscriber),
            ("ChannelPublisher", Publisher),
        ):
            self._patches.enter_context(patch.object(channel, name, replacement))
        for name in ("DomainParticipant", "DataReader", "DataWriter"):
            if hasattr(channel, name):
                self._patches.enter_context(
                    patch.object(channel, name, side_effect=AssertionError("离线测试禁止构造 DDS 网络实体"))
                )
        return self

    def __exit__(self, *_):
        self._patches.close()
