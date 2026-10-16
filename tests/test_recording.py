import sys
import unittest
from contextlib import ExitStack, contextmanager
from threading import Event, Thread
from unittest.mock import patch

import numpy as np
from fake_channels import HOME, Channels, Clock

from g1_action_studio import robot
from g1_action_studio.model import GROUPS, UPPER_SLOTS

GAINS = {
    "arms": {"hold_kp": 60, "hold_kd": 2, "teach_kp": 15, "teach_kd": 0.4},
    "waist": {"hold_kp": 40, "hold_kd": 1.5, "teach_kp": 10, "teach_kd": 0.2},
}


def field(command, name):
    return np.array([getattr(command.motor_cmd[slot], name) for slot in UPPER_SLOTS])


def gains(group=None):
    kp = np.array([GAINS["arms"]["hold_kp"]] * 14 + [GAINS["waist"]["hold_kp"]] * 3, dtype=float)
    kd = np.array([GAINS["arms"]["hold_kd"]] * 14 + [GAINS["waist"]["hold_kd"]] * 3, dtype=float)
    if group is not None:
        for index in GROUPS[group]:
            values = GAINS["arms" if index < 14 else "waist"]
            kp[index], kd[index] = values["teach_kp"], values["teach_kd"]
    return kp, kd


@unittest.skipUnless(sys.platform == "linux", "真实 Unitree SDK 示教会话测试的交付平台为 Linux")
class RecordingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.utils.crc import CRC

        cls.message = staticmethod(unitree_hg_msg_dds__LowState_)
        CRC()

    @contextmanager
    def environment(self, scale=40):
        clock = Clock(scale=scale)
        with ExitStack() as stack:
            stack.enter_context(patch.object(robot, "_factory_initialized", False))
            stack.enter_context(patch.object(robot.time, "monotonic", clock.monotonic))
            stack.enter_context(patch.object(robot.time, "monotonic_ns", clock.monotonic_ns))
            stack.enter_context(patch.object(robot, "DT", 0.001))
            channels = stack.enter_context(Channels())
            yield channels, clock

    def feedback(self, q):
        message = self.message()
        for slot, value in zip(UPPER_SLOTS, q):
            message.motor_state[slot].q = float(value)
            message.motor_state[slot].dq = 9.0
        return message

    def emit_at(self, channels, q, received):
        with patch.object(robot.time, "monotonic_ns", return_value=received):
            channels.emit(message=self.feedback(q))

    def sent_target(self, channels, expected):
        self.assertTrue(
            channels.wait_for(
                lambda: any(
                    record.result is True and np.array_equal(field(record.command, "q"), expected)
                    for record in channels.records[-3:]
                )
            )
        )

    def test_four_groups_record_real_callbacks_freeze_and_restore_their_own_gains(self):
        for group, selected in GROUPS.items():
            with self.subTest(group=group), self.environment() as (channels, clock):
                session = robot.RobotSession("offline0", GAINS, Event())
                selected = list(selected)
                inactive = sorted(set(range(17)) - set(selected))
                teach_kp, teach_kd = gains(group)
                hold_kp, hold_kd = gains()
                latest = HOME + np.linspace(0.01, 0.17, 17)
                with session:
                    session.acquire()
                    channels.emit(message=self.feedback(latest))
                    before_follow = len(channels.records)
                    session.follow(group)
                    expected = HOME.copy()
                    expected[selected] = latest[selected]
                    self.sent_target(channels, expected)
                    np.testing.assert_array_equal(field(channels.commands[-1], "kp"), teach_kp)
                    np.testing.assert_array_equal(field(channels.commands[-1], "kd"), teach_kd)
                    for command in channels.commands[before_follow:]:
                        np.testing.assert_array_equal(field(command, "q")[inactive], HOME[inactive])
                        np.testing.assert_array_equal(field(command, "kp")[inactive], hold_kp[inactive])
                        np.testing.assert_array_equal(field(command, "kd")[inactive], hold_kd[inactive])
                        np.testing.assert_array_equal(field(command, "dq"), np.zeros(17))
                    epoch = clock.monotonic_ns()
                    with patch.object(robot.time, "monotonic_ns", return_value=epoch):
                        session.begin_recording()
                    first_write = len(channels.records)
                    samples = [latest + offset for offset in (0.03, 0.07, 0.12)]
                    for q, offset_ns in zip(samples, (100_000_000, 350_000_000, 900_000_000)):
                        self.emit_at(channels, q, epoch + offset_ns)
                    expected[selected] = samples[-1][selected]
                    self.sent_target(channels, expected)
                    self.assertTrue(channels.wait_for(lambda: len(channels.records) >= first_write + 9))
                    action, frozen = session.end_recording()
                    self.assertEqual(action.group, group)
                    self.assertEqual(action.q.shape, (3, len(selected)))
                    np.testing.assert_array_equal(action.t, [0, 0.25, 0.8])
                    np.testing.assert_array_equal(action.q, np.array(samples)[:, selected])
                    np.testing.assert_array_equal(frozen, expected)
                    self.assertFalse(action.q.flags.writeable)
                    self.assertFalse(action.t.flags.writeable)
                    self.assertGreater(len(channels.records) - first_write, len(action.t))
                    frozen_start = len(channels.records)
                    channels.emit(message=self.feedback(HOME + 0.5))
                    self.assertTrue(channels.wait_for(lambda: len(channels.records) >= frozen_start + 6))
                    for command in channels.commands[frozen_start:]:
                        np.testing.assert_array_equal(field(command, "q"), expected)
                        np.testing.assert_array_equal(field(command, "dq"), np.zeros(17))
                        np.testing.assert_array_equal(field(command, "kp"), teach_kp)
                        np.testing.assert_array_equal(field(command, "kd"), teach_kd)
                    before_hold = len(channels.records)
                    session.hold()
                    for command in channels.commands[before_hold:]:
                        np.testing.assert_array_equal(field(command, "q"), expected)
                        np.testing.assert_array_equal(field(command, "dq"), np.zeros(17))
                    np.testing.assert_array_equal(field(channels.commands[-1], "kp"), hold_kp)
                    np.testing.assert_array_equal(field(channels.commands[-1], "kd"), hold_kd)
                self.assertFalse(session._sender.is_alive())
                self.assertEqual(channels.commands[-1].motor_cmd[29].q, 0)

    def test_zero_and_one_sample_freeze_before_rejecting_without_fabricating_an_action(self):
        for group, selected in GROUPS.items():
            for count in (0, 1):
                with self.subTest(group=group, samples=count), self.environment() as (channels, clock):
                    session = robot.RobotSession("offline0", GAINS, Event())
                    selected = list(selected)
                    latest = HOME + np.linspace(0.01, 0.17, 17)
                    expected = HOME.copy()
                    expected[selected] = latest[selected]
                    with session:
                        session.acquire()
                        channels.emit(message=self.feedback(latest))
                        session.follow(group)
                        epoch = clock.monotonic_ns()
                        with patch.object(robot.time, "monotonic_ns", return_value=epoch):
                            session.begin_recording()
                        if count:
                            latest = latest + 0.1
                            self.emit_at(channels, latest, epoch + 100_000_000)
                            expected[selected] = latest[selected]
                        self.sent_target(channels, expected)
                        with self.assertRaisesRegex(ValueError, "两个真实反馈样本"):
                            session.end_recording()
                        channels.emit(message=self.feedback(HOME + 0.7))
                        start = len(channels.records)
                        self.assertTrue(channels.wait_for(lambda: len(channels.records) >= start + 4))
                        for command in channels.commands[start:]:
                            np.testing.assert_array_equal(field(command, "q"), expected)
                            np.testing.assert_array_equal(field(command, "dq"), np.zeros(17))
                        session.hold()
                    self.assertFalse(session._sender.is_alive())

    def deferred_feedback(self, channels, q):
        entered, resume = Event(), Event()
        original = self.feedback(q)

        class Deferred:
            @property
            def motor_state(self):
                entered.set()
                if not resume.wait(2):
                    raise RuntimeError("测试未释放反馈读取")
                return original.motor_state

        thread = Thread(target=lambda: channels.emit(message=Deferred()), daemon=False)
        thread.start()
        self.assertTrue(entered.wait(2))
        return thread, resume

    def test_callbacks_crossing_begin_and_end_do_not_enter_or_change_the_frozen_recording(self):
        with self.environment() as (channels, clock):
            session = robot.RobotSession("offline0", GAINS, Event())
            with session:
                session.acquire()
                session.follow("arms_waist")
                old_thread, old_resume = self.deferred_feedback(channels, HOME + 0.1)
                try:
                    session.begin_recording()
                finally:
                    old_resume.set()
                old_thread.join(2)
                self.assertFalse(old_thread.is_alive())
                epoch = clock.monotonic_ns()
                samples = [HOME + 0.2, HOME + 0.3]
                self.emit_at(channels, samples[0], epoch + 100_000_000)
                self.emit_at(channels, samples[1], epoch + 400_000_000)
                self.sent_target(channels, samples[1])
                clock.advance(2)
                pending, resume = self.deferred_feedback(channels, HOME + 0.8)
                try:
                    action, frozen = session.end_recording()
                finally:
                    resume.set()
                pending.join(2)
                self.assertFalse(pending.is_alive())
                np.testing.assert_array_equal(action.q, samples)
                np.testing.assert_array_equal(action.t, [0, 0.3])
                np.testing.assert_array_equal(frozen, samples[-1])
                start = len(channels.records)
                self.assertTrue(channels.wait_for(lambda: len(channels.records) >= start + 4))
                for command in channels.commands[start:]:
                    np.testing.assert_array_equal(field(command, "q"), frozen)
                session.hold()

    def test_cancel_during_follow_or_gain_transition_releases_from_the_last_sent_gains(self):
        hold_kp, hold_kd = gains()
        for stage in ("follow_ramp", "follow", "hold_ramp"):
            with self.subTest(stage=stage), self.environment(scale=0) as (channels, clock):
                cancel = Event()
                session = robot.RobotSession("offline0", GAINS, cancel)
                ready = Event()
                trigger, released = [], []

                def write(record):
                    command = record.command
                    kp = field(command, "kp")
                    follow_ramp = session._phase == "FOLLOW" and 15 < kp[0] < 60
                    following = session._phase == "FOLLOW" and ready.is_set()
                    hold_ramp = session._phase == "HOLD" and ready.is_set() and 15 < kp[0] < 60
                    should_cancel = {"follow_ramp": follow_ramp, "follow": following, "hold_ramp": hold_ramp}
                    if should_cancel[stage] and not trigger:
                        trigger.append(command)
                        cancel.set()
                    elif session._phase == "RELEASE":
                        released.append(command)
                    clock.advance(0.25)
                    return True

                channels.on_write = write
                with self.assertRaises(InterruptedError):
                    with session:
                        session.acquire()
                        channels.emit(message=self.feedback(HOME + 0.1))
                        session.follow("arms_waist")
                        if stage == "follow":
                            ready.set()
                            self.assertTrue(cancel.wait(2))
                        elif stage == "hold_ramp":
                            session.begin_recording()
                            clock.advance(0.1)
                            channels.emit(message=self.feedback(HOME + 0.1))
                            clock.advance(0.1)
                            channels.emit(message=self.feedback(HOME + 0.2))
                            session.end_recording()
                            ready.set()
                            session.hold()
                self.assertEqual(len(trigger), 1)
                self.assertGreater(len(released), 1)
                for name in ("q", "kp", "kd"):
                    np.testing.assert_array_equal(field(released[0], name), field(trigger[0], name))
                self.assertEqual(released[0].motor_cmd[29].q, trigger[0].motor_cmd[29].q)
                np.testing.assert_array_equal(field(released[-1], "kp"), hold_kp)
                np.testing.assert_array_equal(field(released[-1], "kd"), hold_kd)
                self.assertEqual(released[-1].motor_cmd[29].q, 0)
                for command in released:
                    np.testing.assert_array_equal(field(command, "q"), field(trigger[0], "q"))
                    np.testing.assert_array_equal(field(command, "dq"), np.zeros(17))
                self.assertFalse(session._sender.is_alive())


if __name__ == "__main__":
    unittest.main()
