import sys
import unittest
from contextlib import ExitStack, contextmanager
from threading import Event, Thread
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from fake_channels import HOME, Channels, Clock

from g1_action_studio import robot
from g1_action_studio.model import GROUPS, UPPER_SLOTS
from g1_action_studio.motion import connection_curve

GAINS = {
    "arms": {"hold_kp": 60, "hold_kd": 1.5, "teach_kp": 20, "teach_kd": 1.5},
    "waist": {"hold_kp": 40, "hold_kd": 2, "teach_kp": 15, "teach_kd": 1},
}


def positions(command):
    return np.array([command.motor_cmd[slot].q for slot in UPPER_SLOTS])


def velocities(command):
    return np.array([command.motor_cmd[slot].dq for slot in UPPER_SLOTS])


def causes(error):
    if isinstance(error, ExceptionGroup):
        return [cause for item in error.exceptions for cause in causes(item)]
    return [error]


class RecordingTests(unittest.TestCase):
    @contextmanager
    def recording(self, group):
        clock = SimpleNamespace(now=1_000_000_000)
        session = robot.RobotSession("offline0", GAINS, Event())
        message = SimpleNamespace(motor_state=[SimpleNamespace(q=0.0) for _ in range(29)])

        def emit(offset_ns, value):
            clock.now = 1_000_000_000 + offset_ns
            for index, slot in enumerate(UPPER_SLOTS):
                message.motor_state[slot].q = float(HOME[index] + value)
            session._on_feedback(message)

        with patch.object(robot.time, "monotonic_ns", lambda: clock.now):
            emit(0, 0)
            session._phase = "FOLLOW"
            session._group = group
            session._selected = list(GROUPS[group])
            session.begin_recording()
            yield session, emit

    def test_one_khz_feedback_records_50_hz_for_every_group(self):
        for group, width in {"left_arm": 7, "right_arm": 7, "arms": 14, "arms_waist": 17}.items():
            with self.subTest(group=group), self.recording(group) as (session, emit):
                for millisecond in range(1, 28_002):
                    emit(millisecond * 1_000_000, millisecond / 100_000)
                action, _ = session.end_recording()

                self.assertEqual(action.group, group)
                self.assertEqual(action.q.shape, (1401, width))
                saved_ms = np.r_[1, np.arange(20, 28_002, 20)]
                np.testing.assert_array_equal(action.t, (saved_ms - 1) / 1000)
                expected_q = HOME[list(GROUPS[group])] + saved_ms[:, None] / 100_000
                np.testing.assert_array_equal(action.q, expected_q)
                self.assertIsNone(session._publisher)
                self.assertIsNone(session._sender)

    def test_recording_keeps_real_jitter_and_updates_feedback_between_saved_frames(self):
        with self.recording("left_arm") as (session, emit):
            for offset, value in (
                (1_000_000, 0.01),
                (19_900_000, 0.02),
                (20_000_000, 0.03),
                (39_900_000, 0.04),
                (40_100_000, 0.05),
                (60_000_000, 0.06),
                (123_000_000, 0.07),
                (123_100_000, 0.08),
            ):
                emit(offset, value)
                self.assertEqual(session._feedback[0], 1_000_000_000 + offset)
                np.testing.assert_array_equal(session._feedback[1].upper_q, HOME + value)
            action, frozen = session.end_recording()

        np.testing.assert_array_equal(action.t, [0, 0.019, 0.0391, 0.059, 0.122])
        np.testing.assert_array_equal(action.q, HOME[:7] + np.array([0.01, 0.03, 0.05, 0.06, 0.07])[:, None])
        np.testing.assert_array_equal(frozen[:7], action.q[-1])
        np.testing.assert_array_equal(frozen[7:], HOME[7:])
        np.testing.assert_array_equal(session._target_q, frozen)
        np.testing.assert_array_equal(session._feedback[1].upper_q, HOME + 0.08)


@unittest.skipUnless(sys.platform == "linux", "真实 Unitree SDK 生命周期测试的交付平台为 Linux")
class RobotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 不跳过缺失 SDK/IDL/CRC 库；Linux 正式测试使用官方消息及 CRC。
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
        from unitree_sdk2py.utils.crc import CRC

        unitree_hg_msg_dds__LowState_()
        CRC()

    @contextmanager
    def environment(self, clock_scale=40, **options):
        clock = Clock(scale=clock_scale)
        with ExitStack() as stack:
            stack.enter_context(patch.object(robot, "_factory_initialized", False))
            stack.enter_context(patch.object(robot.time, "monotonic", clock.monotonic))
            stack.enter_context(patch.object(robot.time, "monotonic_ns", clock.monotonic_ns))
            stack.enter_context(patch.object(robot, "DT", 0.001))
            channels = stack.enter_context(Channels(**options))
            yield channels, clock

    def background(self, operation):
        call = SimpleNamespace(done=Event(), errors=[], result=None)

        def run():
            try:
                call.result = operation()
            except Exception as error:
                call.errors.append(error)
            finally:
                call.done.set()

        call.thread = Thread(target=run, daemon=False)
        call.thread.start()
        return call

    def finished(self, call):
        call.thread.join(2)
        self.assertFalse(call.thread.is_alive(), "测试调用没有退出")

    def test_factory_once_fixed_feedback_non_daemon_sender_and_idempotent_close(self):
        with self.environment(legs=np.linspace(-0.1, 0.1, 12)) as (channels, _):
            cancel = Event()
            for _ in range(2):
                session = robot.RobotSession("offline0", GAINS, cancel)
                with session:
                    np.testing.assert_array_equal(session.start_pose.upper_q, HOME)
                    np.testing.assert_array_equal(session.start_pose.leg_q, channels.legs)
                    self.assertFalse(any(call[0] == "publisher" for call in channels.calls[-2:]))
                    channels.emit(upper=HOME + 0.3)
                    np.testing.assert_array_equal(session.start_pose.upper_q, HOME)
                    session.acquire()
                    sender = session._sender
                    self.assertFalse(sender.daemon)
                    self.assertTrue(sender.is_alive())
                    self.assertTrue(all(record.thread is sender for record in channels.records[-2:]))
                    with patch.object(sender, "join", wraps=sender.join) as join:
                        session.close()
                        writes = len(channels.records)
                        session.close()
                        self.assertEqual(join.call_count, 1)
                        self.assertEqual(len(channels.records), writes)
                    self.assertFalse(sender.is_alive())
                    self.assertFalse(cancel.is_set())
                    self.assertEqual(channels.commands[-1].motor_cmd[29].q, 0)
                    np.testing.assert_array_equal(positions(channels.commands[-1]), HOME)
            names = [call[0] for call in channels.calls]
            self.assertEqual(names.count("factory"), 1)
            self.assertEqual(names.count("subscriber"), 2)
            self.assertEqual(names.count("publisher"), 2)
            self.assertEqual(names.count("publisher_close"), 2)
            self.assertEqual(names.count("subscriber_close"), 2)
            self.assertIn(("factory", 0, "offline0"), channels.calls)
            self.assertTrue(all(call[1] == 0 for call in channels.calls if call[0] == "subscriber_init"))
            self.assertEqual(channels.close_while_sender_alive, [])

    def test_waiting_for_first_feedback_can_cancel_without_a_publisher(self):
        with self.environment(initial=False) as (channels, _):
            cancel = Event()
            session = robot.RobotSession("offline0", GAINS, cancel)
            call = self.background(session.__enter__)
            self.assertTrue(channels.wait_for(lambda: channels.callback is not None))
            self.assertFalse(call.done.is_set())
            cancel.set()
            self.finished(call)
            self.assertIsInstance(call.errors[0], InterruptedError)
            self.assertIsNone(session.start_pose)
            self.assertEqual(channels.records, [])
            self.assertFalse(any(item[0] == "publisher" for item in channels.calls))
            self.assertEqual([item[0] for item in channels.calls].count("subscriber_close"), 1)
            session.close()

    def test_half_acquire_cancellation_releases_from_the_last_weight_once(self):
        with self.environment(clock_scale=0) as (channels, clock):
            cancel = Event()
            session = robot.RobotSession("offline0", GAINS, cancel)
            triggered = []

            def write(record):
                weight = record.command.motor_cmd[29].q
                if session._phase == "ACQUIRE" and weight >= 0.3 and not triggered:
                    triggered.append((len(channels.records) - 1, weight))
                    cancel.set()
                clock.advance(0.25)
                return True

            channels.on_write = write
            with self.assertRaises(InterruptedError):
                with session:
                    session.acquire()
            self.assertEqual(len(triggered), 1)
            index, initial_weight = triggered[0]
            self.assertLess(initial_weight, 1)
            weights = [command.motor_cmd[29].q for command in channels.commands[index:]]
            self.assertEqual(weights[-1], 0)
            self.assertTrue(all(later <= earlier for earlier, later in zip(weights, weights[1:])))
            self.assertLess(max(weights), 1)
            self.assertFalse(session._sender.is_alive())

    def test_sender_releases_while_the_main_thread_has_not_called_close(self):
        with self.environment() as (channels, _):
            cancel = Event()
            session = robot.RobotSession("offline0", GAINS, cancel)
            session.__enter__()
            try:
                session.acquire()
                previous = len(channels.records)
                cancel.set()
                self.assertTrue(
                    channels.wait_for(
                        lambda: any(record.command.motor_cmd[29].q == 0 for record in channels.records[previous:])
                    )
                )
                session._sender.join(2)
                self.assertFalse(session._sender.is_alive())
                self.assertFalse(session._closing.is_set())
            finally:
                session.close()
            self.assertFalse(session._sender.is_alive())

    def test_play_waits_for_successful_exact_tail_and_close_keeps_that_target(self):
        with self.environment() as (channels, _):
            target = HOME.copy()
            target[0] += 0.2
            tail_entered, allow_tail = Event(), Event()
            session = robot.RobotSession("offline0", GAINS, Event())

            def write(record):
                if session._phase == "PLAY" and np.array_equal(positions(record.command), target):
                    tail_entered.set()
                    self.assertTrue(allow_tail.wait(2))
                return True

            with session:
                session.acquire()
                channels.on_write = write
                call = self.background(lambda: session.play(connection_curve([HOME, target], 1)))
                try:
                    self.assertTrue(tail_entered.wait(2))
                    self.assertFalse(call.done.is_set())
                finally:
                    allow_tail.set()
                self.finished(call)
                self.assertEqual(call.errors, [])
                self.assertTrue(
                    any(
                        record.result is True
                        for record in channels.records
                        if np.array_equal(positions(record.command), target)
                    )
                )
            np.testing.assert_array_equal(positions(channels.commands[-1]), target)
            self.assertEqual(channels.commands[-1].motor_cmd[29].q, 0)

    def test_clock_gap_skips_to_current_curve_time_without_a_catchup_burst(self):
        with self.environment() as (channels, clock):
            session = robot.RobotSession("offline0", GAINS, Event())
            target = HOME.copy()
            target[0] += 0.2
            trajectory = connection_curve([HOME, target], 1)
            original_sample = trajectory.sample
            samples = []

            def sample(curve, elapsed):
                q, dq = original_sample(elapsed)
                samples.append((elapsed, q.copy(), dq.copy()))
                if len(samples) == 1:
                    clock.advance(2.0)
                return q, dq

            with session:
                session.acquire()
                with patch.object(type(trajectory), "sample", sample):
                    session.play(trajectory)
            self.assertEqual(len(samples), 2)
            self.assertEqual(samples[0][0], 0)
            self.assertGreaterEqual(samples[-1][0], trajectory.duration)
            np.testing.assert_array_equal(samples[0][1], HOME)
            np.testing.assert_array_equal(samples[-1][1], target)
            np.testing.assert_array_equal(samples[-1][2], np.zeros(17))
            np.testing.assert_array_equal(positions(channels.commands[-1]), target)
            np.testing.assert_array_equal(velocities(channels.commands[-1]), np.zeros(17))

    def test_play_cancel_freezes_last_successful_target_instead_of_returning_home(self):
        with self.environment() as (channels, _):
            cancel = Event()
            session = robot.RobotSession("offline0", GAINS, cancel)
            target = HOME.copy()
            target[0] += 0.4
            frozen = []
            released = []

            def write(record):
                q = positions(record.command)
                if session._phase == "PLAY" and q[0] > HOME[0] + 0.05 and not frozen:
                    frozen.append(q)
                    cancel.set()
                elif session._phase == "RELEASE":
                    released.append(record.command)
                return True

            with self.assertRaises(InterruptedError):
                with session:
                    session.acquire()
                    channels.on_write = write
                    session.play(connection_curve([HOME, target], 4))
            self.assertEqual(len(frozen), 1)
            self.assertGreater(len(released), 1)
            for command in released:
                np.testing.assert_array_equal(positions(command), frozen[0])
                np.testing.assert_array_equal(velocities(command), np.zeros(17))
            self.assertEqual(released[-1].motor_cmd[29].q, 0)

    def test_write_false_and_exception_stop_every_phase_without_any_further_write(self):
        for phase in ("ACQUIRE", "HOLD", "PLAY", "RELEASE"):
            for mode in ("false", "exception"):
                with self.subTest(phase=phase, mode=mode), self.environment() as (channels, _):
                    cancel = Event()
                    session = robot.RobotSession("offline0", GAINS, cancel)
                    failed = []

                    def write(record):
                        if session._phase == phase:
                            failed.append(record)
                            if mode == "exception":
                                raise OSError(f"write-{phase}")
                            return False
                        return True

                    channels.on_write = write
                    with self.assertRaises(Exception) as caught:
                        with session:
                            session.acquire()
                            if phase == "HOLD":
                                self.assertTrue(channels.wait_for(lambda: bool(failed)))
                            elif phase == "PLAY":
                                target = HOME.copy()
                                target[0] += 0.1
                                session.play(connection_curve([HOME, target], 1))
                    self.assertEqual(len(failed), 1)
                    self.assertIs(channels.records[-1], failed[0])
                    self.assertTrue(
                        any("write-" in str(error) or "False" in str(error) for error in causes(caught.exception))
                    )
                    self.assertEqual(len(causes(caught.exception)), 1)
                    self.assertTrue(cancel.is_set())
                    self.assertFalse(session._sender.is_alive())
                    self.assertEqual([item[0] for item in channels.calls].count("publisher_close"), 1)
                    self.assertEqual([item[0] for item in channels.calls].count("subscriber_close"), 1)

    def test_feedback_read_error_releases_the_last_command_and_is_reported(self):
        with self.environment() as (channels, _):
            cancel = Event()
            session = robot.RobotSession("offline0", GAINS, cancel)
            with self.assertRaises(IndexError):
                with session:
                    session.acquire()
                    channels.emit(message=SimpleNamespace(motor_state=[]))
                    session._sender.join(2)
                    self.assertFalse(session._sender.is_alive())
                    self.assertTrue(cancel.is_set())
            self.assertEqual(channels.commands[-1].motor_cmd[29].q, 0)
            self.assertFalse(session._sender.is_alive())

    def test_partial_initialization_closes_every_resource_already_owned(self):
        for stage in ("factory", "subscriber", "subscriber_init", "publisher", "publisher_init"):
            with self.subTest(stage=stage), self.environment() as (channels, _):
                channels.errors[stage] = OSError(stage)
                session = robot.RobotSession("offline0", GAINS, Event())
                with self.assertRaisesRegex(OSError, stage):
                    with session:
                        session.acquire()
                names = [item[0] for item in channels.calls]
                self.assertEqual(names.count("subscriber_close"), int(stage not in ("factory", "subscriber")))
                self.assertEqual(names.count("publisher_close"), int(stage == "publisher_init"))
                self.assertEqual(channels.records, [])
                self.assertIsNone(session._sender)

    def test_close_failure_does_not_hide_business_failure_or_skip_other_channel(self):
        for failing in (("publisher_close",), ("subscriber_close",), ("publisher_close", "subscriber_close")):
            with self.subTest(failing=failing), self.environment() as (channels, _):
                for name in failing:
                    channels.errors[name] = OSError(name)
                session = robot.RobotSession("offline0", GAINS, Event())
                with self.assertRaises(ExceptionGroup) as caught:
                    with session:
                        session.acquire()
                        raise ValueError("business-failure")
                errors = causes(caught.exception)
                self.assertEqual({str(error) for error in errors}, {"business-failure", *failing})
                self.assertFalse(session._sender.is_alive())
                names = [item[0] for item in channels.calls]
                self.assertEqual(names.count("publisher_close"), 1)
                self.assertEqual(names.count("subscriber_close"), 1)
                session.close()


if __name__ == "__main__":
    unittest.main()
