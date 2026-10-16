import io
import json
import os
import signal
import sys
import tempfile
import threading
import unittest
from contextlib import ExitStack, contextmanager, redirect_stderr, redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from fake_channels import HOME, Channels, Clock

from g1_action_studio import cli, robot, teach
from g1_action_studio.model import GROUPS, UPPER_SLOTS, ArmModel
from g1_action_studio.motion import action_curve, load_action

GAINS = {
    "arms": {"hold_kp": 60, "hold_kd": 1.5, "teach_kp": 20, "teach_kd": 1.5},
    "waist": {"hold_kp": 40, "hold_kd": 2, "teach_kp": 15, "teach_kd": 1},
}


def positions(command):
    return np.array([command.motor_cmd[slot].q for slot in UPPER_SLOTS])


def velocities(command):
    return np.array([command.motor_cmd[slot].dq for slot in UPPER_SLOTS])


def expanded(group, feedback):
    q = HOME.copy()
    selected = list(GROUPS[group])
    q[selected] = feedback[selected]
    return q


class PromptOutput(io.StringIO):
    def __init__(self):
        super().__init__()
        self.expected = None
        self.stage = 0
        self.on_prompt = None

    def write(self, text):
        result = super().write(text)
        if self.expected is not None and text == self.expected:
            self.expected = None
            self.on_prompt(self.stage)
        return result


@unittest.skipUnless(sys.platform == "linux", "真实 Unitree SDK 示教的交付平台为 Linux")
class TeachTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ompl import geometric
        from unitree_sdk2py.utils.crc import CRC

        cls.crc = CRC()
        cls.rrt_type = geometric.RRTConnect

    def setUp(self):
        self.directory = Path(self.enterContext(tempfile.TemporaryDirectory()))
        self.session_id = 0
        self.config = self.directory / "gains.json"
        self.config.write_text(json.dumps(GAINS), encoding="utf-8")

    @contextmanager
    def environment(self, group="arms_waist", count=3):
        self.session_id += 1
        managed = {"g1-action-sender", "g1-terminal-input"}
        self.assertFalse([thread for thread in threading.enumerate() if thread.name in managed])
        previous_handler = signal.getsignal(signal.SIGINT)
        clock = Clock(scale=40)
        selected = list(GROUPS[group])
        follow = HOME + 0.08
        follow[selected] = HOME[selected] + 0.01
        samples = []
        for factor in (1, 2, 1.5)[:count]:
            feedback = HOME + 0.05
            feedback[selected] = HOME[selected] + factor * np.arange(1, len(selected) + 1) / 1000
            samples.append(feedback)
        output = PromptOutput()
        terminal_threads, producers = [], []
        original_wait = cli._TerminalInput.wait_enter

        def create_thread(*args, **kwargs):
            thread = threading.Thread(*args, **kwargs)
            terminal_threads.append(thread)
            return thread

        def wait_enter(terminal, prompt):
            output.stage += 1
            output.expected = prompt
            return original_wait(terminal, prompt)

        read_fd, write_fd = os.pipe()
        harness = SimpleNamespace(
            group=group,
            clock=clock,
            selected=selected,
            follow=follow,
            samples=samples,
            output=self.directory / f"{group}-{self.session_id}.json",
            writer=[write_fd],
            producers=producers,
            stdout=output,
        )
        with ExitStack() as stack:
            stdin = stack.enter_context(os.fdopen(read_fd, "r", encoding="utf-8"))
            stack.enter_context(patch.object(sys, "stdin", stdin))
            stack.enter_context(patch.object(cli, "Thread", side_effect=create_thread))
            stack.enter_context(patch.object(cli._TerminalInput, "wait_enter", wait_enter))
            stack.enter_context(patch.object(robot, "_factory_initialized", False))
            stack.enter_context(patch.object(robot.time, "monotonic", clock.monotonic))
            stack.enter_context(patch.object(robot.time, "monotonic_ns", clock.monotonic_ns))
            harness.channels = stack.enter_context(Channels())

            def on_prompt(stage):
                if stage == 1:
                    harness.channels.emit(harness.follow)
                    target = expanded(group, harness.follow)
                    self.assertTrue(
                        harness.channels.wait_for(
                            lambda: (
                                bool(harness.channels.commands)
                                and np.array_equal(positions(harness.channels.commands[-1]), target)
                            )
                        )
                    )
                else:
                    for feedback in harness.samples:
                        clock.advance(0.05)
                        harness.channels.emit(feedback)
                os.write(harness.writer[0], b"\n")

            harness.on_prompt = on_prompt
            output.on_prompt = lambda stage: harness.on_prompt(stage)
            try:
                yield harness
            finally:
                for producer in producers:
                    producer.join()
                if harness.writer:
                    os.close(harness.writer.pop())
        self.assertIs(signal.getsignal(signal.SIGINT), previous_handler)
        self.assertEqual(len(terminal_threads), 1)
        senders = {record.thread for record in harness.channels.records}
        self.assertEqual(len(senders), 1)
        for thread in [*terminal_threads, *senders, *producers]:
            self.assertFalse(thread.daemon)
            self.assertFalse(thread.is_alive(), thread.name)
        self.assertFalse([thread for thread in threading.enumerate() if thread.name in managed])
        names = [call[0] for call in harness.channels.calls]
        for name in ("factory", "subscriber", "publisher", "subscriber_close", "publisher_close"):
            self.assertEqual(names.count(name), 1, name)
        self.assertEqual(harness.channels.close_while_sender_alive, [])

    def run_cli(self, harness):
        stderr = io.StringIO()
        arguments = [
            "teach",
            "--interface",
            "offline-test-interface",
            "--group",
            harness.group,
            "--config",
            str(self.config),
            "--output",
            str(harness.output),
            "--execute",
        ]
        with redirect_stdout(harness.stdout), redirect_stderr(stderr):
            code = cli.main(arguments)
        return code, harness.stdout.getvalue(), stderr.getvalue()

    def assert_release(self, harness):
        commands = harness.channels.commands
        self.assertEqual(commands[0].motor_cmd[29].q, 0)
        self.assertEqual(commands[-1].motor_cmd[29].q, 0)
        self.assertEqual(max(command.motor_cmd[29].q for command in commands), 1)
        np.testing.assert_array_equal(velocities(commands[-1]), np.zeros(17))
        self.assertTrue(all(record.result is True for record in harness.channels.records))

    def assert_saved(self, harness):
        data = json.loads(harness.output.read_text(encoding="utf-8"))
        self.assertEqual(set(data), {"group", "frames"})
        self.assertTrue(all(set(frame) == {"t", "q"} for frame in data["frames"]))
        action = load_action(harness.output)
        self.assertEqual(action.group, harness.group)
        self.assertEqual(action.q.shape, (len(harness.samples), len(harness.selected)))
        np.testing.assert_array_equal(action.q, np.array(harness.samples)[:, harness.selected])
        self.assertEqual(action.t[0], 0)
        self.assertTrue(np.all(np.diff(action.t) > 0))
        curve = action_curve(action, HOME, 1)
        np.testing.assert_array_equal(curve.first_q, expanded(harness.group, harness.samples[0]))
        np.testing.assert_array_equal(curve.last_q, expanded(harness.group, harness.samples[-1]))
        return action

    def test_four_groups_freeze_save_then_restore_gains_and_return_using_the_same_file_format(self):
        for group in GROUPS:
            with self.subTest(group=group), self.environment(group) as harness:
                save = teach.save_action
                connect = teach.connect
                saved = []

                def save_while_feedback_changes(action, path):
                    frozen = expanded(group, harness.samples[-1])
                    count = len(harness.channels.records)
                    harness.channels.emit(HOME + 0.3)
                    self.assertTrue(harness.channels.wait_for(lambda: len(harness.channels.records) >= count + 3))
                    for command in harness.channels.commands[count + 1 :]:
                        np.testing.assert_array_equal(positions(command), frozen)
                        np.testing.assert_array_equal(velocities(command), np.zeros(17))
                        for joint, slot in enumerate(UPPER_SLOTS):
                            gains = GAINS["arms" if joint < 14 else "waist"]
                            mode = "teach" if joint in harness.selected else "hold"
                            self.assertEqual(command.motor_cmd[slot].kp, gains[f"{mode}_kp"])
                            self.assertEqual(command.motor_cmd[slot].kd, gains[f"{mode}_kd"])
                    save(action, path)
                    saved.append(True)

                def connect_after_save(model, first, last, active_groups, seconds, cancel):
                    self.assertEqual(saved, [True])
                    self.assertTrue(harness.output.is_file())
                    np.testing.assert_array_equal(first, expanded(group, harness.samples[-1]))
                    np.testing.assert_array_equal(last, HOME)
                    self.assertEqual(active_groups, {group})
                    command = harness.channels.commands[-1]
                    for joint, slot in enumerate(UPPER_SLOTS):
                        gains = GAINS["arms" if joint < 14 else "waist"]
                        self.assertEqual(command.motor_cmd[slot].kp, gains["hold_kp"])
                        self.assertEqual(command.motor_cmd[slot].kd, gains["hold_kd"])
                    return connect(model, first, last, active_groups, seconds, cancel)

                with patch.object(teach, "save_action", side_effect=save_while_feedback_changes) as saving:
                    with patch.object(teach, "connect", side_effect=connect_after_save) as returning:
                        code, stdout, stderr = self.run_cli(harness)
                self.assertEqual((code, stderr), (0, ""))
                self.assertIn("零权重发送完成", stdout)
                self.assertEqual(saving.call_count, 1)
                self.assertEqual(returning.call_count, 1)
                self.assert_saved(harness)
                self.assert_release(harness)
                np.testing.assert_array_equal(positions(harness.channels.commands[-1]), HOME)
                inactive = sorted(set(range(17)) - set(harness.selected))
                for command in harness.channels.commands:
                    np.testing.assert_array_equal(positions(command)[inactive], HOME[inactive])
                    self.assertEqual(command.crc, self.crc.Crc(command))

    def test_zero_or_one_feedback_sample_does_not_save_or_plan_a_return(self):
        for count in (0, 1):
            with self.subTest(count=count), self.environment(count=count) as harness:
                with patch.object(teach, "save_action", wraps=teach.save_action) as saving:
                    with patch.object(teach, "connect", wraps=teach.connect) as returning:
                        code, _, stderr = self.run_cli(harness)
                self.assertEqual(code, 1)
                self.assertIn("两个真实反馈样本", stderr)
                self.assertFalse(harness.output.exists())
                saving.assert_not_called()
                returning.assert_not_called()
                self.assert_release(harness)

    def test_existing_output_and_real_file_write_failure_do_not_start_a_return(self):
        for failure in ("existing", "missing_parent"):
            with self.subTest(failure=failure), self.environment() as harness:
                if failure == "existing":
                    harness.output.write_bytes(b"existing recording")
                else:
                    harness.output = self.directory / "missing" / "action.json"
                with patch.object(teach, "connect", wraps=teach.connect) as returning:
                    code, _, stderr = self.run_cli(harness)
                self.assertEqual(code, 1)
                self.assertIn(str(harness.output), stderr)
                returning.assert_not_called()
                if failure == "existing":
                    self.assertEqual(harness.output.read_bytes(), b"existing recording")
                else:
                    self.assertFalse(harness.output.exists())
                self.assert_release(harness)
                self.assertFalse(np.array_equal(positions(harness.channels.commands[-1]), HOME))

    def test_invalid_return_keeps_the_successfully_saved_real_action(self):
        with self.environment("left_arm") as harness:
            harness.samples[-1][2] = -1.6
            code, _, stderr = self.run_cli(harness)
            self.assertEqual(code, 1)
            self.assertIn("碰撞", stderr)
            self.assert_saved(harness)
            self.assert_release(harness)

    def test_real_rrt_without_exact_return_keeps_the_recording(self):
        with self.environment("left_arm") as harness:
            for feedback in harness.samples:
                feedback[:7] = HOME[:7]
                feedback[2] = -2.4

            def narrow_search(legs):
                model = ArmModel(legs)
                ranges = model.ranges.copy()
                for joint in set(GROUPS["left_arm"]) - {2}:
                    ranges[joint] = HOME[joint] + np.array([-1e-6, 1e-6])
                model.ranges = ranges
                return model

            with patch.object(teach, "ArmModel", side_effect=narrow_search):
                code, _, stderr = self.run_cli(harness)
            self.assertEqual(code, 1)
            self.assertTrue("近似" in stderr or "未找到精确解" in stderr, stderr)
            self.assert_saved(harness)
            self.assert_release(harness)
            self.assertFalse(np.array_equal(positions(harness.channels.commands[-1]), HOME))

    def test_cancellation_in_follow_record_saved_planning_and_return_play(self):
        for phase in ("follow", "record", "saved", "planning", "play"):
            with self.subTest(phase=phase), self.environment() as harness:
                prompt = harness.on_prompt
                save = teach.save_action
                connect = teach.connect
                play = robot.RobotSession.play
                cancelled = []
                submitted = []

                def cancel():
                    if not cancelled:
                        cancelled.append(True)
                        signal.getsignal(signal.SIGINT)(signal.SIGINT, None)

                def on_prompt(stage):
                    if (phase, stage) in (("follow", 1), ("record", 2)):
                        if stage == 2:
                            harness.channels.emit(harness.samples[0])
                        cancel()
                    else:
                        prompt(stage)

                def after_save(action, path):
                    save(action, path)
                    if phase == "saved":
                        cancel()

                def while_planning(*arguments):
                    if phase == "planning":
                        cancel()
                    return connect(*arguments)

                def while_playing(session, trajectory):
                    submitted.append(trajectory)
                    return play(session, trajectory)

                def on_write(record):
                    if phase == "play" and np.any(velocities(record.command)):
                        cancel()
                    return True

                harness.on_prompt = on_prompt
                harness.channels.on_write = on_write
                with patch.object(teach, "save_action", side_effect=after_save):
                    with patch.object(teach, "connect", side_effect=while_planning) as returning:
                        with patch.object(robot.RobotSession, "play", while_playing):
                            code, stdout, stderr = self.run_cli(harness)
                self.assertEqual((code, stderr), (130, ""))
                self.assertNotIn("流程完成", stdout)
                self.assertEqual(cancelled, [True])
                if phase in ("follow", "record"):
                    self.assertFalse(harness.output.exists())
                else:
                    self.assert_saved(harness)
                self.assertEqual(returning.call_count, int(phase in ("planning", "play")))
                self.assertEqual(len(submitted), int(phase == "play"))
                self.assert_release(harness)
                if phase == "play":
                    self.assertFalse(np.array_equal(positions(harness.channels.commands[-1]), HOME))

    def test_two_enters_in_one_chunk_do_not_skip_the_real_recording_prompt(self):
        with self.environment() as harness:
            second_enter_sent = threading.Event()
            save = teach.save_action

            def on_prompt(stage):
                if stage == 1:
                    harness.channels.emit(harness.follow)
                    os.write(harness.writer[0], b"\n\n")
                else:
                    for feedback in harness.samples:
                        harness.clock.advance(0.05)
                        harness.channels.emit(feedback)

                    def send_second_enter():
                        second_enter_sent.set()
                        if harness.writer:
                            os.write(harness.writer[0], b"\n")

                    producer = threading.Timer(0.05, send_second_enter)
                    producer.daemon = False
                    harness.producers.append(producer)
                    producer.start()

            def save_after_second_enter(action, path):
                self.assertTrue(second_enter_sent.is_set(), "旧回车跨过了录制阶段")
                save(action, path)

            harness.on_prompt = on_prompt
            with patch.object(teach, "save_action", side_effect=save_after_second_enter):
                code, _, stderr = self.run_cli(harness)
            self.assertEqual((code, stderr), (0, ""))
            self.assertTrue(second_enter_sent.is_set())
            self.assert_saved(harness)
            self.assert_release(harness)

    def test_real_stdin_eof_during_follow_cancels_without_a_partial_file(self):
        with self.environment() as harness:

            def close_stdin(stage):
                self.assertEqual(stage, 1)
                os.close(harness.writer.pop())

            harness.on_prompt = close_stdin
            code, stdout, stderr = self.run_cli(harness)
            self.assertEqual((code, stderr), (130, ""))
            self.assertNotIn("流程完成", stdout)
            self.assertFalse(harness.output.exists())
            self.assert_release(harness)


if __name__ == "__main__":
    unittest.main()
