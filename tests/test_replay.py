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
from unittest.mock import patch

import numpy as np
from fake_channels import HOME, Channels, Clock

from g1_action_studio import cli, planner, robot
from g1_action_studio.model import GROUPS, UPPER_SLOTS

ROOT = Path(__file__).resolve().parents[1]
GAINS = {
    "arms": {"hold_kp": 60, "hold_kd": 1.5, "teach_kp": 20, "teach_kd": 1.5},
    "waist": {"hold_kp": 40, "hold_kd": 2, "teach_kp": 15, "teach_kd": 1},
}


def positions(command):
    return np.array([command.motor_cmd[slot].q for slot in UPPER_SLOTS])


def velocities(command):
    return np.array([command.motor_cmd[slot].dq for slot in UPPER_SLOTS])


@unittest.skipUnless(sys.platform == "linux", "真实 Unitree SDK 回放的交付平台为 Linux")
class ReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from ompl import geometric
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        cls.message_type = LowCmd_
        cls.crc = CRC()
        cls.rrt_type = geometric.RRTConnect

    def setUp(self):
        temporary = self.enterContext(tempfile.TemporaryDirectory())
        self.directory = Path(temporary)
        self.config = self.directory / "gains.json"
        self.config.write_text(json.dumps(GAINS), encoding="utf-8")

    @contextmanager
    def environment(self, **options):
        managed_names = {"g1-action-sender", "g1-terminal-input"}
        self.assertFalse([thread for thread in threading.enumerate() if thread.name in managed_names])
        previous_handler = signal.getsignal(signal.SIGINT)
        terminal_threads = []

        def create_thread(*args, **kwargs):
            thread = threading.Thread(*args, **kwargs)
            terminal_threads.append(thread)
            return thread

        read_fd, write_fd = os.pipe()
        writer = [write_fd]
        clock = Clock(scale=40)
        with ExitStack() as stack:
            stdin = stack.enter_context(os.fdopen(read_fd, "r", encoding="utf-8"))
            stack.enter_context(patch.object(sys, "stdin", stdin))
            stack.enter_context(patch.object(cli, "Thread", side_effect=create_thread))
            stack.enter_context(patch.object(robot, "_factory_initialized", False))
            stack.enter_context(patch.object(robot.time, "monotonic", clock.monotonic))
            stack.enter_context(patch.object(robot.time, "monotonic_ns", clock.monotonic_ns))
            channels = stack.enter_context(Channels(**options))
            try:
                yield channels, writer
            finally:
                if writer:
                    os.close(writer.pop())
        self.assertIs(signal.getsignal(signal.SIGINT), previous_handler)
        self.assertEqual(len(terminal_threads), 1)
        sender_threads = {record.thread for record in channels.records}
        self.assertLessEqual(len(sender_threads), 1)
        for thread in [*terminal_threads, *sender_threads]:
            self.assertFalse(thread.daemon)
            self.assertFalse(thread.is_alive(), thread.name)
        self.assertFalse([thread for thread in threading.enumerate() if thread.name in managed_names])

    def run_cli(self, paths):
        stdout, stderr = io.StringIO(), io.StringIO()
        arguments = [
            "replay",
            *map(str, paths),
            "--interface",
            "offline-test-interface",
            "--config",
            str(self.config),
            "--execute",
        ]
        with redirect_stdout(stdout), redirect_stderr(stderr):
            code = cli.main(arguments)
        return code, stdout.getvalue(), stderr.getvalue()

    def assert_channel_lifecycle(self, channels, publisher=True):
        names = [call[0] for call in channels.calls]
        self.assertEqual(names.count("factory"), 1)
        self.assertEqual(names.count("subscriber"), 1)
        self.assertEqual(names.count("subscriber_init"), 1)
        self.assertEqual(names.count("subscriber_close"), 1)
        for name in ("publisher", "publisher_init", "publisher_close"):
            self.assertEqual(names.count(name), int(publisher), name)
        self.assertEqual(channels.calls[0], ("factory", 0, "offline-test-interface"))

    def assert_completed(self, channels):
        self.assert_channel_lifecycle(channels)
        commands = channels.commands
        weights = np.array([command.motor_cmd[29].q for command in commands])
        self.assertEqual(weights[0], 0)
        self.assertEqual(weights[-1], 0)
        peak = np.flatnonzero(weights == 1)
        self.assertGreater(len(peak), 0)
        self.assertTrue(np.all(np.diff(weights[: peak[0] + 1]) >= 0))
        self.assertTrue(np.all(weights[peak[0] : peak[-1] + 1] == 1))
        self.assertTrue(np.all(np.diff(weights[peak[-1] :]) <= 0))
        np.testing.assert_array_equal(positions(commands[-1]), HOME)
        np.testing.assert_array_equal(velocities(commands[-1]), np.zeros(17))
        self.assertTrue(any(np.any(positions(command) != HOME) for command in commands))
        self.assertTrue(all(record.result is True for record in channels.records))
        for command in commands:
            self.assertEqual(command.crc, self.crc.Crc(command))
            wire = command.serialize()
            decoded = self.message_type.deserialize(wire)
            self.assertEqual(decoded.serialize(), wire)
            self.assertEqual(decoded.crc, self.crc.Crc(decoded))
            np.testing.assert_array_equal(positions(decoded), positions(command).astype(np.float32))
            np.testing.assert_array_equal(velocities(decoded), velocities(command).astype(np.float32))
            for slot in range(12):
                motor = command.motor_cmd[slot]
                self.assertEqual(
                    (motor.mode, motor.q, motor.dq, motor.kp, motor.kd, motor.tau, motor.reserve), (0, 0, 0, 0, 0, 0, 0)
                )
            for index, slot in enumerate(UPPER_SLOTS):
                gains = GAINS["arms" if index < 14 else "waist"]
                self.assertEqual(command.motor_cmd[slot].kp, gains["hold_kp"])
                self.assertEqual(command.motor_cmd[slot].kd, gains["hold_kd"])

    def test_each_group_and_mixed_actions_complete_one_real_cli_session(self):
        sequences = [[group] for group in GROUPS] + [["arms_waist", "left_arm"]]
        for groups in sequences:
            with self.subTest(groups=groups), self.environment(legs=np.arange(12) / 10000) as (channels, _):
                paths = [ROOT / "examples" / f"{group}.json" for group in groups]
                with patch.object(cli, "load_action", wraps=cli.load_action) as actions:
                    with patch.object(cli, "load_gains", wraps=cli.load_gains) as gains:
                        code, stdout, stderr = self.run_cli(paths)
                self.assertEqual((code, stderr), (0, ""))
                self.assertIn("零权重发送完成", stdout)
                self.assertEqual([call.args[0] for call in actions.call_args_list], list(map(str, paths)))
                gains.assert_called_once_with(str(self.config))
                self.assert_completed(channels)
                active = {joint for group in groups for joint in GROUPS[group]}
                inactive = sorted(set(range(17)) - active)
                for command in channels.commands:
                    np.testing.assert_array_equal(positions(command)[inactive], HOME[inactive])
                    np.testing.assert_array_equal(velocities(command)[inactive], np.zeros(len(inactive)))
                if "arms_waist" in groups:
                    self.assertTrue(any(command.motor_cmd[12].dq != 0 for command in channels.commands))

    def test_real_rrt_connections_are_used_before_replay_acquires(self):
        crossing = [0.561, 0.238, -1.245, 0.563, 1.853, 0.501, -1.141]
        path = self.directory / "crossing.json"
        path.write_text(
            json.dumps({"group": "left_arm", "frames": [{"t": 0, "q": crossing}, {"t": 1, "q": crossing}]}),
            encoding="utf-8",
        )
        with self.environment() as (channels, _):
            rrt_path = planner._rrt_path

            def search_before_acquire(*arguments):
                self.assertNotIn("publisher", [call[0] for call in channels.calls])
                self.assertEqual(channels.commands, [])
                return rrt_path(*arguments)

            with patch.object(planner, "_rrt_path", side_effect=search_before_acquire) as search:
                code, _, stderr = self.run_cli([path])
            self.assertEqual((code, stderr), (0, ""))
            self.assertEqual(search.call_count, 2)
            self.assert_completed(channels)

    def test_sigint_freezes_the_last_successful_target_and_releases_without_returning_home(self):
        frozen = []
        with self.environment() as (channels, _):

            def cancel_when_moving(record):
                if not frozen and np.any(velocities(record.command)):
                    frozen.append(positions(record.command))
                    signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
                return True

            channels.on_write = cancel_when_moving
            code, stdout, stderr = self.run_cli([ROOT / "examples" / "arms_waist.json"])
            self.assertEqual((code, stderr), (130, ""))
            self.assertNotIn("流程完成", stdout)
            self.assertEqual(len(frozen), 1)
            commands = channels.commands
            trigger = next(index for index, command in enumerate(commands) if np.any(velocities(command)))
            self.assertFalse(np.array_equal(frozen[0], HOME))
            for command in commands[trigger + 1 :]:
                np.testing.assert_array_equal(positions(command), frozen[0])
                np.testing.assert_array_equal(velocities(command), np.zeros(17))
            self.assertEqual(commands[-1].motor_cmd[29].q, 0)
            self.assert_channel_lifecycle(channels)

    def test_real_terminal_eof_cancels_and_joins_both_threads(self):
        closed = []
        with self.environment() as (channels, writer):

            def eof_when_moving(record):
                if not closed and np.any(velocities(record.command)):
                    closed.append(True)
                    os.close(writer.pop())
                return True

            channels.on_write = eof_when_moving
            code, stdout, stderr = self.run_cli([ROOT / "examples" / "arms_waist.json"])
            self.assertEqual((code, stderr), (130, ""))
            self.assertNotIn("流程完成", stdout)
            self.assertEqual(closed, [True])
            self.assertEqual(channels.commands[-1].motor_cmd[29].q, 0)
            np.testing.assert_array_equal(velocities(channels.commands[-1]), np.zeros(17))
            self.assertFalse(np.array_equal(positions(channels.commands[-1]), HOME))
            self.assert_channel_lifecycle(channels)

    def test_write_false_and_close_error_keep_both_causes_and_stop_writing(self):
        with self.environment() as (channels, _):
            channels.errors["publisher_close"] = RuntimeError("publisher close fault")
            channels.on_write = lambda record: not np.any(velocities(record.command))
            code, stdout, stderr = self.run_cli([ROOT / "examples" / "arms_waist.json"])
            self.assertEqual(code, 1)
            self.assertIn("Write 返回 False", stderr)
            self.assertIn("publisher close fault", stderr)
            self.assertNotIn("流程完成", stdout)
            failures = [index for index, record in enumerate(channels.records) if record.result is False]
            self.assertEqual(failures, [len(channels.records) - 1])
            self.assert_channel_lifecycle(channels)

    def test_collision_is_reported_before_any_publisher_or_command(self):
        q = HOME[:7].copy()
        q[2] = -1.6
        path = self.directory / "collision.json"
        path.write_text(
            json.dumps({"group": "left_arm", "frames": [{"t": 0, "q": q.tolist()}, {"t": 1, "q": q.tolist()}]}),
            encoding="utf-8",
        )
        with self.environment() as (channels, _):
            code, stdout, stderr = self.run_cli([path])
            self.assertEqual(code, 1)
            self.assertIn("碰撞", stderr)
            self.assertNotIn("流程完成", stdout)
            self.assertEqual(channels.commands, [])
            self.assert_channel_lifecycle(channels, publisher=False)


if __name__ == "__main__":
    unittest.main()
