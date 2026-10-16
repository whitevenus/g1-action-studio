import contextlib
import copy
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from threading import Event
from unittest.mock import patch

from g1_action_studio import cli

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


class GainTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent)
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "gains.json"
        self.gains = json.loads((EXAMPLES / "gains.json").read_text(encoding="utf-8"))

    def write(self, data):
        self.path.write_text(json.dumps(data), encoding="utf-8")
        return self.path

    def test_exact_eight_gains_allow_zero_and_have_no_threshold(self):
        values = copy.deepcopy(self.gains)
        values["arms"]["hold_kp"] = 0
        values["waist"]["teach_kd"] = 10000
        loaded = cli.load_gains(self.write(values))
        self.assertEqual(loaded, values)
        self.assertTrue(all(isinstance(value, float) for group in loaded.values() for value in group.values()))

    def test_gains_reject_old_extra_missing_and_invalid_values_with_path(self):
        invalid = [[], {"hold_kp": 60}, {**self.gains, "notes": "unsupported"}]
        for group in ("arms", "waist"):
            data = copy.deepcopy(self.gains)
            data[group]["qualification"] = "unsupported"
            invalid.append(data)
            data = copy.deepcopy(self.gains)
            del data[group]["teach_kd"]
            invalid.append(data)
            for value in (True, None, "60", -0.1, float("nan"), float("inf")):
                data = copy.deepcopy(self.gains)
                data[group]["hold_kp"] = value
                invalid.append(data)
        for data in invalid:
            with self.subTest(data=data), self.assertRaisesRegex(ValueError, "gains.json"):
                cli.load_gains(self.write(data))

    def test_replay_requires_execute_and_rejects_old_or_invalid_arguments_before_dispatch(self):
        base = [
            "replay",
            str(EXAMPLES / "left_arm.json"),
            "--interface",
            "offline-test",
            "--config",
            str(EXAMPLES / "gains.json"),
        ]
        cases = [base]
        cases += [base + ["--execute", flag, "old"] for flag in ("--side", "--profile", "--start")]
        cases += [
            base + ["--execute", option, value]
            for option in ("--speed", "--transition-seconds")
            for value in ("0", "-1", "nan", "inf", "bad")
        ]
        with patch.object(cli, "run_replay", side_effect=AssertionError("用法错误不得连接设备")) as replay:
            for arguments in cases:
                with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as error:
                        cli.main(arguments)
                    self.assertEqual(error.exception.code, 2)
            replay.assert_not_called()


@unittest.skipUnless(sys.platform == "linux", "终端设备会话以 Linux 为交付平台")
class TerminalInputTests(unittest.TestCase):
    def setUp(self):
        self.stack = contextlib.ExitStack()
        self.addCleanup(self.stack.close)
        read_fd, write_fd = os.pipe()
        self.reader = self.stack.enter_context(os.fdopen(read_fd, "r"))
        self.writer = self.stack.enter_context(os.fdopen(write_fd, "w"))
        self.stack.enter_context(patch.object(sys, "stdin", self.reader))
        self.cancel = Event()

    def test_eof_cancels_and_input_thread_is_joined(self):
        with cli._TerminalInput(self.cancel) as terminal:
            self.assertFalse(terminal.thread.daemon)
            self.writer.close()
            self.assertTrue(self.cancel.wait(1))
        self.assertFalse(terminal.thread.is_alive())

    def test_normal_input_stop_does_not_cancel_success(self):
        with cli._TerminalInput(self.cancel) as terminal:
            self.assertTrue(terminal.thread.is_alive())
        self.assertFalse(terminal.thread.is_alive())
        self.assertFalse(self.cancel.is_set())

    def test_input_and_business_errors_are_both_preserved(self):
        errors = None
        with patch.object(cli.os, "read", side_effect=OSError("terminal read failed")):
            try:
                with cli._TerminalInput(self.cancel) as terminal:
                    self.writer.write("\n")
                    self.writer.flush()
                    self.assertTrue(self.cancel.wait(1))
                    raise ValueError("business failed")
            except ExceptionGroup as error:
                errors = error
        self.assertIsNotNone(errors)
        self.assertIn("terminal read failed", cli._error_message(errors))
        self.assertIn("business failed", cli._error_message(errors))
        self.assertFalse(terminal.thread.is_alive())


if __name__ == "__main__":
    unittest.main()
