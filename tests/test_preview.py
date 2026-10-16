import contextlib
import io
import signal
import subprocess
import sys
import unittest
from pathlib import Path
from threading import Event
from unittest.mock import patch

import numpy as np

from g1_action_studio.cli import main
from g1_action_studio.model import GROUPS, ArmModel
from g1_action_studio.motion import load_action, load_start
from g1_action_studio.planner import plan_sequence
from g1_action_studio.preview import run_preview

EXAMPLES = Path(__file__).resolve().parents[1] / "examples"


class PreviewTests(unittest.TestCase):
    def test_help_and_module_entry_do_not_import_optional_or_device_libraries(self):
        code = """
import sys
from importlib.abc import MetaPathFinder
class NoDeviceOrModel(MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('mujoco', 'unitree_sdk2py', 'cyclonedds'):
            raise AssertionError('offline import: ' + fullname)
sys.meta_path.insert(0, NoDeviceOrModel())
from g1_action_studio.cli import main
main(['--help'])
"""
        result = subprocess.run([sys.executable, "-X", "utf8", "-c", code], capture_output=True, encoding="utf-8")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("preview", result.stdout)
        module = subprocess.run(
            [sys.executable, "-X", "utf8", "-m", "g1_action_studio", "--help"],
            capture_output=True,
            encoding="utf-8",
        )
        self.assertEqual(module.returncode, 0, module.stderr)
        self.assertEqual(module.stdout, result.stdout)

    def test_cli_rejects_old_commands_options_and_invalid_timing(self):
        for arguments in (
            ["teach"],
            ["replay"],
            ["build"],
            ["transition"],
            ["preflight"],
            ["audit"],
            ["preview", "action.json", "--start", "start.json", "--side", "left"],
            ["preview", "action.json", "--start", "start.json", "--profile", "old.json"],
            *(
                ["preview", "action.json", "--start", "start.json", option, value]
                for option in ("--speed", "--transition-seconds")
                for value in ("0", "-1", "nan", "inf", "bad")
            ),
        ):
            with self.subTest(arguments=arguments), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(arguments)
                self.assertEqual(error.exception.code, 2)

    def test_cli_file_error_has_path_and_restores_signal_handler(self):
        previous = signal.getsignal(signal.SIGINT)
        errors = io.StringIO()
        with contextlib.redirect_stderr(errors):
            result = main(["preview", "missing-action.json", "--start", "missing-start.json"])
        self.assertEqual(result, 1)
        self.assertIn("missing-action.json", errors.getvalue())
        self.assertIs(signal.getsignal(signal.SIGINT), previous)

    def test_real_cli_draws_same_four_group_and_mixed_trajectories(self):
        import mujoco
        import mujoco.viewer

        start = load_start(EXAMPLES / "start.json")
        sequences = [[group] for group in GROUPS]
        sequences += [["arms_waist", "left_arm"], ["left_arm", "right_arm"]]
        if sys.platform == "linux":
            sequences.append(["detour_left_arm"])
        for groups in sequences:
            with self.subTest(groups=groups):
                actions = [load_action(EXAMPLES / f"{group}.json") for group in groups]
                reference_model = ArmModel(start.leg_q)
                reference = plan_sequence(actions, start, reference_model, 0.5, 0.3, Event())
                times = [0, reference.duration / 2, reference.duration]
                drawn, viewer_data = [], []

                class Viewer:
                    def __init__(self, model, data):
                        viewer_data.append((model, data))

                    def __enter__(self):
                        return self

                    def __exit__(self, *_):
                        pass

                    def is_running(self):
                        return len(drawn) < len(times)

                    def lock(self):
                        return contextlib.nullcontext()

                    def sync(self):
                        drawn.append(viewer_data[0][1].qpos.copy())

                argv = [
                    "preview",
                    *(str(EXAMPLES / f"{group}.json") for group in groups),
                    "--start",
                    str(EXAMPLES / "start.json"),
                    "--speed",
                    "0.5",
                    "--transition-seconds",
                    "0.3",
                ]
                with (
                    patch.object(mujoco.viewer, "launch_passive", side_effect=Viewer),
                    patch("g1_action_studio.preview.time.monotonic", side_effect=[0, *times]),
                    patch.object(Event, "wait", return_value=False),
                ):
                    self.assertEqual(main(argv), 0)
                for qpos, time in zip(drawn, times):
                    target, _ = reference.sample(time)
                    np.testing.assert_allclose(qpos[reference_model.upper_qpos], target, atol=1e-13)
                    np.testing.assert_array_equal(qpos[reference_model.leg_qpos], start.leg_q)
                self.assertIsNot(viewer_data[0][1], reference_model.data)
                self.assertFalse(any(name.startswith(("unitree", "cyclonedds")) for name in sys.modules))

    def test_cancelled_preview_never_opens_viewer(self):
        start = load_start(EXAMPLES / "start.json")
        action = load_action(EXAMPLES / "left_arm.json")
        cancel = Event()
        cancel.set()
        with patch("mujoco.viewer.launch_passive") as launch:
            with self.assertRaises(InterruptedError):
                run_preview([action], start, 1, 2, cancel)
            launch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
