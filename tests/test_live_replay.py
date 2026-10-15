from g1_action_studio.constants import JOINT_NAMES
import contextlib
import io
import hashlib
import json
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio import ARM_SLOTS, PROFILE
from g1_action_studio.__main__ import main
from g1_action_studio.live_replay import replay
from g1_action_studio.replay import build_schedule, retime_frames, screen_schedule
from fake_sdk import SimulatedSDK, VirtualClock


class LiveReplayTests(unittest.TestCase):
    def setUp(self):
        try:
            import ompl
            from g1_action_studio.collision import CollisionModel
            CollisionModel()
        except ImportError as error:
            self.skipTest(str(error))
        self.folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.source = self.root / "recording.json"
        self.profile_path = self.root / "profile.json"
        self.output = self.root / "report.json"
        self.safe = json.loads((Path(__file__).parents[1] / "examples/transition/action-b.json").read_text())["samples"][0]["q"]
        self.initial = list(self.safe)
        self.initial[0] += 0.02
        samples = []
        for i in range(51):
            q = list(self.safe)
            q[3] += 0.002 * i
            samples.append({"t": i * 0.02, "q": q, "dq": [0.1] * 28,
                            "mode_pr": 0, "mode_machine": 5})
        self.source.write_text(json.dumps({"schema": "g1-arm7-dex3-state-recording-v1",
            "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS), "joint_names": list(JOINT_NAMES), "samples": samples}), encoding="utf-8")
        candidate = Path(__file__).parents[1] / "examples/profiles/replay.json"
        self.profile_path.write_text(candidate.read_text(encoding="utf-8"), encoding="utf-8")
        from g1_action_studio.constants import DEFAULT_MODEL
        self.model = DEFAULT_MODEL

    def run_replay(self, execute=False):
        return replay("unused", self.source, self.profile_path, self.model,
                      self.output, side="left", speed_scale=0.5, execute=execute)

    def test_preview_and_cli_do_not_import_sdk_or_initialize_network(self):
        with patch.dict(sys.modules, {"unitree_sdk2py": None}), \
                patch("g1_action_studio.live_replay.create_sdk") as factory, \
                contextlib.redirect_stdout(io.StringIO()):
            code = main(["replay", "unused", str(self.source), "--profile", str(self.profile_path),
                         "--model", str(self.model), "--output", str(self.output)])
        self.assertEqual(code, 0)
        factory.assert_not_called()
        report = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertFalse(report["hardware_enabled"])
        self.assertEqual(report["status"], "preview")
        self.assertEqual(report["plan"]["status"], "offline_numerical_pass")

    def test_execute_uses_fresh_baseline_holds_other_arm_and_returns_before_release(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, self.initial)
        with patch("g1_action_studio.live_replay.create_sdk", return_value=sdk), \
                patch("g1_action_studio.live_replay.time", clock):
            result = self.run_replay(execute=True)
        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["release_sent"])
        self.assertFalse(result["handover_confirmed"])
        report = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertEqual(report["plan"]["baseline"]["q"], self.initial)
        self.assertTrue(all(f["q"][7:] == self.initial[7:] for f in report["plan"]["frames"]))
        self.assertTrue(all(r.closed for r in sdk.resources))
        self.assertEqual(sdk.writes[-1].motor_cmd[29].q, 0)
        phases = [s["phase"] for s in report["samples"]]
        self.assertLess(phases.index("settle"), phases.index("release"))

    def test_preview_uses_one_recording_snapshot_even_if_file_changes(self):
        original = self.source.read_bytes()
        read_bytes = Path.read_bytes
        reads = []

        def read_and_change(path):
            raw = read_bytes(path)
            if path == self.source:
                reads.append(raw)
                self.source.write_text("{}", encoding="utf-8")
            return raw

        with patch.object(Path, "read_bytes", read_and_change):
            result = self.run_replay()

        self.assertEqual(result["status"], "preview")
        self.assertEqual(reads, [original])
        report = json.loads(self.output.read_text(encoding="utf-8"))
        samples = json.loads(original)["samples"]
        self.assertEqual(report["plan"]["baseline"]["q"], samples[0]["q"])
        from g1_action_studio.author import load_action
        self.source.write_bytes(original)
        draft = load_action(self.source, None, None, "trial.left_replay", 1, "单臂示教回放试验")
        self.assertEqual(draft["source"]["sha256"], hashlib.sha256(original).hexdigest())
        self.assertEqual(report["plan"]["source_trajectory_sha256"], draft["trajectory_sha256"])

    def test_existing_report_blocks_network_initialization(self):
        self.output.write_text("existing", encoding="utf-8")
        with patch("g1_action_studio.live_replay.create_sdk") as factory:
            with self.assertRaises(FileExistsError):
                self.run_replay(execute=True)
            factory.assert_not_called()
        self.assertEqual(self.output.read_text(encoding="utf-8"), "existing")

    def test_synthetic_example_cannot_initialize_hardware(self):
        data = json.loads(self.source.read_text(encoding="utf-8"))
        data["qualification"] = "synthetic_example_not_for_hardware"
        self.source.write_text(json.dumps(data), encoding="utf-8")
        with patch("g1_action_studio.live_replay.create_sdk") as factory:
            with self.assertRaisesRegex(ValueError, "合成示例"):
                self.run_replay(execute=True)
            factory.assert_not_called()

    def test_live_baseline_outside_model_closes_without_creating_publisher(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [4.0] * 28)
        with patch("g1_action_studio.live_replay.create_sdk", return_value=sdk), \
                patch("g1_action_studio.live_replay.time", clock):
            result = self.run_replay(execute=True)
        self.assertEqual(result["status"], "failed")
        self.assertFalse(sdk.publisher_started)
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_cancel_through_signal_runs_recovery_and_restores_handlers(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, self.initial)
        handlers = {}
        def cancel():
            if clock.now > 7:
                handlers[signal.SIGINT](signal.SIGINT, None)
        sdk.before_step = cancel
        with patch("g1_action_studio.live_replay.create_sdk", return_value=sdk), \
                patch("g1_action_studio.live_replay.time", clock), \
                patch("g1_action_studio.live_replay.signal.getsignal", return_value="previous"), \
                patch("g1_action_studio.live_replay.signal.signal", handlers.__setitem__):
            result = self.run_replay(execute=True)
        self.assertEqual(result["status"], "cancelled")
        report = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertTrue(report["recovery_release_sent"])
        self.assertEqual(handlers[signal.SIGINT], "previous")
        self.assertEqual(handlers[signal.SIGTERM], "previous")
        self.assertTrue(all(r.closed for r in sdk.resources))


class RetimeTests(unittest.TestCase):
    def test_moving_endpoint_has_small_stop_junction_without_changing_path_endpoints(self):
        source = [{"t": i * 0.02, "q": [i * 0.002] * 28} for i in range(51)]
        motion = retime_frames(source, 0.5)
        self.assertEqual(motion[0]["q"], source[0]["q"])
        self.assertEqual(motion[-1]["q"], source[-1]["q"])
        self.assertEqual(motion[-1]["t"], 2.0)
        self.assertTrue(all(a["q"][0] <= b["q"][0] for a, b in zip(motion, motion[1:])))
        original, _ = build_schedule(source, [0.0] * 28)
        prepared, _ = build_schedule(motion, [0.0] * 28)
        before = screen_schedule(original, [(-2, 2)] * 28, 1, 1)
        after = screen_schedule(prepared, [(-2, 2)] * 28, 1, 1)
        self.assertTrue(before["violations"])
        self.assertFalse(after["violations"])

    def test_invalid_scale_is_rejected(self):
        frames = [{"t": i * 0.02, "q": [0.0] * 28} for i in range(3)]
        for scale in (0, -1, 1.1, float("nan"), True):
            with self.subTest(scale=scale), self.assertRaises(ValueError):
                retime_frames(frames, scale)


if __name__ == "__main__":
    unittest.main()
