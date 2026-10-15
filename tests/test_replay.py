import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio.author import load_action, validate_action
from g1_action_studio.cli import main
from g1_action_studio.config import read_json, write_json
from g1_action_studio.constants import ARM_SLOTS, COLLISION_MODEL, JOINT_NAMES, PROFILE
from g1_action_studio.replay import (
    build_schedule,
    create_plan,
    load_limits,
    sample_schedule,
    screen_schedule,
)
from g1_action_studio.trajectory import frame_digest

TEST_ROOT = Path(__file__).resolve().parent


def fixture(folder):
    frames = []
    for index in range(51):
        q = [0.0] * 28
        q[3] = 0.2 + index * 0.002
        frames.append({"t": round(index * 0.02, 6), "q": q})
    action = {
        "schema": "g1-arm7-dex3-action-v1",
        "kind": "single",
        "synthetic": False,
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "action_code": "classroom.test",
        "action_version": 1,
        "display_name": "测试动作",
        "frames": frames,
        "trajectory_sha256": frame_digest(frames),
    }
    baseline = {
        "schema": "g1-arm7-replay-baseline-v1",
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "provenance": "合成测试姿态，不用于真机",
        "q": [0.0] * 28,
    }
    action_path, baseline_path = Path(folder) / "action.json", Path(folder) / "baseline.json"
    write_json(action, action_path)
    write_json(baseline, baseline_path)
    return action_path, baseline_path, frames


class ReplayTests(unittest.TestCase):
    def test_cli_rejects_separate_joint_limit_model(self):
        with patch("sys.stderr", io.StringIO()), self.assertRaises(SystemExit) as error:
            main(
                [
                    "replay",
                    "unused",
                    "unused-action.json",
                    "--profile",
                    "unused-profile.json",
                    "--output",
                    "unused-report.json",
                    "--model",
                    "other.xml",
                ]
            )
        self.assertEqual(error.exception.code, 2)

    def test_complete_schedule_returns_to_explicit_baseline_and_releases(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            action, baseline, source = fixture(folder)
            with patch.dict(sys.modules, {"unitree_sdk2py": None}):
                result = create_plan(read_json(action), read_json(baseline), 1.0, 20.0)
            self.assertEqual(result["status"], "offline_numerical_pass")
            self.assertFalse(result["hardware_enabled"])
            self.assertEqual(result["model"]["file_name"], COLLISION_MODEL.name)
            self.assertEqual(result["model"]["limits_rad"], [list(pair) for pair in load_limits(COLLISION_MODEL)])
            frames = result["frames"]
            self.assertEqual(len(frames), 801)
            self.assertEqual(frames[-1]["t"], 16.0)
            self.assertEqual(frames[0]["weight"], 0)
            self.assertEqual(frames[-1]["weight"], 0)
            self.assertEqual(frames[-1]["q"], [0.0] * 28)
            self.assertEqual(
                [p["phase"] for p in result["phases"]], ["takeover", "enter", "replay", "return", "release"]
            )
            self.assertEqual(frames[400]["q"], source[0]["q"])
            self.assertEqual(frames[450]["q"], source[-1]["q"])
            self.assertTrue(all(0 <= frame["weight"] <= 1 for frame in frames))

    def test_samples_use_elapsed_time_and_hold_endpoints(self):
        frames = [
            {"t": 0.0, "q": [0.0] * 28, "phase": "takeover", "weight": 0},
            {"t": 0.02, "q": [0.2] * 28, "phase": "enter", "weight": 1},
        ]
        sampled = sample_schedule(frames, 0.005)
        self.assertAlmostEqual(sampled["q"][0], 0.05)
        self.assertAlmostEqual(sampled["weight"], 0.25)
        self.assertEqual(sample_schedule(frames, -1)["q"], frames[0]["q"])
        self.assertEqual(sample_schedule(frames, 1)["q"], frames[-1]["q"])
        with self.assertRaises(ValueError):
            sample_schedule(frames, float("nan"))

    def test_checks_phase_boundary_acceleration(self):
        # 轨迹内部匀速，但 enter 静止到 replay 的边界存在加速度尖峰。
        frames = [{"t": round(i * 0.02, 6), "q": [i * 0.01] * 28} for i in range(3)]
        schedule, _ = build_schedule(frames, [0.0] * 28)
        result = screen_schedule(schedule, [(-2, 2)] * 28, 1, 10)
        self.assertTrue(any(v["kind"] == "acceleration" and v["phase"] == "replay" for v in result["violations"]))

    def test_short_enter_transition_is_screened(self):
        frames = [{"t": i * 0.02, "q": [0.2] * 28} for i in range(3)]
        schedule, _ = build_schedule(frames, [0.0] * 28, enter=0.02)
        result = screen_schedule(schedule, [(-2, 2)] * 28, 1, 20)
        self.assertTrue(any(v["kind"] == "velocity" and v["phase"] == "enter" for v in result["violations"]))

    def test_position_limits_include_baseline(self):
        frames = [{"t": i * 0.02, "q": [0.0] * 28} for i in range(3)]
        schedule, _ = build_schedule(frames, [3.0] * 28)
        result = screen_schedule(schedule, [(-2, 2)] * 28, 100, 1000)
        self.assertTrue(any(v["kind"] == "position" and v["t"] == 0 for v in result["violations"]))

    def test_rejects_tampered_source(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            action, baseline, _ = fixture(folder)
            data = json.loads(action.read_text(encoding="utf-8"))
            data["frames"][0]["q"][0] = 1
            action.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "摘要"):
                create_plan(load_action(action), read_json(baseline), 1, 20)

    def test_rejects_malformed_action_without_a_traceback(self):
        for data in ([], None, {"frames": None}):
            with self.assertRaises(ValueError):
                validate_action(data)

    def test_invalid_durations_and_thresholds(self):
        frames = [{"t": i * 0.02, "q": [0.0] * 28} for i in range(3)]
        for duration in (0, -1, 0.03, float("nan"), float("inf"), 121):
            with self.assertRaises(ValueError):
                build_schedule(frames, [0.0] * 28, enter=duration)
        schedule, _ = build_schedule(frames, [0.0] * 28)
        for threshold in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                screen_schedule(schedule, [(-2, 2)] * 28, threshold, 10)

    def test_model_requires_all_joint_limits_and_radians(self):
        self.assertEqual(len(load_limits(COLLISION_MODEL)), 28)
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            model = Path(folder) / "model.xml"
            for content in (
                '<mujoco><compiler angle="degree"/></mujoco>',
                '<mujoco><compiler angle="radian"/></mujoco>',
                "<invalid",
            ):
                model.write_text(content, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_limits(model)

    def test_rejected_plan_can_be_saved_without_overwriting(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            action, baseline, _ = fixture(folder)
            output = Path(folder) / "plan.json"
            data = create_plan(read_json(action), read_json(baseline), 0.001, 0.001)
            self.assertEqual(data["status"], "offline_numerical_rejected")
            self.assertTrue(data["violations"])
            write_json(data, output)
            original = output.read_bytes()
            with self.assertRaises(FileExistsError):
                write_json(data, output)
            self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
