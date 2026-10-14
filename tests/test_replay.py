import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio import ARM_SLOTS, PROFILE
from g1_action_studio.trajectory import frame_digest
from g1_action_studio.config import read_json, write_json
from g1_action_studio.author import validate_draft
from g1_action_studio.replay import (
    DEFAULT_MODEL, build_schedule, load_limits, create_plan,
    sample_schedule, screen_schedule,
)


TEST_ROOT = Path(__file__).resolve().parent


def fixture(folder):
    frames = []
    for index in range(51):
        q = [0.0] * 14
        q[3] = 0.2 + index * 0.002
        frames.append({"t": round(index * 0.02, 6), "q": q})
    draft = {"schema": "g1-arm7-action-draft-v1", "qualification": "draft_unqualified_not_executable",
             "hardware_enabled": False, "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS),
             "action_code": "classroom.test", "action_version": 1,
             "frames": frames, "trajectory_sha256": frame_digest(frames)}
    baseline = {"schema": "g1-arm7-replay-baseline-v1", "target_profile": PROFILE,
                "arm_slots": list(ARM_SLOTS), "provenance": "合成测试姿态，不用于真机", "q": [0.0] * 14}
    draft_path, baseline_path = Path(folder) / "draft.json", Path(folder) / "baseline.json"
    write_json(draft, draft_path)
    write_json(baseline, baseline_path)
    return draft_path, baseline_path, frames


class ReplayTests(unittest.TestCase):
    def test_complete_schedule_returns_to_explicit_baseline_and_releases(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            draft, baseline, source = fixture(folder)
            with patch.dict(sys.modules, {"unitree_sdk2py": None}):
                result = create_plan(read_json(draft), read_json(baseline), DEFAULT_MODEL, 1.0, 20.0)
            self.assertEqual(result["status"], "offline_numerical_pass")
            self.assertFalse(result["hardware_enabled"])
            frames = result["frames"]
            self.assertEqual(len(frames), 801)
            self.assertEqual(frames[-1]["t"], 16.0)
            self.assertEqual(frames[0]["weight"], 0)
            self.assertEqual(frames[-1]["weight"], 0)
            self.assertEqual(frames[-1]["q"], [0.0] * 14)
            self.assertEqual([p["phase"] for p in result["phases"]],
                             ["takeover", "enter", "replay", "return", "release"])
            self.assertEqual(frames[400]["q"], source[0]["q"])
            self.assertEqual(frames[450]["q"], source[-1]["q"])
            self.assertTrue(all(0 <= frame["weight"] <= 1 for frame in frames))

    def test_samples_use_elapsed_time_and_hold_endpoints(self):
        frames = [{"t": 0.0, "q": [0.0] * 14, "phase": "takeover", "weight": 0},
                  {"t": 0.02, "q": [0.2] * 14, "phase": "enter", "weight": 1}]
        sampled = sample_schedule(frames, 0.005)
        self.assertAlmostEqual(sampled["q"][0], 0.05)
        self.assertAlmostEqual(sampled["weight"], 0.25)
        self.assertEqual(sample_schedule(frames, -1)["q"], frames[0]["q"])
        self.assertEqual(sample_schedule(frames, 1)["q"], frames[-1]["q"])
        with self.assertRaises(ValueError):
            sample_schedule(frames, float("nan"))

    def test_checks_phase_boundary_acceleration(self):
        # 轨迹内部匀速，但 enter 静止到 replay 的边界存在加速度尖峰。
        frames = [{"t": round(i * 0.02, 6), "q": [i * 0.01] * 14} for i in range(3)]
        schedule, _ = build_schedule(frames, [0.0] * 14)
        result = screen_schedule(schedule, [(-2, 2)] * 14, 1, 10)
        self.assertTrue(any(v["kind"] == "acceleration" and v["phase"] == "replay"
                            for v in result["violations"]))

    def test_short_enter_transition_is_screened(self):
        frames = [{"t": i * 0.02, "q": [0.2] * 14} for i in range(3)]
        schedule, _ = build_schedule(frames, [0.0] * 14, enter=0.02)
        result = screen_schedule(schedule, [(-2, 2)] * 14, 1, 20)
        self.assertTrue(any(v["kind"] == "velocity" and v["phase"] == "enter"
                            for v in result["violations"]))

    def test_position_limits_include_baseline(self):
        frames = [{"t": i * 0.02, "q": [0.0] * 14} for i in range(3)]
        schedule, _ = build_schedule(frames, [3.0] * 14)
        result = screen_schedule(schedule, [(-2, 2)] * 14, 100, 1000)
        self.assertTrue(any(v["kind"] == "position" and v["t"] == 0
                            for v in result["violations"]))

    def test_rejects_tampered_source(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            draft, baseline, _ = fixture(folder)
            data = json.loads(draft.read_text(encoding="utf-8"))
            data["frames"][0]["q"][0] = 1
            draft.write_text(json.dumps(data), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "摘要"):
                create_plan(read_json(draft), read_json(baseline), DEFAULT_MODEL, 1, 20)

    def test_rejects_malformed_draft_without_a_traceback(self):
        for data in ([], None, {"frames": None}):
            with self.assertRaises(ValueError):
                validate_draft(data)


    def test_invalid_durations_and_thresholds(self):
        frames = [{"t": i * 0.02, "q": [0.0] * 14} for i in range(3)]
        for duration in (0, -1, 0.03, float("nan"), float("inf"), 121):
            with self.assertRaises(ValueError):
                build_schedule(frames, [0.0] * 14, enter=duration)
        schedule, _ = build_schedule(frames, [0.0] * 14)
        for threshold in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                screen_schedule(schedule, [(-2, 2)] * 14, threshold, 10)

    def test_model_requires_all_joint_limits_and_radians(self):
        self.assertEqual(len(load_limits(DEFAULT_MODEL)), 14)
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            model = Path(folder) / "model.xml"
            for content in ('<mujoco><compiler angle="degree"/></mujoco>',
                            '<mujoco><compiler angle="radian"/></mujoco>', '<invalid'):
                model.write_text(content, encoding="utf-8")
                with self.assertRaises(ValueError):
                    load_limits(model)

    def test_rejected_plan_can_be_saved_without_overwriting(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            draft, baseline, _ = fixture(folder)
            output = Path(folder) / "plan.json"
            data = create_plan(read_json(draft), read_json(baseline), DEFAULT_MODEL, 0.001, 0.001)
            self.assertEqual(data["status"], "offline_numerical_rejected")
            self.assertTrue(data["violations"])
            write_json(data, output)
            original = output.read_bytes()
            with self.assertRaises(FileExistsError):
                write_json(data, output)
            self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
