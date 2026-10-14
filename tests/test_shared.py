"""公共实现的行为回归：数值、参数快照与输出不覆盖。"""

import io
import json
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio.constants import DT
from g1_action_studio.config import dump_json, load_replay_profile, write_json
from g1_action_studio.replay import screen_schedule
from g1_action_studio.trajectory import hold_other_arm, joint_motion, metrics


class SharedBehaviorTests(unittest.TestCase):
    def test_joint_differences_and_screening_agree(self):
        frames = [{"t": i * DT, "q": [value] + [0.0] * 13, "phase": "replay"}
                  for i, value in enumerate((0.0, 0.002, 0.006))]
        motion = joint_motion(frames)
        self.assertEqual(motion[0][0], (0.0, 0.002, 0.006))
        self.assertAlmostEqual(motion[0][1][0], 0.1)
        self.assertAlmostEqual(motion[0][1][1], 0.2)
        self.assertAlmostEqual(motion[0][2][0], 5.0)
        result = screen_schedule(frames, [(-1, 1)] * 14, 0.15, 4.0)
        self.assertEqual(result["metrics"], metrics(frames))
        self.assertEqual([(v["kind"], v["t"]) for v in result["violations"]],
                         [("velocity", 0.04), ("acceleration", 0.04)])

    def test_single_arm_planning_preserves_input_and_holds_opposite_side(self):
        frames = [{"t": 0.0, "q": [0.1] * 14}, {"t": DT, "q": [0.2] * 14}]
        original = json.loads(json.dumps(frames))
        baseline = [0.3] * 7 + [0.4] * 7
        left = hold_other_arm(frames, baseline, "left")
        right = hold_other_arm(frames, baseline, "right")
        self.assertEqual(left[1]["q"], [0.2] * 7 + [0.4] * 7)
        self.assertEqual(right[1]["q"], [0.3] * 7 + [0.2] * 7)
        self.assertEqual(frames, original)

    def test_replay_profile_uses_one_snapshot_for_gain_and_executor_checks(self):
        path = Path(__file__).parents[1] / "examples/profiles/replay.json"
        text = path.read_text(encoding="utf-8")
        with patch.object(Path, "read_text", side_effect=[text, "{}"]) as reader:
            result = load_replay_profile(path)
        reader.assert_called_once()
        self.assertEqual(result, json.loads(text))

    def test_replay_profile_checks_both_gain_limits_and_feedback_thresholds(self):
        path = Path(__file__).parents[1] / "examples/profiles/replay.json"
        profile = json.loads(path.read_text(encoding="utf-8"))
        for key, value, error in (("hold_kp", 61, "保持增益"),
                                  ("state_timeout_s", 0, "state_timeout_s")):
            with self.subTest(key=key), patch.object(Path, "read_text", return_value=json.dumps({**profile, key: value})):
                with self.assertRaisesRegex(ValueError, error):
                    load_replay_profile(path)

    def test_json_writer_keeps_chinese_and_refuses_existing_output(self):
        stream = io.StringIO()
        dump_json({"名称": "示教"}, stream, compact=True)
        self.assertEqual(stream.getvalue(), '{"名称":"示教"}\n')
        with patch.object(Path, "open", side_effect=FileExistsError) as opening:
            with self.assertRaises(FileExistsError):
                write_json({}, "existing.json")
        opening.assert_called_once_with("x", encoding="utf-8")
