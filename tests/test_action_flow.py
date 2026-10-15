"""实际使用流程：录制直接回放、制作后回放、衔接后回放。"""

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio.author import load_action
from g1_action_studio.cli import main
from g1_action_studio.config import write_json
from g1_action_studio.constants import ACTION_SCHEMA, ARM_SLOTS, DEFAULT_MODEL, PROFILE
from g1_action_studio.live_replay import replay
from g1_action_studio.transition import plan_transition
from fake_sdk import SimulatedSDK, VirtualClock


ROOT = Path(__file__).parents[1]


class ActionFixtures:
    def setUp(self):
        try:
            import ompl
            from g1_action_studio.collision import CollisionModel
            CollisionModel()
        except ImportError as error:
            self.skipTest(str(error))
        folder = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(folder.cleanup)
        self.root = Path(folder.name)
        self.profile = ROOT / "examples/profiles/replay.json"
        self.source = self.root / "recording.json"
        self.output = self.root / "report.json"
        self.q = json.loads((ROOT / "examples/transition/action-b.json").read_text())["samples"][0]["q"]
        self.record(self.source, self.q)

    def record(self, path, q, seconds=1.0, synthetic=False):
        data = {"schema": "g1-arm7-state-recording-v1", "target_profile": PROFILE,
                "arm_slots": list(ARM_SLOTS), "invalid_messages": 0,
                "samples": [{"t": round(i * 0.02, 6), "q": list(q), "dq": [0.0] * 14,
                             "mode_pr": 0, "mode_machine": 5} for i in range(round(seconds / 0.02) + 1)]}
        if synthetic:
            data["qualification"] = "synthetic_example_not_for_hardware"
        write_json(data, path)

    def execute(self, source, q=None, **kwargs):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, self.q if q is None else q)
        with patch("g1_action_studio.live_replay._make_sdk", return_value=sdk), \
                patch("g1_action_studio.live_replay.time", clock), \
                patch("g1_action_studio.teach.time", clock), \
                contextlib.redirect_stdout(io.StringIO()):
            result = replay("unused", source, self.profile, DEFAULT_MODEL, self.output,
                            speed_scale=1.0, execute=True, **kwargs)
        return result, sdk, json.loads(self.output.read_text(encoding="utf-8"))


class ActionFlowTests(ActionFixtures, unittest.TestCase):
    def test_recording_and_built_action_share_the_executor(self):
        result, sdk, recording_report = self.execute(self.source)
        self.assertEqual(result["status"], "completed")
        self.output = self.root / "built-report.json"
        action = self.root / "action.json"
        write_json(load_action(self.source), action)
        with patch("g1_action_studio.author._interpolate", side_effect=AssertionError("已有动作不应重新制作")):
            result, sdk, built_report = self.execute(action)
        self.assertEqual(result["status"], "completed")
        self.assertEqual(recording_report["plan"]["frames"], built_report["plan"]["frames"])
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_build_does_not_require_manual_audit_or_all_metadata(self):
        action = self.root / "built.json"
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(["build", str(self.source), "--output", str(action)]), 0)
        self.assertEqual(load_action(action)["schema"], ACTION_SCHEMA)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as exit:
            main(["audit", str(action)])
        self.assertEqual(exit.exception.code, 2)

    def test_old_recording_and_old_action_formats_are_rejected(self):
        for schema in ("g1-preset-arm-readonly-trace-v1", "g1-arm7-action-draft-v1", "g1-arm7-collision-transition-v1"):
            path = self.root / (schema + ".json")
            write_json({"schema": schema}, path)
            with self.subTest(schema=schema), self.assertRaisesRegex(ValueError, "格式"):
                load_action(path)

    def test_synthetic_marker_survives_build_and_blocks_execution(self):
        source = self.root / "synthetic.json"
        self.record(source, self.q, synthetic=True)
        action = self.root / "synthetic-action.json"
        write_json(load_action(source), action)
        with patch("g1_action_studio.live_replay._make_sdk") as factory:
            with self.assertRaisesRegex(ValueError, "合成示例"):
                replay("unused", action, self.profile, DEFAULT_MODEL, self.output, execute=True)
            factory.assert_not_called()


class CombinedFlowTests(ActionFixtures, unittest.TestCase):
    def setUp(self):
        super().setUp()
        try:
            import ompl
            from g1_action_studio.collision import CollisionModel
            model = CollisionModel()
        except ImportError as error:
            self.skipTest(str(error))
        self.q = json.loads((ROOT / "examples/transition/action-b.json").read_text())["samples"][0]["q"]
        self.assertTrue(model.clearance(self.q)["valid"])
        self.before = self.root / "a.json"
        self.after = self.root / "b.json"
        self.record(self.before, self.q, seconds=0.1)
        self.record(self.after, self.q, seconds=0.1)

    def combined(self, side="left"):
        path = self.root / "combined.json"
        plan_transition(self.before, self.after, path, side=side)
        return path

    def test_same_endpoints_add_no_transition_duration_and_execute_once(self):
        action = self.combined()
        data = load_action(action)
        self.assertEqual(data["kind"], "sequence")
        self.assertAlmostEqual(data["metrics"]["duration_s"], 0.2)
        self.assertEqual(data["transition_metrics"]["duration_s"], 0.0)
        result, sdk, report = self.execute(action, q=self.q)
        self.assertEqual(result["status"], "completed", report["errors"])
        self.assertEqual([p["phase"] for p in report["plan"]["phases"]],
                         ["takeover", "enter", "replay", "return", "release"])
        self.assertEqual(report["plan"]["body_pose"], [0.0] * 15)
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_multiple_sources_are_planned_and_execute_once_without_transition(self):
        result, sdk, report = self.execute([self.before, self.after, self.before], q=self.q)
        self.assertEqual(result["status"], "completed", report["errors"])
        self.assertEqual(len(report["plan"]["actions"]), 3)
        self.assertEqual(len(report["plan"]["phases"]), 5)
        self.assertEqual(report["source_files"], ["a.json", "b.json", "a.json"])
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_wrong_side_cannot_initialize_network(self):
        action = self.combined()
        with patch("g1_action_studio.live_replay._make_sdk") as factory:
            with self.assertRaisesRegex(ValueError, "回放侧"):
                replay("unused", action, self.profile, DEFAULT_MODEL, self.output, side="right", execute=True)
            factory.assert_not_called()

    def test_nonzero_bridge_and_return_route_execute_as_one_action(self):
        from g1_action_studio.collision import CollisionModel
        start = json.loads((ROOT / "examples/transition/action-a.json").read_text())["samples"][0]["q"]
        self.before = self.root / "routed-start.json"
        self.record(self.before, start, seconds=0.1)
        model = CollisionModel()
        self.assertFalse(model.edge_clear(start, self.q, 0.02))
        action = self.root / "routed.json"
        plan_transition(self.before, self.after, action)
        self.assertGreater(load_action(action)["transition_metrics"]["duration_s"], 0)
        result, sdk, report = self.execute([self.before, self.after], q=start)
        self.assertEqual(result["status"], "completed", report["errors"])
        returning = [frame for frame in report["plan"]["frames"] if frame["phase"] == "return"]
        self.assertEqual(returning[-1]["q"], start)
        self.assertTrue(model.edge_clear(self.q, returning[0]["q"], 0.02))
        self.assertGreater(report["plan"]["collision"]["minimum_clearance_m"], 0.02)
        self.assertEqual(len(report["plan"]["phases"]), 5)
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_actual_start_pose_is_checked_for_collision_before_publishing(self):
        action = self.combined()
        q = list(self.q)
        q[0] = -0.5
        # 用碰撞反馈明确拒绝实际基线，而不是信任文件中保存的姿态。
        with patch("g1_action_studio.collision.CollisionModel.check_motion", side_effect=ValueError("当前姿态碰撞")):
            result, sdk, report = self.execute(action, q=q)
        self.assertEqual(result["status"], "failed")
        self.assertFalse(sdk.publisher_started)
        self.assertFalse(sdk.writes)
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_body_pose_change_stops_combined_playback(self):
        action = self.combined()
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, self.q)
        def move_body():
            if sdk.publisher_started and clock.now > 4.0:
                sdk.body_q[0] = 0.03
        sdk.before_step = move_body
        with patch("g1_action_studio.live_replay._make_sdk", return_value=sdk), \
                patch("g1_action_studio.live_replay.time", clock), \
                patch("g1_action_studio.teach.time", clock), \
                contextlib.redirect_stdout(io.StringIO()):
            result = replay("unused", action, self.profile, DEFAULT_MODEL, self.output, execute=True)
        report = json.loads(self.output.read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "failed")
        self.assertIn("腿/腰姿态", report["errors"][0]["message"])
        self.assertFalse(report["recovery_release_sent"])
        self.assertTrue(all(r.closed for r in sdk.resources))
