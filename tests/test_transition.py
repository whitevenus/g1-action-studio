import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio.collision import CollisionModel, find_path, parameterize_path
from g1_action_studio.constants import DT
from g1_action_studio.trajectory import metrics
from g1_action_studio.transition import plan_transition

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples/transition"


class TransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import ompl  # noqa: F401 -- 实际导入以确认规划依赖可用。

            cls.model = CollisionModel()
        except ImportError as error:
            raise unittest.SkipTest(str(error))
        cls.a = json.loads((EXAMPLES / "action-a.json").read_text())["samples"][-1]["q"]
        cls.b = json.loads((EXAMPLES / "action-b.json").read_text())["samples"][0]["q"]

    def test_endpoints_clear_but_direct_path_penetrates_body(self):
        self.assertTrue(self.model.clearance(self.a)["valid"])
        self.assertTrue(self.model.clearance(self.b)["valid"])
        middle = [a + 0.3 * (b - a) for a, b in zip(self.a, self.b)]
        collision = self.model.clearance(middle)
        self.assertFalse(collision["valid"])
        self.assertLess(collision["distance_m"], 0)
        self.assertIn("left_hip_pitch_link_collision", collision["pair"])
        self.assertFalse(self.model.edge_clear(self.a, self.b, 0.02))

    def test_interval_check_finds_collision_away_from_midpoint(self):
        midpoint = [(a + b) / 2 for a, b in zip(self.a, self.b)]
        self.assertTrue(self.model.clearance(midpoint, 0.01)["valid"])
        self.assertFalse(self.model.edge_clear(self.a, self.b, 0.01))

    def test_right_arm_can_detour_with_left_arm_fixed(self):
        def mirror(q):
            signs = [1, -1, -1, 1, -1, 1, -1]
            return [v * s for v, s in zip(q[7:14], signs)] + [v * s for v, s in zip(q[:7], signs)] + [0.0] * 14

        a, b = mirror(self.a), mirror(self.b)
        path = find_path(self.model, a, b, "right")
        self.assertGreater(len(path), 2)
        for point in path:
            self.assertEqual(point[:7], a[:7])
        for start, end in zip(path, path[1:]):
            self.assertTrue(self.model.edge_clear(start, end, 0.02))

    def test_search_detours_and_keeps_other_arm_fixed(self):
        path = find_path(self.model, self.a, self.b, "left")
        self.assertGreater(len(path), 2)
        self.assertEqual(path[0], self.a)
        self.assertEqual(path[-1], self.b)
        for a, b in zip(path, path[1:]):
            self.assertEqual(a[7:14] + a[21:], self.a[7:14] + self.a[21:])
            self.assertTrue(self.model.edge_clear(a, b, 0.02))
        frames = parameterize_path(path, 0.5, 2)
        values = metrics(frames)
        self.assertLessEqual(values["max_abs_velocity_rad_s"], 0.5)
        self.assertLessEqual(values["max_abs_acceleration_rad_s2"], 2)
        # 端点解析速度为零；离散首末步速度按加速度上限与采样周期约束。
        self.assertLessEqual(values["first_step_max_velocity_rad_s"], 2 * DT / 2)
        self.assertLessEqual(values["last_step_max_velocity_rad_s"], 2 * DT / 2)

    def test_clear_direct_path_needs_no_search(self):
        end = list(self.b)
        end[0] -= 0.03
        self.assertEqual(find_path(self.model, self.b, end, "left", timeout=0), [self.b, end])

    def test_initial_baseline_tuple_and_finger_fk_are_supported(self):
        self.assertTrue(self.model.clearance(tuple(self.b))["valid"])
        geom = self.model.model.geom("left_hand_thumb_2_link_collision").id
        self.model._position(self.b)
        before = self.model.data.geom_xpos[geom].copy()
        q = list(self.b)
        q[15] += 0.1
        self.model._position(q)
        self.assertGreater(float(self.model.np.linalg.norm(self.model.data.geom_xpos[geom] - before)), 0.001)
        motion = parameterize_path(find_path(self.model, self.b, q, "left"), 0.5, 2.0)
        self.model.check_motion(motion, 0.02)
        self.assertEqual(motion[-1]["q"], q)

    def test_approximate_solution_is_not_accepted_as_a_complete_path(self):
        from ompl import geometric as og

        with (
            patch.object(og.SimpleSetup, "solve", return_value=True),
            patch.object(og.SimpleSetup, "haveExactSolutionPath", return_value=False),
        ):
            with self.assertRaisesRegex(ValueError, "完整避碰路径"):
                find_path(self.model, self.a, self.b, "left")

    def test_missing_ompl_is_reported_without_a_handwritten_fallback(self):
        with patch.dict("sys.modules", {"ompl": None}):
            with self.assertRaisesRegex(ImportError, "OMPL 2.0.1"):
                find_path(self.model, self.b, self.b, "left")

    def test_invalid_endpoints_and_exhausted_search_are_rejected(self):
        bad = list(self.a)
        bad[0] = float("nan")
        with self.assertRaises(ValueError):
            find_path(self.model, bad, self.b, "left")
        bad = list(self.a)
        bad[0] = 10
        with self.assertRaises(ValueError):
            find_path(self.model, bad, self.b, "left")
        with self.assertRaisesRegex(ValueError, "预算"):
            find_path(self.model, self.a, self.b, "left", timeout=0)
        with self.assertRaises(ValueError):
            parameterize_path([self.a, self.b], float("nan"), 2)
        with self.assertRaises(ValueError):
            CollisionModel([0] * 14)

    def test_official_fingers_and_both_arms_are_included_without_envelopes(self):
        m = self.model.model
        names = [m.geom(i).name for i in range(m.ngeom)]
        self.assertEqual(sum(n.endswith("_sweep") for n in names), 0)
        self.assertEqual(len(self.model.joints), 28)
        self.assertIn("head_link_collision", names)
        self.assertIn("left_hand_palm_link_collision", names)
        self.assertIn("right_hand_palm_link_collision", names)
        cross_arm = []
        for (a, b), mount in zip(self.model.pairs, self.model.mount_pairs):
            na, nb = m.geom(int(a)).name, m.geom(int(b)).name
            if na.startswith("left_") and nb.startswith("right_"):
                cross_arm.append((na, nb))
                self.assertFalse(mount)
        self.assertIn(("left_hand_thumb_2_link_collision", "right_hand_thumb_2_link_collision"), cross_arm)
        self.assertFalse(any(m.geom_group[a] == 1 or m.geom_group[b] == 1 for a, b in self.model.pairs))

    def test_complete_transition_outputs_uniform_loadable_action(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            output = Path(directory) / "plan.json"
            with patch.dict("sys.modules", {"unitree_sdk2py": None}):
                result = plan_transition(EXAMPLES / "action-a.json", EXAMPLES / "action-b.json", output)
            self.assertEqual(result["kind"], "sequence")
            self.assertEqual(result["status"], "offline_collision_pass")
            self.assertGreaterEqual(result["minimum_clearance_m"], 0.02)
            plan = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual({f["phase"] for f in plan["frames"]}, {"action_1", "transition", "action_2"})
            self.assertEqual(plan["frames"][0]["q"][7:], plan["frames"][-1]["q"][7:])
            from g1_action_studio.author import load_action

            self.assertEqual(load_action(output), plan)
            with self.assertRaises(FileExistsError):
                plan_transition(EXAMPLES / "action-a.json", EXAMPLES / "action-b.json", output)

    def test_collision_inside_action_is_rejected_without_output(self):
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            source = Path(directory) / "bad.json"
            output = Path(directory) / "plan.json"
            data = json.loads((EXAMPLES / "action-a.json").read_text())
            collision = [a + 0.3 * (b - a) for a, b in zip(self.a, self.b)]
            for row in data["samples"][10:40]:
                row["q"] = collision
            source.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "完整动作"):
                plan_transition(source, EXAMPLES / "action-b.json", output, max_velocity=1000, max_acceleration=1e6)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()
