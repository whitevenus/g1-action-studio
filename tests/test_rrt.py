import sys
import time
import unittest
from threading import Event
from unittest.mock import patch

import numpy as np

from g1_action_studio import planner
from g1_action_studio.model import GROUPS, ArmModel
from g1_action_studio.planner import PlanningError, check_trajectory, connect

HOME = np.array([0, 0.2, 0, 0.8, 0, 0, 0] + [0, -0.2, 0, 0.8, 0, 0, 0] + [0, 0, 0], dtype=np.float64)
CROSSING_LEFT = np.array([0.561, 0.238, -1.245, 0.563, 1.853, 0.501, -1.141])


@unittest.skipUnless(sys.platform == "linux", "OMPL 规划的交付平台为 Linux")
class RrtTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Linux 缺依赖必须失败，不能以 skip 代替真实规划验收。
        from ompl import base, geometric

        cls.ompl_base = base
        cls.ompl_geometric = geometric

    def crossing(self):
        model = ArmModel(np.zeros(12))
        end = HOME.copy()
        end[:7] = CROSSING_LEFT
        self.assertTrue(model.valid(HOME), model.last_error)
        self.assertTrue(model.valid(end), model.last_error)
        self.assertFalse(model.edge_free(HOME, end, Event()))
        return model, end

    def disconnected_box(self):
        # 真实模型的肩 yaw 两侧可达，中间碰撞；窄搜索盒阻止其他关节绕开。
        model = ArmModel(np.zeros(12))
        ranges = model.ranges.copy()
        for joint in set(GROUPS["left_arm"]) - {2}:
            ranges[joint] = HOME[joint] + np.array([-1e-6, 1e-6])
        model.ranges = ranges
        start, end = HOME.copy(), HOME.copy()
        start[2] = -2.4
        self.assertTrue(model.valid(start), model.last_error)
        self.assertTrue(model.valid(end), model.last_error)
        middle = HOME.copy()
        middle[2] = -1.6
        self.assertFalse(model.valid(middle))
        self.assertIn("碰撞", model.last_error)
        return model, start, end

    def test_real_rrt_uses_all_selected_dimensions_and_preserves_the_polyline(self):
        search = planner._rrt_path
        for groups in ({"left_arm"}, {"left_arm", "right_arm"}, {"arms_waist", "left_arm"}):
            with self.subTest(groups=groups):
                model, end = self.crossing()
                if "right_arm" in groups or "arms_waist" in groups:
                    end[7:14] = [-1, -0.6, 0.2, 0.5, 0, 0, 0]
                if "arms_waist" in groups:
                    end[14:] = [0.3, 0.1, -0.1]
                self.assertTrue(model.valid(end), model.last_error)
                self.assertFalse(model.edge_free(HOME, end, Event()))
                active = sorted({joint for group in groups for joint in GROUPS[group]})
                paths = []

                def record(*arguments):
                    self.assertEqual(arguments[3], active)
                    points = search(*arguments)
                    paths.append(points)
                    return points

                with patch.object(planner, "_rrt_path", new=record):
                    trajectory = connect(model, HOME, end, groups, 1.7, Event())
                self.assertEqual(len(paths), 1)
                self.assertGreater(len(paths[0]), 2)
                self.assertEqual(trajectory.duration, 1.7)
                np.testing.assert_array_equal(trajectory.first_q, HOME)
                np.testing.assert_array_equal(trajectory.last_q, end)
                check_trajectory(model, trajectory, Event())
                points = np.asarray(paths[0])
                if "right_arm" in groups or "arms_waist" in groups:
                    np.testing.assert_array_equal(end[11:14], HOME[11:14])
                    self.assertTrue(np.any(points[1:-1, 11:14] != HOME[11:14]))
                lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
                widths = np.diff(trajectory.polynomial.x)
                np.testing.assert_allclose(widths, 1.7 * lengths / lengths.sum(), atol=1e-13)
                for index, width in enumerate(widths):
                    left = trajectory.polynomial.x[index]
                    for fraction in (0.2, 0.6):
                        q, dq = trajectory.sample(float(left + fraction * width))
                        delta = points[index + 1] - points[index]
                        h = 10 * fraction**3 - 15 * fraction**4 + 6 * fraction**5
                        np.testing.assert_allclose(q, points[index] + h * delta, atol=1e-12)
                        np.testing.assert_allclose(
                            dq, 30 * fraction**2 * (1 - fraction) ** 2 * delta / width, atol=1e-11
                        )
                inactive = sorted(set(range(17)) - set(active))
                for moment in np.linspace(0, trajectory.duration, 101):
                    q, dq = trajectory.sample(float(moment))
                    np.testing.assert_array_equal(q[inactive], HOME[inactive])
                    np.testing.assert_array_equal(dq[inactive], np.zeros(len(inactive)))
                for moment in trajectory.polynomial.x:
                    _, dq = trajectory.sample(float(moment))
                    np.testing.assert_allclose(dq, np.zeros(17), atol=1e-12)

    def test_disconnected_real_geometry_rejects_native_approximate_solution(self):
        model, start, end = self.disconnected_box()
        self.assertEqual(planner.SEARCH_SECONDS, 5.0)
        self.assertEqual(planner.SHORTCUT_SECONDS, 1.0)
        started = time.monotonic()
        with patch.object(planner, "connection_curve", side_effect=AssertionError("无解不能构造直线兜底")):
            with self.assertRaisesRegex(PlanningError, "近似"):
                connect(model, start, end, {"left_arm"}, 2.0, Event())
        self.assertLess(time.monotonic() - started, 8.0)

    def test_real_search_with_zero_budget_rejects_missing_solution(self):
        model, start, end = self.disconnected_box()
        with patch.object(planner, "SEARCH_SECONDS", 0.0):
            with self.assertRaisesRegex(PlanningError, "未找到精确解"):
                planner._rrt_path(model, start, end, list(GROUPS["left_arm"]), Event())

    def test_cancel_during_real_search_returns_without_crossing_cpp_with_an_exception(self):
        model, start, end = self.disconnected_box()
        cancel = Event()
        valid = model.valid
        visited = 0

        def observe(q):
            nonlocal visited
            visited += 1
            if visited == 20:
                cancel.set()
            return valid(q)

        with patch.object(model, "valid", new=observe):
            with self.assertRaises(InterruptedError):
                planner._rrt_path(model, start, end, list(GROUPS["left_arm"]), cancel)
        self.assertEqual(visited, 20)
        self.assertTrue(cancel.is_set())

    def test_cancel_during_shortcut_does_not_publish_the_found_path(self):
        model, end = self.crossing()
        cancel = Event()
        edge_free = model.edge_free
        shortcut_started = False

        def observe(first, second, event):
            nonlocal shortcut_started
            # 搜索时调用方为 C++ MotionValidator；返回 Python 删点后才触发取消。
            if sys._getframe(1).f_code.co_name == "_rrt_path":
                shortcut_started = True
                cancel.set()
            return edge_free(first, second, event)

        with patch.object(model, "edge_free", new=observe):
            with self.assertRaises(InterruptedError):
                connect(model, HOME, end, {"left_arm"}, 2.0, cancel)
        self.assertTrue(shortcut_started)
        self.assertTrue(cancel.is_set())


if __name__ == "__main__":
    unittest.main()
