import unittest
from threading import Event

import numpy as np
from scipy.interpolate import PPoly

from g1_action_studio.model import GROUPS, ArmModel
from g1_action_studio.motion import Action, StartPose, Trajectory, action_curve, connection_curve
from g1_action_studio.planner import (
    PlanningError,
    check_trajectory,
    connect,
    plan_sequence,
)

HOME = np.array(
    [0, 0.2, 0, 0.8, 0, 0, 0, 0, -0.2, 0, 0.8, 0, 0, 0, 0, 0, 0],
    dtype=np.float64,
)
CROSSING_LEFT = np.array([0.561, 0.238, -1.245, 0.563, 1.853, 0.501, -1.141])


class PlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = ArmModel(np.zeros(12))
        if not cls.model.valid(HOME):
            raise AssertionError(f"测试home无效：{cls.model.last_error}")
        cls.start = StartPose(np.zeros(12), HOME)

    def action(self, group, offset=0.02):
        indices = GROUPS[group]
        q = np.tile(HOME[list(indices)], (2, 1))
        q[:, 0] += [offset, 2 * offset]
        if group == "arms_waist":
            q[:, 7] += [offset, 2 * offset]
            q[:, 14] += [offset, 2 * offset]
        return Action(group, np.array([0.0, 0.4]), q)

    def test_four_groups_complete_enter_action_and_return(self):
        for group, active in GROUPS.items():
            with self.subTest(group=group):
                trajectory = plan_sequence([self.action(group)], self.start, self.model, 1, 2, Event())
                self.assertAlmostEqual(trajectory.duration, 4.4)
                np.testing.assert_array_equal(trajectory.first_q, HOME)
                np.testing.assert_array_equal(trajectory.last_q, HOME)
                inactive = [index for index in range(17) if index not in active]
                moved = False
                for time in np.linspace(0, trajectory.duration, 41):
                    q, dq = trajectory.sample(time)
                    self.assertEqual(q.shape, (17,))
                    np.testing.assert_array_equal(q[inactive], HOME[inactive])
                    np.testing.assert_array_equal(dq[inactive], np.zeros(len(inactive)))
                    moved |= bool(np.any(np.abs(q - HOME) > 0.001))
                self.assertTrue(moved)

    def test_half_speed_only_changes_action_duration(self):
        action = self.action("left_arm")
        fast = plan_sequence([action], self.start, self.model, 1, 2, Event())
        slow = plan_sequence([action], self.start, self.model, 0.5, 2, Event())
        self.assertAlmostEqual(fast.duration, 4.4)
        self.assertAlmostEqual(slow.duration, 4.8)
        q_fast, dq_fast = fast.sample(2.1)
        q_slow, dq_slow = slow.sample(2.2)
        np.testing.assert_allclose(q_slow, q_fast, atol=1e-13)
        np.testing.assert_allclose(dq_slow, dq_fast * 0.5, atol=1e-13)

    def test_waist_to_left_restores_other_arm_and_waist_before_left_action(self):
        trajectory = plan_sequence(
            [self.action("arms_waist"), self.action("left_arm")],
            self.start,
            self.model,
            1,
            2,
            Event(),
        )
        self.assertAlmostEqual(trajectory.duration, 6.8)
        for time in (4.4, 4.5, 4.6, 4.7):
            q, dq = trajectory.sample(time)
            np.testing.assert_array_equal(q[7:], HOME[7:])
            np.testing.assert_array_equal(dq[7:], np.zeros(10))
        np.testing.assert_array_equal(trajectory.last_q, HOME)

    def test_left_to_right_keeps_waist_at_home(self):
        trajectory = plan_sequence(
            [self.action("left_arm"), self.action("right_arm")],
            self.start,
            self.model,
            1,
            2,
            Event(),
        )
        for time in np.linspace(0, trajectory.duration, 83):
            q, dq = trajectory.sample(time)
            np.testing.assert_array_equal(q[14:], HOME[14:])
            np.testing.assert_array_equal(dq[14:], np.zeros(3))

    def test_coincident_valid_connection_has_no_duration(self):
        self.assertIsNone(connect(self.model, HOME, HOME, {"left_arm"}, 2, Event()))
        action = Action("left_arm", np.array([0.0, 0.4]), np.tile(HOME[:7], (2, 1)))
        trajectory = plan_sequence([action], self.start, self.model, 1, 2, Event())
        self.assertEqual(trajectory.duration, 0.4)
        np.testing.assert_array_equal(trajectory.polynomial.x, action.t)
        q = HOME.copy()
        q[0] = self.model.ranges[0, 1] + 1
        with self.assertRaisesRegex(PlanningError, "起点"):
            connect(self.model, q, q, {"left_arm"}, 2, Event())

    def test_connection_does_not_change_unselected_joints(self):
        q = HOME.copy()
        q[14] += 0.02
        with self.assertRaisesRegex(PlanningError, "动作组之外"):
            connect(self.model, HOME, q, {"left_arm"}, 2, Event())

    def test_exact_xml_boundaries_survive_polynomial_rounding(self):
        for joint, first, last, seconds in (
            (0, 2.5704, 2.6704, 0.7),
            (13, -0.968658, 1.61443, 2.0),
            (14, -2.618, 2.618, 2.0),
        ):
            with self.subTest(joint=joint):
                a, b = HOME.copy(), HOME.copy()
                a[joint], b[joint] = first, last
                self.assertTrue(self.model.valid(a), self.model.last_error)
                self.assertTrue(self.model.valid(b), self.model.last_error)
                self.assertTrue(self.model.edge_free(a, b, Event()), self.model.last_error)
                check_trajectory(self.model, connection_curve([a, b], seconds), Event())
                b[joint] = self.model.ranges[joint, 1] + 1e-8
                self.assertFalse(self.model.valid(b))

    def test_valid_frames_with_collision_between_them_are_rejected(self):
        end = HOME.copy()
        end[:7] = CROSSING_LEFT
        self.assertTrue(self.model.valid(HOME), self.model.last_error)
        self.assertTrue(self.model.valid(end), self.model.last_error)
        self.assertFalse(self.model.valid((HOME + end) / 2))
        action = Action("left_arm", np.array([0.0, 1.0]), np.stack([HOME[:7], end[:7]]))
        trajectory = action_curve(action, HOME, 1)
        with self.assertRaisesRegex(PlanningError, "碰撞"):
            check_trajectory(self.model, trajectory, Event())
        np.testing.assert_array_equal(action.q[-1], CROSSING_LEFT)

    def test_curve_returning_to_same_endpoint_checks_its_excursion(self):
        midpoint = HOME.copy()
        midpoint[:7] = (HOME[:7] + CROSSING_LEFT) / 2
        self.assertFalse(self.model.valid(midpoint))
        delta = midpoint - HOME
        coefficients = np.zeros((6, 1, 17))
        coefficients[1, 0] = 16 * delta
        coefficients[2, 0] = -32 * delta
        coefficients[3, 0] = 16 * delta
        coefficients[5, 0] = HOME
        polynomial = PPoly(coefficients, [0.0, 1.0], extrapolate=False)
        trajectory = Trajectory(polynomial, 1, HOME, HOME)
        np.testing.assert_allclose(trajectory.sample(0.5)[0], midpoint)
        self.assertTrue(self.model.valid(trajectory.sample(0)[0]))
        self.assertTrue(self.model.valid(trajectory.sample(1)[0]))
        with self.assertRaisesRegex(PlanningError, "碰撞"):
            check_trajectory(self.model, trajectory, Event())

    def test_range_extremum_is_checked_between_legal_samples(self):
        q = np.tile(HOME[:7], (3, 1))
        q[:, 0] = self.model.ranges[0, 1] - np.array([0.004, 0.0001, 0.0002])
        action = Action("left_arm", np.array([0.0, 0.5, 1.0]), q)
        trajectory = action_curve(action, HOME, 1)
        for time in action.t:
            sample, _ = trajectory.sample(time)
            self.assertTrue(self.model.valid(sample), self.model.last_error)
        speeds = trajectory.polynomial(np.linspace(0, 1, 101), 1)
        self.assertLess(np.max(np.abs(speeds)) * 0.5, 0.02)
        with self.assertRaisesRegex(PlanningError, "left_shoulder_pitch_joint.*越界"):
            check_trajectory(self.model, trajectory, Event())

    def test_dense_recording_keeps_every_knot_without_rechecking_shared_endpoints(self):
        times = np.linspace(0.0, 28.208, 29694)
        positions = np.tile(HOME[:7], (len(times), 1))
        positions[:, 0] += 0.2 * (1 - np.cos(2 * np.pi * times / times[-1]))
        positions[:, 3] += 0.15 * np.sin(np.pi * times / times[-1])
        action = Action("left_arm", times, positions)
        trajectory = action_curve(action, HOME, 0.5)
        coefficients = trajectory.polynomial.c.copy()
        knots = trajectory.polynomial.x.copy()
        sample_times = np.linspace(0, trajectory.duration, 81)
        before = [trajectory.sample(float(time)) for time in sample_times]
        samples = []

        class RecordingModel:
            ranges = self.model.ranges
            last_error = None

            def valid(self, q):
                samples.append(q.copy())
                return True

        check_trajectory(RecordingModel(), trajectory, Event())
        expected = np.broadcast_to(HOME, (len(times), 17)).copy()
        expected[:, :7] = positions
        self.assertEqual(len(samples), len(times))
        np.testing.assert_allclose(samples, expected, atol=1e-14, rtol=0)
        np.testing.assert_array_equal(trajectory.polynomial.c, coefficients)
        np.testing.assert_array_equal(trajectory.polynomial.x, knots)
        for time, (q, dq) in zip(sample_times, before):
            after_q, after_dq = trajectory.sample(float(time))
            np.testing.assert_array_equal(after_q, q)
            np.testing.assert_array_equal(after_dq, dq)

    def test_conservative_range_hull_does_not_reject_a_legal_curve(self):
        # q(u)=upper-.001-.1*(u-.5)^2 stays legal although its control hull does not.
        upper = self.model.ranges[0, 1]
        coefficients = np.zeros((6, 1, 17))
        coefficients[3, 0, 0] = -0.1
        coefficients[4, 0, 0] = 0.1
        coefficients[5, 0] = HOME
        coefficients[5, 0, 0] = upper - 0.026
        polynomial = PPoly(coefficients, [0.0, 1.0], extrapolate=False)
        trajectory = Trajectory(polynomial, 1.0, polynomial(0), polynomial(1))
        self.assertAlmostEqual(float(polynomial(0.5)[0]), upper - 0.001)

        class RangeModel:
            ranges = self.model.ranges
            last_error = None

            def valid(self, q):
                return bool(np.all(q >= self.ranges[:, 0]) and np.all(q <= self.ranges[:, 1]))

        check_trajectory(RangeModel(), trajectory, Event())

    def test_tiny_interval_with_matching_endpoints_checks_its_excursion(self):
        midpoint = HOME.copy()
        midpoint[:7] = (HOME[:7] + CROSSING_LEFT) / 2
        delta = midpoint - HOME
        width = 0.00048
        coefficients = np.zeros((6, 1, 17))
        coefficients[1, 0] = 16 * delta / width**4
        coefficients[2, 0] = -32 * delta / width**3
        coefficients[3, 0] = 16 * delta / width**2
        coefficients[5, 0] = HOME
        polynomial = PPoly(coefficients, [0.0, width], extrapolate=False)
        trajectory = Trajectory(polynomial, width, HOME, HOME)
        np.testing.assert_allclose(trajectory.sample(width / 2)[0], midpoint)
        with self.assertRaisesRegex(PlanningError, "碰撞"):
            check_trajectory(self.model, trajectory, Event())

    def test_cancellation_during_dense_recording_stops_before_remaining_knots(self):
        times = np.linspace(0.0, 1.0, 1001)
        positions = np.tile(HOME[:7], (len(times), 1))
        trajectory = action_curve(Action("left_arm", times, positions), HOME, 1.0)
        cancel = Event()
        samples = []

        class CancellingModel:
            ranges = self.model.ranges
            last_error = None

            def valid(self, q):
                samples.append(q.copy())
                if len(samples) == 10:
                    cancel.set()
                return True

        with self.assertRaises(InterruptedError):
            check_trajectory(CancellingModel(), trajectory, cancel)
        self.assertEqual(len(samples), 10)

    def test_cancel_stops_all_planning_entry_points(self):
        cancel = Event()
        cancel.set()
        trajectory = action_curve(self.action("left_arm"), HOME, 1)
        with self.assertRaises(InterruptedError):
            connect(self.model, HOME, HOME, {"left_arm"}, 2, cancel)
        with self.assertRaises(InterruptedError):
            check_trajectory(self.model, trajectory, cancel)
        with self.assertRaises(InterruptedError):
            plan_sequence([self.action("left_arm")], self.start, self.model, 1, 2, cancel)


if __name__ == "__main__":
    unittest.main()
