import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from numpy.testing import assert_allclose, assert_array_equal
from scipy.interpolate import PPoly

from g1_action_studio.model import GROUPS
from g1_action_studio.motion import (
    Action,
    StartPose,
    Trajectory,
    action_curve,
    concatenate,
    connection_curve,
    load_action,
    load_start,
    save_action,
)


class FileTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent)
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "action.json"

    def write(self, data):
        self.path.write_text(json.dumps(data), encoding="utf-8")
        return self.path

    def action_data(self, group="left_arm"):
        width = len(GROUPS[group])
        return {
            "group": group,
            "frames": [
                {"t": 0.0, "q": [0.0] * width},
                {"t": 0.12, "q": [0.1] * width},
            ],
        }

    def test_four_groups_load_and_save_only_core_fields(self):
        for group, indices in GROUPS.items():
            with self.subTest(group=group):
                action = load_action(self.write(self.action_data(group)))
                self.assertEqual(action.group, group)
                self.assertEqual(action.q.shape, (2, len(indices)))
                self.assertEqual(action.t.dtype, np.float64)
                self.assertEqual(action.q.dtype, np.float64)
                self.assertFalse(action.t.flags.writeable)
                self.assertFalse(action.q.flags.writeable)
                output = self.path.with_name(f"{group}.json")
                save_action(action, output)
                saved = json.loads(output.read_text(encoding="utf-8"))
                self.assertEqual(saved, self.action_data(group))
                self.assertTrue(output.read_bytes().endswith(b"\n"))
                assert_array_equal(load_action(output).q, action.q)

    def test_existing_output_is_preserved(self):
        self.path.write_bytes(b"existing content")
        with self.assertRaises(FileExistsError):
            save_action(Action("arms", [0, 1], np.zeros((2, 14))), self.path)
        self.assertEqual(self.path.read_bytes(), b"existing content")

    def test_rejects_old_and_extra_fields(self):
        for key in ("side", "schema", "notes", "qualification", "dq", "hand"):
            with self.subTest(key=key):
                data = self.action_data()
                data[key] = "unsupported"
                with self.assertRaisesRegex(ValueError, "action.json"):
                    load_action(self.write(data))
        for key in ("dq", "notes", "schema"):
            with self.subTest(frame_key=key):
                data = self.action_data()
                data["frames"][0][key] = []
                with self.assertRaises(ValueError):
                    load_action(self.write(data))
        with self.assertRaises(ValueError):
            load_action(self.write({"side": "left", "frames": []}))

    def test_rejects_wrong_group_dimensions_and_structures(self):
        for group in ("left", "dex3", None, [], True):
            with self.subTest(group=group):
                data = self.action_data()
                data["group"] = group
                with self.assertRaises(ValueError):
                    load_action(self.write(data))
        for group in GROUPS:
            data = self.action_data(group)
            data["frames"][1]["q"].append(0.0)
            with self.subTest(wrong_width=group), self.assertRaises(ValueError):
                load_action(self.write(data))
        for frames in (None, {}, [], [{"t": 0, "q": [0] * 7}], [0, 1]):
            data = self.action_data()
            data["frames"] = frames
            with self.subTest(frames=frames), self.assertRaises(ValueError):
                load_action(self.write(data))

    def test_rejects_boolean_nonfinite_and_invalid_timestamps(self):
        for value in (True, False, float("nan"), float("inf"), "1", None, 10**400):
            for field in ("t", "q"):
                with self.subTest(field=field, value=value):
                    data = self.action_data()
                    if field == "t":
                        data["frames"][1]["t"] = value
                    else:
                        data["frames"][1]["q"][0] = value
                    with self.assertRaises(ValueError):
                        load_action(self.write(data))
        for times in ((0.1, 1), (0, 0), (0, -1)):
            data = self.action_data()
            for frame, t in zip(data["frames"], times):
                frame["t"] = t
            with self.subTest(times=times), self.assertRaises(ValueError):
                load_action(self.write(data))

    def test_start_pose_has_one_waist_and_exact_dimensions(self):
        data = {"leg_q": [0.2] * 12, "upper_q": [0.3] * 17}
        start = load_start(self.write(data))
        assert_array_equal(start.leg_q, data["leg_q"])
        assert_array_equal(start.upper_q, data["upper_q"])
        self.assertFalse(start.leg_q.flags.writeable)
        self.assertFalse(start.upper_q.flags.writeable)
        for key, value in (("body_q", [0] * 15), ("schema", "v2")):
            with self.subTest(key=key), self.assertRaises(ValueError):
                load_start(self.write({**data, key: value}))
        for key in data:
            invalid = {**data, key: data[key][:-1]}
            with self.subTest(key=key), self.assertRaises(ValueError):
                load_start(self.write(invalid))

    def test_arrays_do_not_alias_construction_inputs(self):
        q = np.zeros((2, 7))
        action = Action("left_arm", [0, 1], q)
        upper = np.zeros(17)
        start = StartPose(np.zeros(12), upper)
        q[:] = 2
        upper[:] = 3
        assert_array_equal(action.q, np.zeros((2, 7)))
        assert_array_equal(start.upper_q, np.zeros(17))
        with self.assertRaises(ValueError):
            action.q[0, 0] = 1


class CurveTests(unittest.TestCase):
    def action(self, group):
        width = len(GROUPS[group])
        q = np.outer([0.0, 0.2, 0.5, 0.8], np.linspace(0.5, 1.0, width))
        return Action(group, [0, 0.3, 0.8, 1.3], q)

    def test_four_groups_expand_without_moving_unselected_joints(self):
        home = np.linspace(-0.3, 0.3, 17)
        for group, active in GROUPS.items():
            with self.subTest(group=group):
                action = self.action(group)
                curve = action_curve(action, home, 1)
                self.assertIsInstance(curve.polynomial, PPoly)
                self.assertEqual(curve.polynomial.c.shape, (6, 3, 17))
                inactive = sorted(set(range(17)) - set(active))
                for t in np.linspace(0, curve.duration, 17):
                    q, dq = curve.sample(t)
                    self.assertEqual(q.shape, (17,))
                    self.assertEqual(dq.shape, (17,))
                    assert_array_equal(q[inactive], home[inactive])
                    assert_array_equal(dq[inactive], np.zeros(len(inactive)))
                for index, t in enumerate(action.t):
                    assert_allclose(curve.sample(t)[0][list(active)], action.q[index])

    def test_derivative_matches_position_and_half_speed(self):
        action = self.action("arms_waist")
        normal = action_curve(action, np.zeros(17), 1)
        slow = action_curve(action, np.zeros(17), 0.5)
        self.assertEqual(slow.duration, 2 * normal.duration)
        for t in (0.12, 0.3, 0.51, 0.8, 1.11):
            q, dq = normal.sample(t)
            epsilon = 1e-6
            difference = normal.sample(t + epsilon)[0] - normal.sample(t - epsilon)[0]
            assert_allclose(dq, difference / (2 * epsilon), rtol=1e-8, atol=1e-8)
            slow_q, slow_dq = slow.sample(2 * t)
            assert_allclose(slow_q, q, rtol=1e-13, atol=1e-13)
            assert_allclose(slow_dq, 0.5 * dq, rtol=1e-13, atol=1e-13)
        self.assertGreater(np.linalg.norm(normal.sample(0.3)[1]), 0)
        for t, expected in ((-1, action.q[0]), (0, action.q[0]), (1.3, action.q[-1]), (5, action.q[-1])):
            q, dq = normal.sample(t)
            assert_array_equal(q, expected)
            assert_array_equal(dq, np.zeros(17))
        q, _ = normal.sample(0)
        q[:] = 99
        assert_array_equal(normal.sample(0)[0], action.q[0])

    def test_quintic_connection_preserves_segments_and_stops_at_corners(self):
        a = np.zeros(17)
        b = a.copy()
        b[0] = 1
        c = b.copy()
        c[1] = 2
        curve = connection_curve([a, a, b, b, c], 3)
        self.assertEqual(curve.duration, 3)
        assert_allclose(curve.polynomial.x, [0, 1, 3])
        for segment, (start, end, seconds) in enumerate(((a, b, 1), (b, c, 2))):
            for u in (0.1, 0.4, 0.8):
                t = curve.polynomial.x[segment] + u * seconds
                q, dq = curve.sample(t)
                h = 10 * u**3 - 15 * u**4 + 6 * u**5
                assert_allclose(q, start + h * (end - start), atol=1e-13)
                assert_allclose(dq, 30 * u**2 * (1 - u) ** 2 * (end - start) / seconds, atol=1e-13)
        q, dq = curve.sample(1)
        assert_array_equal(q, b)
        assert_array_equal(dq, np.zeros(17))
        self.assertIsNone(connection_curve([a, a, a], 3))

    def test_concatenation_keeps_original_coefficients_path_and_derivative(self):
        action = self.action("arms")
        first = action_curve(action, np.zeros(17), 1)
        zero = connection_curve([first.last_q, first.last_q], 2)
        second = connection_curve([first.last_q, np.zeros(17)], 2)
        combined = concatenate([None, first, zero, second, None])
        self.assertEqual(combined.duration, first.duration + second.duration)
        self.assertEqual(len(combined.polynomial.x), len(first.polynomial.x) + len(second.polynomial.x) - 1)
        intervals = first.polynomial.c.shape[1]
        assert_array_equal(combined.polynomial.c[:, :intervals], first.polynomial.c)
        assert_array_equal(combined.polynomial.c[:, intervals:], second.polynomial.c)
        for curve, offset in ((first, 0), (second, first.duration)):
            for t in np.linspace(0.01, curve.duration - 0.01, 19):
                for actual, expected in zip(combined.sample(offset + t), curve.sample(t)):
                    assert_allclose(actual, expected, rtol=1e-11, atol=1e-11)
        assert_array_equal(combined.sample(combined.duration)[0], np.zeros(17))
        assert_array_equal(combined.sample(combined.duration)[1], np.zeros(17))
        with self.assertRaises(ValueError):
            concatenate([None])

    def test_internal_boundary_selects_the_later_interval(self):
        coefficients = np.zeros((6, 1, 17))
        coefficients[4] = 1
        first = Trajectory(PPoly(coefficients.copy(), [0, 1]), 1, np.zeros(17), np.ones(17))
        coefficients[4] = 2
        coefficients[5] = 1
        second = Trajectory(PPoly(coefficients.copy(), [0, 1]), 1, np.ones(17), np.full(17, 3))
        combined = concatenate([first, second])
        q, dq = combined.sample(1)
        assert_array_equal(q, np.ones(17))
        assert_array_equal(dq, np.full(17, 2))


if __name__ == "__main__":
    unittest.main()
