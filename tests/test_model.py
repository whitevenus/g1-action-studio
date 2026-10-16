import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from threading import Event

import mujoco
import numpy as np

from g1_action_studio.model import (
    GROUPS,
    LEG_JOINTS,
    UPPER_JOINTS,
    UPPER_SLOTS,
    ArmModel,
)

HOME = np.array(
    [0, 0.2, 0, 0.8, 0, 0, 0, 0, -0.2, 0, 0.8, 0, 0, 0, 0, 0, 0],
    dtype=np.float64,
)


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = ArmModel(np.zeros(12))

    def test_mapping_import_does_not_load_numeric_or_geometry_libraries(self):
        source = Path(__file__).resolve().parents[1] / "src"
        subprocess.run(
            [
                sys.executable,
                "-c",
                f"import sys; sys.path.insert(0, {str(source)!r}); "
                "import g1_action_studio.model; "
                "assert 'mujoco' not in sys.modules; "
                "assert 'numpy' not in sys.modules",
            ],
            check=True,
        )

    def test_four_groups_and_named_joint_addresses(self):
        self.assertEqual(
            {name: len(indices) for name, indices in GROUPS.items()},
            {"left_arm": 7, "right_arm": 7, "arms": 14, "arms_waist": 17},
        )
        self.assertEqual(UPPER_SLOTS, tuple(range(15, 29)) + (12, 13, 14))
        self.assertEqual(self.robot.model.njnt, 30)
        self.assertEqual(self.robot.model.nq, 36)
        self.assertEqual(self.robot.ranges.shape, (17, 2))
        for index, name in enumerate(UPPER_JOINTS):
            address = self.robot.model.joint(name).qposadr[0]
            self.assertEqual(self.robot.upper_qpos[index], address)
            self.assertNotEqual(address, UPPER_SLOTS[index])
        self.assertEqual(len(LEG_JOINTS), 12)

    def test_official_resources_are_complete_and_have_no_finger_joints(self):
        import g1_action_studio.model as module

        resources = Path(module.__file__).with_name("resources")
        xml = ET.parse(resources / "g1_29dof.xml")
        meshes = {node.attrib["file"] for node in xml.findall("./asset/mesh")}
        self.assertEqual(meshes, {path.name for path in (resources / "meshes").iterdir()})
        self.assertEqual(len(meshes), 36)
        for word in ("thumb", "index", "middle", "finger"):
            self.assertFalse(any(word in node.attrib["name"] for node in xml.iter("joint") if "name" in node.attrib))
        self.assertIn("BSD 3-Clause License", (resources / "LICENSE").read_text())
        self.assertIn(
            "ab03eec53238487c4a7c7f61dd8e29a2f66abc04",
            (resources / "ORIGIN.md").read_text(encoding="utf-8"),
        )

    def test_separate_viewer_data_has_fixed_legs_and_default_free_base(self):
        legs = np.array([-0.1, 0, 0, 0.2, -0.1, 0] * 2)
        robot = ArmModel(legs)
        robot.set_pose(HOME)
        original = robot.data.qpos.copy()
        viewer_data = mujoco.MjData(robot.model)
        moved = HOME.copy()
        moved[14:] = [0.3, 0.1, -0.1]
        robot.set_pose(moved, viewer_data)
        np.testing.assert_array_equal(robot.data.qpos, original)
        np.testing.assert_array_equal(viewer_data.qpos[:7], robot.model.qpos0[:7])
        np.testing.assert_array_equal(viewer_data.qpos[robot.leg_qpos], legs)
        np.testing.assert_array_equal(viewer_data.qpos[robot.upper_qpos], moved)

    def test_dynamic_waist_moves_torso_head_and_both_arms_but_not_legs(self):
        self.robot.set_pose(HOME)
        before_positions = self.robot.data.xpos.copy()
        before_rotations = self.robot.data.xmat.copy()
        moved = HOME.copy()
        moved[14:] = [0.3, 0.1, -0.1]
        self.robot.set_pose(moved)
        for name in ("torso_link", "left_elbow_link", "right_elbow_link"):
            body = self.robot.model.body(name).id
            self.assertFalse(np.allclose(self.robot.data.xmat[body], before_rotations[body]))
        for name in ("left_knee_link", "right_ankle_roll_link"):
            body = self.robot.model.body(name).id
            np.testing.assert_array_equal(self.robot.data.xpos[body], before_positions[body])
        head_mesh = self.robot.model.mesh("head_link").id
        head_geoms = np.flatnonzero(
            (self.robot.model.geom_type == mujoco.mjtGeom.mjGEOM_MESH) & (self.robot.model.geom_dataid == head_mesh)
        )
        self.assertTrue(any(self.robot.affected_geoms[head_geoms]))
        for geom in head_geoms:
            if self.robot.model.geom_contype[geom] == self.robot.model.geom_conaffinity[geom] == 0:
                self.assertFalse(self.robot.affected_geoms[geom])

    def test_range_and_cancelled_edge(self):
        invalid = HOME.copy()
        invalid[5] = self.robot.ranges[5, 1] + 0.001
        self.assertFalse(self.robot.valid(invalid))
        self.assertIn(UPPER_JOINTS[5], self.robot.last_error)
        cancel = Event()
        cancel.set()
        with self.assertRaises(InterruptedError):
            self.robot.edge_free(HOME, HOME, cancel)

    def test_valid_endpoints_can_collide_with_fixed_leg_between_frames(self):
        end = HOME.copy()
        end[:7] = [0.561, 0.238, -1.245, 0.563, 1.853, 0.501, -1.141]
        self.assertTrue(self.robot.valid(HOME), self.robot.last_error)
        self.assertTrue(self.robot.valid(end), self.robot.last_error)
        self.assertFalse(self.robot.valid((HOME + end) / 2))
        self.assertIn("left_hip_pitch_link", self.robot.last_error)
        self.assertIn("left_wrist_yaw_link", self.robot.last_error)
        self.assertFalse(self.robot.edge_free(HOME, end, Event()))
        self.assertTrue(self.robot.valid(HOME), self.robot.last_error)
        self.assertIsNone(self.robot.last_error)


if __name__ == "__main__":
    unittest.main()
