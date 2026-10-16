from collections.abc import Sequence
from math import ceil, ulp
from pathlib import Path
from threading import Event

# SDK slot order is defined once; upper-body order puts both arms before the waist.
JOINTS = (
    ("left_hip_pitch_joint", 0),
    ("left_hip_roll_joint", 1),
    ("left_hip_yaw_joint", 2),
    ("left_knee_joint", 3),
    ("left_ankle_pitch_joint", 4),
    ("left_ankle_roll_joint", 5),
    ("right_hip_pitch_joint", 6),
    ("right_hip_roll_joint", 7),
    ("right_hip_yaw_joint", 8),
    ("right_knee_joint", 9),
    ("right_ankle_pitch_joint", 10),
    ("right_ankle_roll_joint", 11),
    ("waist_yaw_joint", 12),
    ("waist_roll_joint", 13),
    ("waist_pitch_joint", 14),
    ("left_shoulder_pitch_joint", 15),
    ("left_shoulder_roll_joint", 16),
    ("left_shoulder_yaw_joint", 17),
    ("left_elbow_joint", 18),
    ("left_wrist_roll_joint", 19),
    ("left_wrist_pitch_joint", 20),
    ("left_wrist_yaw_joint", 21),
    ("right_shoulder_pitch_joint", 22),
    ("right_shoulder_roll_joint", 23),
    ("right_shoulder_yaw_joint", 24),
    ("right_elbow_joint", 25),
    ("right_wrist_roll_joint", 26),
    ("right_wrist_pitch_joint", 27),
    ("right_wrist_yaw_joint", 28),
)
LEG_JOINTS = tuple(name for name, _ in JOINTS[:12])
UPPER_JOINTS = tuple(name for name, _ in JOINTS[15:] + JOINTS[12:15])
UPPER_SLOTS = tuple(slot for _, slot in JOINTS[15:] + JOINTS[12:15])
GROUPS = {
    "left_arm": tuple(range(7)),
    "right_arm": tuple(range(7, 14)),
    "arms": tuple(range(14)),
    "arms_waist": tuple(range(17)),
}
ANGULAR_RESOLUTION = 0.02


def within_range(value: float, lower: float, upper: float) -> bool:
    # 五次曲线的 Horner 求值会累积数个 ULP；仅容纳机器舍入，不修改目标。
    rounding = 32 * ulp(max(1.0, abs(lower), abs(upper)))
    return lower - rounding <= value <= upper + rounding


class ArmModel:
    def __init__(self, leg_q: Sequence[float]):
        import mujoco
        import numpy as np

        self._mj = mujoco
        self._np = np
        self.model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("resources") / "g1_29dof.xml"))
        self.data = mujoco.MjData(self.model)
        self.leg_q = np.array(leg_q, dtype=np.float64, copy=True)
        self.leg_q.flags.writeable = False
        upper_ids = np.array([self.model.joint(name).id for name in UPPER_JOINTS])
        leg_ids = np.array([self.model.joint(name).id for name in LEG_JOINTS])
        self.upper_qpos = self.model.jnt_qposadr[upper_ids].copy()
        self.leg_qpos = self.model.jnt_qposadr[leg_ids].copy()
        self.ranges = self.model.jnt_range[upper_ids].copy()
        self.ranges.flags.writeable = False
        upper_bodies = set(self.model.jnt_bodyid[upper_ids])
        affected = np.zeros(self.model.nbody, dtype=bool)
        for body in range(1, self.model.nbody):
            ancestor = body
            while ancestor:
                if ancestor in upper_bodies:
                    affected[body] = True
                    break
                ancestor = self.model.body_parentid[ancestor]
        collidable = (self.model.geom_contype != 0) | (self.model.geom_conaffinity != 0)
        self.affected_geoms = affected[self.model.geom_bodyid] & collidable
        self.last_error: str | None = None

    def set_pose(self, upper_q: Sequence[float], data=None) -> None:
        if data is None:
            data = self.data
        data.qpos[:] = self.model.qpos0
        data.qpos[self.leg_qpos] = self.leg_q
        data.qpos[self.upper_qpos] = upper_q
        self._mj.mj_fwdPosition(self.model, data)

    def valid(self, upper_q: Sequence[float]) -> bool:
        self.last_error = None
        q = self._np.asarray(upper_q)
        outside = [
            index for index, (value, bounds) in enumerate(zip(q, self.ranges)) if not within_range(value, *bounds)
        ]
        if len(outside):
            index = outside[0]
            self.last_error = f"关节{UPPER_JOINTS[index]}越界：{q[index]:.9g} 不在{self.ranges[index].tolist()}"
            return False
        self.set_pose(q)
        for contact in self.data.contact:
            first, second = contact.geom1, contact.geom2
            if contact.dist <= 0 and (self.affected_geoms[first] or self.affected_geoms[second]):
                names = [self.model.body(self.model.geom_bodyid[geom]).name for geom in (first, second)]
                self.last_error = f"碰撞{names[0]} / {names[1]}，距离={contact.dist:.9g}"
                return False
        return True

    def edge_free(self, a: Sequence[float], b: Sequence[float], cancel: Event) -> bool:
        a, b = self._np.asarray(a), self._np.asarray(b)
        steps = max(1, ceil(float(self._np.max(self._np.abs(b - a))) / ANGULAR_RESOLUTION))
        for fraction in self._np.linspace(0, 1, steps + 1):
            if cancel.is_set():
                raise InterruptedError("规划已取消")
            if not self.valid(a + fraction * (b - a)):
                return False
        return True
