"""固定腿/腰姿态下的双臂碰撞检查；只计算几何，不运行 DDS 或动力学。"""

import math

from .constants import ARM_JOINT_COUNT, COLLISION_MODEL, JOINT_NAMES
from .trajectory import finite_vector


class CollisionModel:
    def __init__(self, body_q=None):
        try:
            import mujoco
            import numpy as np
        except ImportError as error:
            raise ImportError('离线避碰需要 Python 3.10+，请安装 pip install -e ".[planning]"') from error
        self.mj, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_path(str(COLLISION_MODEL))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.joints = [m.joint(name + '_joint').id for name in JOINT_NAMES]
        self.addresses = m.jnt_qposadr[self.joints]
        self.limits = m.jnt_range[self.joints].copy()
        body_q = [0.0] * 15 if body_q is None else list(body_q)
        if len(body_q) != 15 or not all(math.isfinite(v) for v in body_q):
            raise ValueError('body_q 必须包含 J0–J14 的 15 个有限弧度值')
        for joint, value in zip(range(15), body_q):
            if not m.jnt_range[joint, 0] <= value <= m.jnt_range[joint, 1]:
                raise ValueError('腿/腰姿态超出模型范围')
            self.data.qpos[m.jnt_qposadr[joint]] = value
        self.body_q = body_q
        arm_bodies = {m.jnt_bodyid[j] for j in self.joints}
        mount_housings = set()
        for side in ('left', 'right'):
            for a, b in [('torso_link', side + '_shoulder_roll_link'),
                         (side + '_elbow_link', side + '_wrist_pitch_link'),
                         (side + '_wrist_roll_link', side + '_wrist_yaw_link')]:
                mount_housings.add(frozenset((a + '_collision', b + '_collision')))
        self.pairs = []
        self.mount_pairs = []
        for a in range(m.ngeom):
            for b in range(a + 1, m.ngeom):
                ba, bb = m.geom_bodyid[[a, b]]
                if ba not in arm_bodies and bb not in arm_bodies:
                    continue
                # 直接相连的关节外壳属于安装区域，不互相检查。
                if ba == bb or m.body_parentid[ba] == bb or m.body_parentid[bb] == ba:
                    continue
                names = [m.geom(a).name, m.geom(b).name]
                # 手指包络覆盖所有角度并伸入本侧腕部，模型不检查手部内部碰撞。
                if any(n.endswith('_sweep') for n in names) and any('_wrist_' in n for n in names):
                    if names[0].split('_')[0] == names[1].split('_')[0]:
                        continue
                self.pairs.append((a, b))
                self.mount_pairs.append(frozenset(names) in mount_housings)
        # 位移上界为半径 × 转角；半径包含网格偏移和所有下游连杆。
        self.reach = np.zeros((ARM_JOINT_COUNT, m.ngeom))
        for column, joint in enumerate(self.joints):
            for geom in range(m.ngeom):
                body = m.geom_bodyid[geom]
                radius = float(np.linalg.norm(m.geom_pos[geom]) + m.geom_rbound[geom])
                while body and body != m.jnt_bodyid[joint]:
                    radius += float(np.linalg.norm(m.body_pos[body]))
                    body = m.body_parentid[body]
                if body:
                    self.reach[column, geom] = radius
        self.pair_reach = np.array([np.where((self.reach[:, a] > 0) & (self.reach[:, b] > 0),
                                            0, self.reach[:, a] + self.reach[:, b]) for a, b in self.pairs])
        self.pairs = np.array(self.pairs)
        self.mount_pairs = np.array(self.mount_pairs)

    def validate(self, q):
        q = finite_vector(q)
        if any(not low <= value <= high for value, (low, high) in zip(q, self.limits)):
            raise ValueError('双臂姿态超出碰撞模型关节范围')
        return q

    def _position(self, q):
        self.data.qpos[self.addresses] = q
        self.mj.mj_kinematics(self.model, self.data)

    def _distances(self, cutoffs):
        m, d = self.model, self.data
        a, b = self.pairs.T
        gaps = self.np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b], axis=1) - m.geom_rbound[a] - m.geom_rbound[b]
        distances = self.np.broadcast_to(cutoffs + 0.0001, len(self.pairs)).copy()
        # 先批量粗筛，只对距离较近的几何对查询凸包距离。
        for index in self.np.flatnonzero(gaps < distances):
            ga, gb = self.pairs[index]
            distances[index] = self.mj.mj_geomDistance(m, d, ga, gb, float(distances[index]), None)
        if not self.np.isfinite(distances).all():
            raise ValueError('碰撞距离计算出现非有限值')
        return distances - 0.0001  # 为距离查询的数值误差留出余量。

    def clearance(self, q, margin=0.02, cutoff=0.5):
        """非安装区最小距离、安装区间隙和首个不合格几何对。"""
        q = self.validate(q)
        self._position(q)
        distances = self._distances(cutoff)
        bad = self.np.flatnonzero(distances < self.np.where(self.mount_pairs, 0, margin))
        pair = None if not len(bad) else [self.model.geom(int(g)).name for g in self.pairs[bad[0]]]
        return {'valid': pair is None, 'distance_m': float(distances[~self.mount_pairs].min()),
                'mount_gap_m': float(distances[self.mount_pairs].min()), 'pair': pair}

    def edge_clear(self, start, end, margin):
        """递归检查整段关节线段；不能证明区间安全时返回 False。"""
        start, end = self.np.array(self.validate(start)), self.np.array(self.validate(end))
        pending = [(start, end, 0)]
        while pending:
            a, b, depth = pending.pop()
            midpoint = (a + b) * 0.5
            bounds = self.pair_reach @ (abs(b - a) * 0.5)
            self._position(midpoint)
            required = self.np.where(self.mount_pairs, 0, margin)
            distances = self._distances(required + bounds + 1e-9)
            if (distances < required).any():
                return False
            if (distances > required + bounds).all():
                continue
            if depth >= 18:
                return False
            pending.extend(((a, midpoint, depth + 1), (midpoint, b, depth + 1)))
        return True
