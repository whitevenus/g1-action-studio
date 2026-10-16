"""固定腿/腰姿态下的双臂与双手碰撞检查；只计算几何，不运行 DDS 或动力学。"""

import math

from .constants import COLLISION_MODEL, DEX3_LAYOUT, DT
from .trajectory import finite_vector, interpolate_positions, smoothstep


class CollisionModel:
    def __init__(self, body_q=None, *, layout=DEX3_LAYOUT, collision_hand_model="dex3-1"):
        if collision_hand_model != "dex3-1":
            raise ValueError("collision_hand_model 目前仅支持 dex3-1")
        self.layout = layout
        self.hand_geometry = {}
        self.envelope_cache = {}
        try:
            import mujoco
            import numpy as np
        except ImportError as error:
            raise ImportError('避碰检查需要 MuJoCo 3.15+；请安装 pip install -e ".[planning]"') from error
        self.mj, self.np = mujoco, np
        self.model = mujoco.MjModel.from_xml_path(str(COLLISION_MODEL))
        if layout.hand_model is None:
            from .hand_envelope import hand_geometry

            self.hand_geometry = hand_geometry(mujoco, self.model)
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.joints = [m.joint(name + "_joint").id for name in layout.joint_names]
        # 双臂和双手关节的角度存储位置
        self.addresses = m.jnt_qposadr[self.joints]
        # 双臂和双手关节的限位（copy独立数组）
        self.limits = m.jnt_range[self.joints].copy()
        # 检查并设置机器人实际的腰、腿姿态
        body_q = [0.0] * 15 if body_q is None else list(body_q)
        if len(body_q) != 15 or not all(math.isfinite(v) for v in body_q):
            raise ValueError("body_q 必须包含 J0–J14 的 15 个有限弧度值")
        for joint, value in zip(range(15), body_q):
            if not m.jnt_range[joint, 0] <= value <= m.jnt_range[joint, 1]:
                raise ValueError("腿/腰姿态超出模型范围")
            self.data.qpos[m.jnt_qposadr[joint]] = value
        self.body_q = body_q
        # 找出所有手臂和手指部件
        arm_bodies = {m.jnt_bodyid[j] for j in self.joints} | {m.geom_bodyid[g] for g in self.hand_geometry}
        # 找出特殊安装区域（躯干、肩；肘部、腕部；....），间隙为0
        mount_housings = set()
        for side in ("left", "right"):
            for a, b in [
                ("torso_link", side + "_shoulder_roll_link"),
                (side + "_elbow_link", side + "_wrist_pitch_link"),
                (side + "_wrist_roll_link", side + "_wrist_yaw_link"),
            ]:
                mount_housings.add(frozenset((a + "_collision", b + "_collision")))
        # 枚举几何体组合
        self.pairs = []
        self.mount_pairs = []
        for a in range(m.ngeom):
            for b in range(a + 1, m.ngeom):
                if m.geom_group[a] == 1 or m.geom_group[b] == 1:
                    continue  # 官方可视网格不叠加到对应的碰撞几何上。
                if m.geom_type[a] == mujoco.mjtGeom.mjGEOM_PLANE or m.geom_type[b] == mujoco.mjtGeom.mjGEOM_PLANE:
                    continue  # 查看器地板不属于本体几何；无限平面没有有限包围球。
                ba, bb = m.geom_bodyid[[a, b]]
                # 两边都不是手臂或手指时，不进行检查。
                if ba not in arm_bodies and bb not in arm_bodies:
                    continue
                names = [m.geom(a).name, m.geom(b).name]
                if (
                    self.hand_geometry
                    and all("_hand_" in name for name in names)
                    and names[0].split("_")[0] == names[1].split("_")[0]
                ):
                    continue  # 同手活动包络的内部重叠，不作为外部碰撞。
                if ba == bb or m.body_parentid[ba] == bb or m.body_parentid[bb] == ba:
                    continue
                self.pairs.append((a, b))
                # 手部内部空间很小，检查穿透，不套用手与躯干之间的2厘米间隙。
                same_hand = names[0].split("_")[0] == names[1].split("_")[0] and all(
                    "_hand_" in n or "_wrist_" in n for n in names
                )
                self.mount_pairs.append(frozenset(names) in mount_housings or same_hand)
        # 位移上界为半径 × 转角；半径包含网格偏移和所有下游连杆。
        reach = np.zeros((layout.joint_count, m.ngeom))
        for column, joint in enumerate(self.joints):
            for geom in range(m.ngeom):
                body = m.geom_bodyid[geom]
                radius = float(np.linalg.norm(m.geom_pos[geom]) + m.geom_rbound[geom])
                while body and body != m.jnt_bodyid[joint]:
                    radius += float(np.linalg.norm(m.body_pos[body]))
                    body = m.body_parentid[body]
                if body:
                    reach[column, geom] = radius
        self.pair_reach = np.array(
            [np.where((reach[:, a] > 0) & (reach[:, b] > 0), 0, reach[:, a] + reach[:, b]) for a, b in self.pairs]
        )
        self.pairs = np.array(self.pairs)
        self.mount_pairs = np.array(self.mount_pairs)

    # 确认关节数目、是有限数值、没有超过关节限位
    def validate(self, q):
        q = finite_vector(list(q), size=self.layout.joint_count)
        if any(not low <= value <= high for value, (low, high) in zip(q, self.limits)):
            raise ValueError("姿态超出碰撞模型关节范围")
        return q

    def check_motion(self, frames, margin):
        """检查完整轨迹和帧间运动；制作与实际回放规划共用。"""
        clearance, mount_gap = 0.5, 0.5
        for index, frame in enumerate(frames):
            if index and frame["q"] == frames[index - 1]["q"]:
                continue  # 原位接管、保持和释放复用前一帧的几何检查。
            check = self.clearance(frame["q"], margin)
            if not check["valid"]:
                raise ValueError("完整动作在 {:.2f}s 碰撞或余量不足：{}".format(frame["t"], check))
            clearance = min(clearance, check["distance_m"])
            mount_gap = min(mount_gap, check["mount_gap_m"])
            if index and not self.edge_clear(frames[index - 1]["q"], frame["q"], margin):
                raise ValueError("完整动作在 {:.2f}s 帧间无法证明无碰撞".format(frame["t"]))
        return {
            "minimum_clearance_m": clearance,
            "minimum_mount_gap_m": mount_gap,
            "hand_collision_policy": "measured_joints" if self.layout.hand_model else "full_range_envelope",
            "envelope_algorithm": None if self.layout.hand_model else "joint_interval_mesh_union_v1",
            "envelope_parameters": None
            if self.layout.hand_model
            else {"node_budget": 4096, "max_depth": 24, "numerical_margin_m": 0.0001},
        }

    # 写入双臂关节姿态并进行正向运动学计算各个零部件位置
    def _position(self, q):
        self.data.qpos[self.addresses] = q
        self.mj.mj_kinematics(self.model, self.data)

    def _distances(self, cutoffs):
        m, d = self.model, self.data
        a, b = self.pairs.T
        gaps = self.np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b], axis=1) - m.geom_rbound[a] - m.geom_rbound[b]
        distances = self.np.broadcast_to(cutoffs + 0.0002, len(self.pairs)).copy()
        # 已知几何批量粗筛；未知手型必须检查连续关节区间，不能按某个手型粗筛。
        thresholds = self.np.broadcast_to(cutoffs, len(self.pairs))
        from .hand_envelope import swept_distance

        for index, (ga, gb) in enumerate(self.pairs):
            ga, gb = int(ga), int(gb)
            if ga in self.hand_geometry or gb in self.hand_geometry:
                names = [m.geom(g).name for g in (ga, gb)]
                cache_key = None
                # 同侧手与腕的相对几何只依赖腕角，不依赖肩肘或机身姿态。
                side = names[0].split("_")[0]
                if (
                    side in ("left", "right")
                    and all(name.startswith(side + "_") for name in names)
                    and any("_wrist_" in name for name in names)
                ):
                    offset = 4 if side == "left" else 11
                    cache_key = (
                        ga,
                        gb,
                        float(thresholds[index]),
                        tuple(self.data.qpos[self.addresses[offset : offset + 3]]),
                    )
                if cache_key is not None and cache_key in self.envelope_cache:
                    distance = self.envelope_cache[cache_key]
                else:
                    distance = swept_distance(
                        self.mj, m, d, ga, gb, float(thresholds[index]) + 0.0001, self.hand_geometry
                    )
                    if cache_key is not None:
                        self.envelope_cache[cache_key] = distance
                distances[index] = distance
            elif gaps[index] < distances[index]:
                distances[index] = self.mj.mj_geomDistance(m, d, ga, gb, float(distances[index]), None)
        if self.np.isnan(distances).any() or self.np.isposinf(distances).any():
            raise ValueError("碰撞距离计算出现非有限值")
        return distances - 0.0001  # 为距离查询的数值误差留出余量。

    def clearance(self, q, margin=0.02, cutoff=0.5):
        """非安装区最小距离、安装区间隙和首个不合格几何对。"""
        q = self.validate(q)
        self._position(q)
        distances = self._distances(self.np.where(self.mount_pairs, 0, margin) if self.hand_geometry else cutoff)
        bad = self.np.flatnonzero(distances < self.np.where(self.mount_pairs, 0, margin))
        pair = None if not len(bad) else [self.model.geom(int(g)).name for g in self.pairs[bad[0]]]
        return {
            "valid": pair is None,
            "distance_m": float(distances[~self.mount_pairs].min()),
            "mount_gap_m": float(distances[self.mount_pairs].min()),
            "pair": pair,
        }

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


def find_path(model, start, end, side, margin=0.02, timeout=5.0):
    """优先直连，否则用 OMPL RRTConnect；全部连接复用 MuJoCo 区间检查。"""
    try:
        from ompl import base as ob
        from ompl import geometric as og
    except ImportError as error:
        raise ImportError("避碰规划需要 OMPL 2.0.1；请在 Linux Python 3.11 环境安装 .[planning]") from error
    start, end = model.validate(start), model.validate(end)
    if side not in ("left", "right") or not 0.001 <= margin <= 0.10:
        raise ValueError("side 须为 left/right；避碰余量须为 0.001–0.10 米")
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout < 0:
        raise ValueError("规划时间预算须为非负有限数")
    selected = model.layout.indices(side)
    if any(start[i] != end[i] for i in range(model.layout.joint_count) if i not in selected):
        raise ValueError("过渡时另一侧手臂必须保持同一姿态")
    for label, q in [("开始", start), ("结束", end)]:
        check = model.clearance(q, margin)
        if not check["valid"]:
            raise ValueError("{}姿态碰撞或余量不足：{}".format(label, check))
    if model.edge_clear(start, end, margin):
        return [start, end]
    space = ob.RealVectorStateSpace(len(selected))
    bounds = ob.RealVectorBounds(len(selected))
    for j, i in enumerate(selected):
        bounds.setLow(j, float(model.limits[i, 0]))
        bounds.setHigh(j, float(model.limits[i, 1]))
    space.setBounds(bounds)
    setup = og.SimpleSetup(space)
    si = setup.getSpaceInformation()

    def positions(state):
        q = list(start)
        for j, i in enumerate(selected):
            q[i] = state[j]
        return q

    class MotionValidator(ob.MotionValidator):
        def checkMotion(self, before, after):
            return model.edge_clear(positions(before), positions(after), margin)

    validator = MotionValidator(si)
    si.setMotionValidator(validator)
    setup.setStateValidityChecker(lambda state: model.clearance(positions(state), margin)["valid"])
    planner = og.RRTConnect(si)
    planner.setRange(0.30)
    setup.setPlanner(planner)
    first, last = space.allocState(), space.allocState()
    for j, i in enumerate(selected):
        first[j], last[j] = start[i], end[i]
    setup.setStartAndGoalStates(first, last)
    setup.solve(float(timeout))
    if not setup.haveExactSolutionPath():
        raise ValueError("OMPL 规划预算内未找到完整避碰路径；不生成可播放结果")
    path = setup.getSolutionPath()
    simplifier = setup.getPathSimplifier()
    simplifier.reduceVertices(path)
    simplifier.smoothBSpline(path, 2)
    return [positions(state) for state in path.getStates()]


def parameterize_path(path, max_velocity=0.5, max_acceleration=2.0):
    """为已规划的路径安排时长和 50 Hz 采样，不改变空间路径。"""
    if any(not math.isfinite(v) or v <= 0 for v in (max_velocity, max_acceleration)):
        raise ValueError("速度、加速度上限必须为正有限数")
    frames = [{"t": 0.0, "q": list(path[0]), "phase": "transition"}]
    if all(a == b for a, b in zip(path[0], path[-1])) and len(path) == 2:
        return frames
    for a, b in zip(path, path[1:]):
        distance = max(abs(y - x) for x, y in zip(a, b))
        # 按五次曲线 10u³ − 15u⁴ + 6u⁵ 的导数最大值计算时长。
        seconds = max(
            2 * DT, 1.875 * distance / max_velocity, math.sqrt((10 / math.sqrt(3)) * distance / max_acceleration)
        )
        steps = math.ceil(seconds / DT)
        for step in range(1, steps + 1):
            blend = smoothstep(step / steps)
            frames.append(
                {"t": round(len(frames) * DT, 6), "q": interpolate_positions(a, b, blend), "phase": "transition"}
            )
    frames[-1]["q"] = list(path[-1])
    return frames
