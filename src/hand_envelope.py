"""对未知手指姿态做保守区间几何查询，覆盖模型限位内的连续活动范围。"""

import math


def hand_geometry(mujoco, model):
    """每个手指几何的祖先关节，以及转角到几何位移的链长上界。"""
    import numpy as np

    result = {}
    for geom in range(model.ngeom):
        name = model.geom(geom).name
        if "_hand_" not in name or "_palm_" in name or model.geom_group[geom] == 1:
            continue
        body = int(model.geom_bodyid[geom])
        radius = float(np.linalg.norm(model.geom_pos[geom]) + model.geom_rbound[geom])
        joints, reaches = [], []
        while "_hand_" in model.body(body).name:
            for joint in range(int(model.body_jntadr[body]), int(model.body_jntadr[body] + model.body_jntnum[body])):
                if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE or any(
                    abs(x) > 1e-12 for x in model.jnt_pos[joint]
                ):
                    raise ValueError("手部包络只支持关节原点为零的转动关节")
                if not model.jnt_limited[joint]:
                    raise ValueError("手部包络需要明确的关节限位")
                joints.append(joint)
                reaches.append(radius)
            radius += float(np.linalg.norm(model.body_pos[body]))
            body = int(model.body_parentid[body])
        result[geom] = (joints, reaches)
    return result


def swept_distance(mujoco, model, data, a, b, cutoff, geometry):
    """返回全手型集合的距离下界；节点预算耗尽返回根区间下界，绝不降为采样通过。"""
    import numpy as np

    reaches = {}
    for geom in (a, b):
        joints, radii = geometry.get(geom, ((), ()))
        for joint, radius in zip(joints, radii):
            reaches[joint] = reaches.get(joint, 0.0) + radius
    joints = list(reaches)
    radii = np.array(list(reaches.values()))
    addresses = model.jnt_qposadr[joints]
    pending = [(model.jnt_range[joints, 0].copy(), model.jnt_range[joints, 1].copy(), 0)]
    lower = float(cutoff) + 0.0001
    nodes = 0
    while pending:
        low, high, depth = pending.pop()
        nodes += 1
        half = (high - low) / 2
        bound = float(radii @ half)
        data.qpos[addresses] = (low + high) / 2
        mujoco.mj_kinematics(model, data)
        distance = float(mujoco.mj_geomDistance(model, data, a, b, cutoff + bound + 0.0002, None))
        value = distance - bound - 0.0001
        if not math.isfinite(distance):
            raise ValueError("包络距离计算出现非有限值")
        if nodes == 1:
            root_lower = value
        if value > cutoff:
            lower = min(lower, value)
            continue
        if distance < cutoff or depth >= 24 or nodes >= 4096:
            return root_lower
        split = int(np.argmax(radii * half))
        middle = (low[split] + high[split]) / 2
        left_high, right_low = high.copy(), low.copy()
        left_high[split] = right_low[split] = middle
        pending.extend(((low, left_high, depth + 1), (right_low, high, depth + 1)))
    return lower
