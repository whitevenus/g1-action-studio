"""动作 A → 避碰过渡 → 动作 B；只生成离线轨迹，不接入真机执行器。"""

import hashlib
import math
import random
from pathlib import Path

from .constants import ARM_OFFSETS, ARM_SLOTS, COLLISION_MODEL, DRAFT_SCHEMA, DT, PROFILE
from .author import build_draft, validate_draft
from .files import read_json, write_json
from .trajectory import hold_other_arm, interpolate_positions, metrics, smoothstep
from .collision import COLLISION_MODEL, CollisionModel
from .replay import retime_frames, screen_schedule


def find_path(model, start, end, side, margin=0.02, seed=0, iterations=1200):
    """先尝试直连，否则用两棵小型 RRT 树寻找绕行；失败就拒绝。"""
    start, end = model.validate(start), model.validate(end)
    if side not in ('left', 'right') or not 0.001 <= margin <= 0.10:
        raise ValueError('side 须为 left/right；避碰余量须为 0.001–0.10 米')
    selected = ARM_OFFSETS[side]
    fixed = set(range(len(ARM_SLOTS))) - set(selected)
    if any(start[i] != end[i] for i in fixed):
        raise ValueError('过渡时另一侧手臂必须保持同一姿态')
    for label, q in [('开始', start), ('结束', end)]:
        check = model.clearance(q, margin)
        if not check['valid']:
            raise ValueError('{}姿态碰撞或余量不足：{}'.format(label, check))
    if model.edge_clear(start, end, margin):
        return [start, end]
    rng = random.Random(seed)
    trees = [([start], [-1]), ([end], [-1])]

    def extend(tree, target):
        points, parents = tree
        nearest = min(range(len(points)), key=lambda n: sum((points[n][i] - target[i])**2 for i in selected))
        distance = math.sqrt(sum((points[nearest][i] - target[i])**2 for i in selected))
        ratio = min(1.0, 0.30 / max(distance, 1e-12))
        q = interpolate_positions(points[nearest], target, ratio)
        if not model.edge_clear(points[nearest], q, margin):
            return None
        points.append(q)
        parents.append(nearest)
        return len(points)-1

    def branch(tree, index):
        path = []
        while index >= 0:
            path.append(tree[0][index])
            index = tree[1][index]
        return list(reversed(path))

    for attempt in range(iterations):
        active = attempt % 2
        target = list(start)
        for i in selected:
            target[i] = rng.uniform(float(model.limits[i,0]), float(model.limits[i,1]))
        first = extend(trees[active], target)
        if first is None:
            continue
        meeting = trees[active][0][first]
        # 限制另一棵树的连接次数，避免内层循环无限搜索。
        for _ in range(40):
            second = extend(trees[1-active], meeting)
            if second is None:
                break
            if max(abs(a-b) for a,b in zip(trees[1-active][0][second], meeting)) < 1e-9:
                a, b = branch(trees[active], first), branch(trees[1-active], second)
                path = a + list(reversed(b))[1:]
                if active == 1:
                    path.reverse()
                # 尝试删去多余路点，每条新连接都重新检查碰撞。
                result, index = [path[0]], 0
                while index < len(path)-1:
                    following = len(path)-1
                    while following > index+1 and not model.edge_clear(path[index], path[following], margin):
                        following -= 1
                    result.append(path[following])
                    index = following
                return result
    raise ValueError('离线规划预算内未找到避碰路径；不生成可播放结果')


def smooth_path(path, max_velocity=0.5, max_acceleration=2.0):
    """沿已检查的关节线段做五次时间插值；每个拐点速度和加速度为零。"""
    if any(not math.isfinite(v) or v <= 0 for v in (max_velocity, max_acceleration)):
        raise ValueError('速度、加速度上限必须为正有限数')
    frames = [{'t': 0.0, 'q': list(path[0]), 'phase': 'transition'}]
    for a, b in zip(path, path[1:]):
        distance = max(abs(y-x) for x,y in zip(a,b))
        # 按五次曲线 10u³ − 15u⁴ + 6u⁵ 的导数最大值计算时长。
        seconds = max(2*DT, 1.875*distance/max_velocity,
                      math.sqrt((10/math.sqrt(3))*distance/max_acceleration))
        steps = math.ceil(seconds/DT)
        for step in range(1, steps+1):
            u = step/steps
            blend = smoothstep(u)
            frames.append({'t': round(len(frames)*DT, 6), 'q': interpolate_positions(a, b, blend),
                           'phase': 'transition'})
    return frames


def _action(path, speed_scale):
    data = read_json(path)
    if data.get('schema') == DRAFT_SCHEMA:
        validate_draft(data)
    else:
        data = build_draft(path, None, None, 'offline.transition', 1, '离线动作衔接')
    return retime_frames(data['frames'], speed_scale)


def plan_transition(before, after, output, side='left', speed_scale=0.5,
                    margin=0.02, max_velocity=0.5, max_acceleration=2.0, seed=0, body_q=None):
    if Path(output).exists():
        raise FileExistsError('输出文件已存在，请换新路径')
    model = CollisionModel(body_q)
    first, second = _action(before, speed_scale), _action(after, speed_scale)
    # 与单臂回放一致：另一侧手臂保持同一起始姿态。
    baseline = first[0]['q']
    if side not in ARM_OFFSETS:
        raise ValueError('side 须为 left/right；避碰余量须为 0.001–0.10 米')
    first = hold_other_arm(first, baseline, side)
    second = hold_other_arm(second, baseline, side)
    path = find_path(model, first[-1]['q'], second[0]['q'], side, margin, seed)
    bridge = smooth_path(path, max_velocity, max_acceleration)
    frames = []
    for name, section in [('action_1', first), ('transition', bridge), ('action_2', second)]:
        for frame in section if not frames else section[1:]:
            frames.append({'t': round(len(frames)*DT, 6), 'phase':name, 'q':list(frame['q'])})
    screening = screen_schedule(frames, model.limits, max_velocity, max_acceleration)
    if screening['violations']:
        raise ValueError('组合轨迹超过速度/加速度限制：{}'.format(screening['violations']))
    clearance, mount_gap = 0.5, 0.5
    for index, frame in enumerate(frames):
        check = model.clearance(frame['q'], margin)
        if not check['valid']:
            raise ValueError('完整动作在 {:.2f}s 碰撞或余量不足：{}'.format(frame['t'], check))
        clearance, mount_gap = min(clearance, check['distance_m']), min(mount_gap, check['mount_gap_m'])
        if index and not model.edge_clear(frames[index-1]['q'], frame['q'], margin):
            raise ValueError('完整动作在 {:.2f}s 帧间无法证明无碰撞'.format(frame['t']))
    report = {'schema':'g1-arm7-collision-transition-v1', 'target_profile':PROFILE,
              'arm_slots':list(ARM_SLOTS), 'status':'offline_collision_pass',
              'qualification':'offline_only_not_executable', 'hardware_enabled':False,
              'side':side, 'body_q':model.body_q, 'speed_scale':speed_scale, 'seed':seed,
              'model_xml_sha256':hashlib.sha256(COLLISION_MODEL.read_bytes()).hexdigest(),
              'model_provenance':read_json(COLLISION_MODEL.with_name('provenance.json')),
              'sources':[{'file':Path(p).name,'sha256':hashlib.sha256(Path(p).read_bytes()).hexdigest()} for p in (before,after)],
              'max_velocity_rad_s':max_velocity, 'max_acceleration_rad_s2':max_acceleration,
              'clearance_m':margin, 'minimum_clearance_m':clearance, 'minimum_mount_gap_m':mount_gap,
              'waypoints':path, 'transition_metrics':metrics(bridge), 'metrics':metrics(frames), 'frames':frames}
    write_json(report, output)
    return {key:report[key] for key in ('status','hardware_enabled','qualification','metrics',
                                        'transition_metrics','minimum_clearance_m','minimum_mount_gap_m')}
