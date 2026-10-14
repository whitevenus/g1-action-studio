"""动作 A → 避碰过渡 → 动作 B；输出同一种动作格式，供 replay 播放。"""

import hashlib
from pathlib import Path

from .constants import ARM_OFFSETS, ARM_SLOTS, COLLISION_MODEL, ACTION_SCHEMA, DT, PROFILE
from .author import load_action
from .config import write_json
from .trajectory import frame_digest, hold_other_arm, metrics
from .collision import CollisionModel, find_path, smooth_path
from .replay import retime_frames, screen_schedule


def plan_transition(before, after, output, side='left', speed_scale=1.0,
                    margin=0.02, max_velocity=0.5, max_acceleration=2.0, seed=0, body_q=None):
    if Path(output).exists():
        raise FileExistsError('输出文件已存在，请换新路径')
    model = CollisionModel(body_q)
    actions = [load_action(before), load_action(after)]
    if any(action["kind"] == "sequence" and action["side"] != side for action in actions):
        raise ValueError("组合动作只能按制作时的回放侧继续衔接")
    first, second = [retime_frames(action["frames"], speed_scale, smooth=action["kind"] == "single")
                     for action in actions]
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
    collision = model.check_motion(frames, margin)
    report = {'schema':ACTION_SCHEMA, 'kind':'sequence', 'target_profile':PROFILE,
              'arm_slots':list(ARM_SLOTS), 'status':'offline_collision_pass',
              'action_code':'combined.action', 'action_version':1, 'display_name':'组合动作',
              'synthetic':any(action['synthetic'] for action in actions),
              'side':side, 'speed_scale':speed_scale, 'seed':seed,
              'model_xml_sha256':hashlib.sha256(COLLISION_MODEL.read_bytes()).hexdigest(),
              'sources':[{'file':Path(p).name, 'trajectory_sha256':action['trajectory_sha256']}
                         for p,action in zip((before,after),actions)],
              'max_velocity_rad_s':max_velocity, 'max_acceleration_rad_s2':max_acceleration,
              'collision':{'body_q':model.body_q, 'clearance_m':margin}, **collision,
              'waypoints':path,
              'transition_metrics':metrics(bridge) if len(bridge) >= 3 else {'duration_s':0.0, 'frame_count':1},
              'metrics':metrics(frames),
              'frames':frames, 'trajectory_sha256':frame_digest(frames)}
    write_json(report, output)
    return {key:report[key] for key in ('status','kind','synthetic','metrics',
                                        'transition_metrics','minimum_clearance_m','minimum_mount_gap_m')}
