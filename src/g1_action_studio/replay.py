"""一次会话规划并连续播放全部动作、连接和返回。"""

from time import monotonic

from .model import ArmModel
from .planner import plan_sequence
from .robot import RobotSession


def run_replay(interface, actions, gains, speed, transition_seconds, cancel) -> None:
    print("等待机器人反馈。", flush=True)
    with RobotSession(interface, gains, cancel) as session:
        print("已收到反馈，正在加载模型。", flush=True)
        model = ArmModel(session.start_pose.leg_q)
        print(f"开始规划与避碰检查，共 {sum(len(action.t) for action in actions)} 帧。", flush=True)
        started = monotonic()
        trajectory = plan_sequence(actions, session.start_pose, model, speed, transition_seconds, cancel)
        print(
            f"规划完成，用时 {monotonic() - started:.2f} 秒，轨迹时长 {trajectory.duration:.2f} 秒；开始接管。",
            flush=True,
        )
        session.acquire()
        print("接管完成，开始回放。", flush=True)
        session.play(trajectory)
        print("回放结束，开始降低权重。", flush=True)
