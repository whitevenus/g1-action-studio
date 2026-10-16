"""一次会话规划并连续播放全部动作、连接和返回。"""

from .model import ArmModel
from .planner import plan_sequence
from .robot import RobotSession


def run_replay(interface, actions, gains, speed, transition_seconds, cancel) -> None:
    with RobotSession(interface, gains, cancel) as session:
        model = ArmModel(session.start_pose.leg_q)
        trajectory = plan_sequence(actions, session.start_pose, model, speed, transition_seconds, cancel)
        session.acquire()
        session.play(trajectory)
