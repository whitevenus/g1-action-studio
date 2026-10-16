"""录制所选真实反馈，先冻结保存，再恢复保持增益并避碰返回。"""

from .model import ArmModel
from .motion import save_action
from .planner import check_trajectory, connect
from .robot import RobotSession


def run_teach(interface, group, gains, output, transition_seconds, cancel) -> None:
    from .cli import _TerminalInput

    with _TerminalInput(cancel) as terminal:
        with RobotSession(interface, gains, cancel) as session:
            session.acquire()
            session.follow(group)
            terminal.wait_enter("跟随已就绪，按回车开始录制。")
            session.begin_recording()
            terminal.wait_enter("录制中，按回车结束并保存。")
            action, end_q = session.end_recording()
            save_action(action, output)
            session.hold()
            model = ArmModel(session.start_pose.leg_q)
            trajectory = connect(model, end_q, session.start_pose.upper_q, {group}, transition_seconds, cancel)
            if trajectory is not None:
                check_trajectory(model, trajectory, cancel)
                session.play(trajectory)
