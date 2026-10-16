"""把规划的同一条曲线交给 MuJoCo 查看器。"""

import time

from g1_action_studio.model import ArmModel
from g1_action_studio.planner import plan_sequence


def run_preview(actions, start, speed, transition_seconds, cancel) -> None:
    model = ArmModel(start.leg_q)
    trajectory = plan_sequence(actions, start, model, speed, transition_seconds, cancel)
    import mujoco
    import mujoco.viewer

    data = mujoco.MjData(model.model)
    model.set_pose(trajectory.first_q, data)
    with mujoco.viewer.launch_passive(model.model, data) as viewer:
        started = time.monotonic()
        while viewer.is_running() and not cancel.is_set():
            q, _ = trajectory.sample(time.monotonic() - started)
            with viewer.lock():
                model.set_pose(q, data)
            viewer.sync()
            cancel.wait(0.02)
