"""从项目目录外使用已安装的 wheel 读取示例并规划；不替换模型或曲线。"""

import sys
from pathlib import Path
from threading import Event

import numpy as np

from g1_action_studio.model import GROUPS, ArmModel
from g1_action_studio.motion import load_action, load_start
from g1_action_studio.planner import plan_sequence


def main():
    examples = Path(sys.argv[1]).resolve()
    start = load_start(examples / "start.json")
    model = ArmModel(start.leg_q)
    sequences = [[group] for group in GROUPS]
    sequences += [["arms_waist", "left_arm"], ["left_arm", "right_arm"]]
    for groups in sequences:
        actions = [load_action(examples / f"{group}.json") for group in groups]
        trajectory = plan_sequence(actions, start, model, 1.0, 2.0, Event())
        np.testing.assert_array_equal(trajectory.first_q, start.upper_q)
        np.testing.assert_array_equal(trajectory.last_q, start.upper_q)
        for time in np.linspace(0, trajectory.duration, 101):
            q, dq = trajectory.sample(float(time))
            assert q.shape == dq.shape == (17,)
            assert model.valid(q), model.last_error
        print(f"installed preview: {' -> '.join(groups)}; {trajectory.duration:g}s")
    import g1_action_studio

    print(f"package: {g1_action_studio.__file__}")
    assert not any(name.startswith("unitree") or name.startswith("cyclonedds") for name in sys.modules)


if __name__ == "__main__":
    main()
