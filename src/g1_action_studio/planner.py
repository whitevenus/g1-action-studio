from collections.abc import Iterable, Sequence
from math import ceil
from threading import Event
from time import monotonic

import numpy as np

from .model import ANGULAR_RESOLUTION, GROUPS, UPPER_JOINTS, ArmModel, within_range
from .motion import (
    Action,
    StartPose,
    Trajectory,
    action_curve,
    concatenate,
    connection_curve,
)

SEARCH_SECONDS = 5.0
SHORTCUT_SECONDS = 1.0


class PlanningError(RuntimeError):
    pass


def _cancelled(cancel: Event) -> None:
    if cancel.is_set():
        raise InterruptedError("规划已取消")


def _critical_times(coefficients: np.ndarray, width: float) -> list[float]:
    coefficients = np.trim_zeros(coefficients, "f")
    times = [0.0, width]
    if len(coefficients) > 1:
        for root in np.roots(coefficients):
            if abs(root.imag) <= 1e-10 and 0 < root.real < width:
                times.append(float(root.real))
    return times


def _rrt_path(model, a, b, active, cancel):
    from ompl import base, geometric

    space = base.RealVectorStateSpace(len(active))
    bounds = base.RealVectorBounds(len(active))
    for dimension, joint in enumerate(active):
        bounds.setLow(dimension, float(model.ranges[joint, 0]))
        bounds.setHigh(dimension, float(model.ranges[joint, 1]))
    space.setBounds(bounds)
    setup = geometric.SimpleSetup(space)
    information = setup.getSpaceInformation()

    def expand(state):
        q = a.copy()
        q[active] = [state[index] for index in range(len(active))]
        return q

    class MotionValidator(base.MotionValidator):
        def checkMotion(self, first, second):
            if cancel.is_set():
                return False
            try:
                return model.edge_free(expand(first), expand(second), cancel)
            except InterruptedError:
                return False

    setup.setStateValidityChecker(lambda state: not cancel.is_set() and model.valid(expand(state)))
    information.setup()
    original_validator = information.getMotionValidator()
    information.setMotionValidator(MotionValidator(information))
    try:
        first, last = space.allocState(), space.allocState()
        for dimension, joint in enumerate(active):
            first[dimension], last[dimension] = float(a[joint]), float(b[joint])
        setup.setStartAndGoalStates(first, last)
        setup.setPlanner(geometric.RRTConnect(information))
        deadline = monotonic() + SEARCH_SECONDS
        termination = base.PlannerTerminationCondition(lambda: cancel.is_set() or monotonic() >= deadline)
        setup.solve(termination)
        _cancelled(cancel)
        problem = setup.getProblemDefinition()
        if not problem.hasExactSolution():
            reason = "仅找到近似解" if problem.hasApproximateSolution() else "未找到精确解"
            raise PlanningError(f"RRTConnect {reason}；起点={a.tolist()}; 终点={b.tolist()}")
        points = [expand(state) for state in setup.getSolutionPath().getStates()]
    finally:
        # 解除 SI 对 Python validator 的引用，避免绑定对象循环持有。
        information.setMotionValidator(original_validator)

    deadline = monotonic() + SHORTCUT_SECONDS
    index = 0
    while index < len(points) - 2 and monotonic() < deadline:
        _cancelled(cancel)
        for target in range(len(points) - 1, index + 1, -1):
            _cancelled(cancel)
            if monotonic() >= deadline:
                break
            if model.edge_free(points[index], points[target], cancel):
                del points[index + 1 : target]
                break
        index += 1
    _cancelled(cancel)
    return points


def connect(
    model: ArmModel,
    a: np.ndarray,
    b: np.ndarray,
    active_groups: Iterable[str],
    seconds: float,
    cancel: Event,
) -> Trajectory | None:
    _cancelled(cancel)
    active = sorted({index for group in active_groups for index in GROUPS[group]})
    inactive = [index for index in range(17) if index not in active]
    if not np.array_equal(a[inactive], b[inactive]):
        raise PlanningError("连接改变了动作组之外的关节")
    for name, q in (("起点", a), ("终点", b)):
        _cancelled(cancel)
        if not model.valid(q):
            raise PlanningError(f"连接{name}无效：{model.last_error}; q={q.tolist()}")
    if np.array_equal(a, b):
        return None
    points = [a, b] if model.edge_free(a, b, cancel) else _rrt_path(model, a, b, active, cancel)
    _cancelled(cancel)
    return connection_curve(points, seconds)


def check_trajectory(model: ArmModel, trajectory: Trajectory, cancel: Event) -> None:
    polynomial = trajectory.polynomial
    degree = polynomial.c.shape[0] - 1
    for interval, (left, right) in enumerate(zip(polynomial.x, polynomial.x[1:])):
        _cancelled(cancel)
        width = float(right - left)
        coefficients = polynomial.c[:, interval, :]
        velocity = np.arange(degree, 0, -1)[:, None] * coefficients[:-1]
        acceleration = np.arange(degree - 1, 0, -1)[:, None] * velocity[:-1]
        max_velocity = 0.0
        for joint, (lower, upper) in enumerate(model.ranges):
            _cancelled(cancel)
            for local_time in _critical_times(velocity[:, joint], width):
                value = float(np.polyval(coefficients[:, joint], local_time))
                if not within_range(value, lower, upper):
                    q = np.asarray(polynomial(left + local_time))
                    raise PlanningError(
                        f"区间{interval} t={left + local_time:.9g} "
                        f"关节{UPPER_JOINTS[joint]}越界：{value:.9g} "
                        f"不在[{lower:.9g}, {upper:.9g}]; q={q.tolist()}"
                    )
            for local_time in _critical_times(acceleration[:, joint], width):
                value = abs(float(np.polyval(velocity[:, joint], local_time)))
                max_velocity = max(max_velocity, value)
        subdivisions = max(1, ceil(max_velocity * width / ANGULAR_RESOLUTION))
        for time in np.linspace(left, right, subdivisions + 1):
            _cancelled(cancel)
            q, _ = trajectory.sample(float(time))
            if not model.valid(q):
                raise PlanningError(f"区间{interval} t={time:.9g}无效：{model.last_error}; q={q.tolist()}")
    _cancelled(cancel)


def plan_sequence(
    actions: Sequence[Action],
    start: StartPose,
    model: ArmModel,
    speed: float,
    transition_seconds: float,
    cancel: Event,
) -> Trajectory:
    if not actions:
        raise ValueError("至少需要一个动作")
    parts: list[Trajectory | None] = []
    current = start.upper_q
    previous_group = actions[0].group
    for action in actions:
        _cancelled(cancel)
        curve = action_curve(action, start.upper_q, speed)
        parts.append(
            connect(
                model,
                current,
                curve.first_q,
                {previous_group, action.group},
                transition_seconds,
                cancel,
            )
        )
        parts.append(curve)
        current = curve.last_q
        previous_group = action.group
    parts.append(
        connect(
            model,
            current,
            start.upper_q,
            {previous_group},
            transition_seconds,
            cancel,
        )
    )
    trajectory = concatenate(parts)
    check_trajectory(model, trajectory, cancel)
    return trajectory
