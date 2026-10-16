from collections.abc import Iterable, Sequence
from threading import Event
from time import monotonic

import numpy as np
from scipy.interpolate import BPoly, PPoly

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
    lower, upper = model.ranges.T
    for begin in range(0, len(polynomial.x) - 1, 512):
        _cancelled(cancel)
        end = min(begin + 512, len(polynomial.x) - 1)
        widths = np.diff(polynomial.x[begin : end + 1])
        coefficients = polynomial.c[:, begin:end]
        # 换基后曲线位于控制点凸包内，导数控制点给出速度上界。
        batch = PPoly(coefficients, polynomial.x[begin : end + 1], extrapolate=False)
        control = BPoly.from_power_basis(batch).c
        ambiguous = (control.min(axis=0) < lower) | (control.max(axis=0) > upper)
        # 凸包只能快速确认合法；包络触及范围外时仍求精确极值，不据此误拒动作。
        for offset, joint in np.argwhere(ambiguous):
            _cancelled(cancel)
            values = coefficients[:, offset, joint]
            velocity = np.arange(degree, 0, -1) * values[:-1]
            for local_time in _critical_times(velocity, float(widths[offset])):
                value = float(np.polyval(values, local_time))
                if not within_range(value, lower[joint], upper[joint]):
                    interval = begin + offset
                    time = polynomial.x[interval] + local_time
                    q = np.asarray(polynomial(time))
                    raise PlanningError(
                        f"区间{interval} t={time:.9g} "
                        f"关节{UPPER_JOINTS[joint]}越界：{value:.9g} "
                        f"不在[{lower[joint]:.9g}, {upper[joint]:.9g}]; q={q.tolist()}"
                    )
        # width * max|dq/dt| 的保守上界；仍覆盖全部 knots，并对较大运动加密。
        travel = degree * np.max(np.abs(np.diff(control, axis=0)), axis=(0, 2))
        subdivisions = np.maximum(1, np.ceil(travel / ANGULAR_RESOLUTION)).astype(np.intp)
        times = [polynomial.x[begin + 1 : end + 1]]
        if begin == 0:
            times.append(polynomial.x[:1])
        for offset in np.flatnonzero(subdivisions > 1):
            interval = begin + offset
            times.append(
                np.linspace(polynomial.x[interval], polynomial.x[interval + 1], subdivisions[offset] + 1)[1:-1]
            )
        times = np.sort(np.concatenate(times))
        positions = polynomial(times)
        if begin == 0:
            positions[0] = trajectory.first_q
        if end == len(polynomial.x) - 1:
            positions[-1] = trajectory.last_q
        for time, q in zip(times, positions):
            _cancelled(cancel)
            if not model.valid(q):
                interval = min(
                    len(polynomial.x) - 2, max(0, int(np.searchsorted(polynomial.x, time, side="right") - 1))
                )
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
