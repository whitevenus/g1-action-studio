from collections.abc import Iterable, Sequence
from math import ceil
from threading import Event

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


def connect(
    model: ArmModel,
    a: np.ndarray,
    b: np.ndarray,
    active_groups: Iterable[str],
    seconds: float,
    cancel: Event,
) -> Trajectory | None:
    _cancelled(cancel)
    active = {index for group in active_groups for index in GROUPS[group]}
    inactive = [index for index in range(17) if index not in active]
    if not np.array_equal(a[inactive], b[inactive]):
        raise PlanningError("连接改变了动作组之外的关节")
    for name, q in (("起点", a), ("终点", b)):
        _cancelled(cancel)
        if not model.valid(q):
            raise PlanningError(f"连接{name}无效：{model.last_error}; q={q.tolist()}")
    if np.array_equal(a, b):
        return None
    if not model.edge_free(a, b, cancel):
        _cancelled(cancel)
        raise PlanningError(f"直连受阻：{model.last_error}; 起点={a.tolist()}; 终点={b.tolist()}")
    _cancelled(cancel)
    return connection_curve([a, b], seconds)


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
