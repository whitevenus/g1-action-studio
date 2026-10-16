"""精简动作文件与统一的上肢 17 维位置、速度曲线。"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy.interpolate import CubicSpline, PPoly

from .model import GROUPS


def _readonly(values):
    array = np.array(values, dtype=np.float64, copy=True)
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class Action:
    group: str
    t: np.ndarray
    q: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "t", _readonly(self.t))
        object.__setattr__(self, "q", _readonly(self.q))


@dataclass(frozen=True)
class StartPose:
    leg_q: np.ndarray
    upper_q: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "leg_q", _readonly(self.leg_q))
        object.__setattr__(self, "upper_q", _readonly(self.upper_q))


@dataclass(frozen=True)
class Trajectory:
    polynomial: PPoly
    duration: float
    first_q: np.ndarray
    last_q: np.ndarray

    def __post_init__(self):
        self.polynomial.c.setflags(write=False)
        self.polynomial.x.setflags(write=False)
        object.__setattr__(self, "first_q", _readonly(self.first_q))
        object.__setattr__(self, "last_q", _readonly(self.last_q))

    def sample(self, t):
        if t <= 0:
            return self.first_q.copy(), np.zeros(17)
        if t >= self.duration:
            return self.last_q.copy(), np.zeros(17)
        return self.polynomial(t), self.polynomial(t, 1)


def _keys(value, expected, label):
    if not isinstance(value, dict) or value.keys() != set(expected):
        raise ValueError(f"{label}必须且只能包含 {', '.join(expected)}")


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} 必须是有限数字")
    try:
        result = float(value)
    except OverflowError as error:
        raise ValueError(f"{label} 必须是有限数字") from error
    if not np.isfinite(result):
        raise ValueError(f"{label} 必须是有限数字")
    return result


def _vector(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} 必须包含 {length} 个数字")
    return [_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def load_action(path) -> Action:
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as source:
            data = json.load(source)
        _keys(data, ("group", "frames"), "动作")
        group = data["group"]
        if not isinstance(group, str) or group not in GROUPS:
            raise ValueError(f"group 必须是 {', '.join(GROUPS)} 之一")
        frames = data["frames"]
        if not isinstance(frames, list) or len(frames) < 2:
            raise ValueError("frames 至少需要两个采样帧")
        times, positions = [], []
        for index, frame in enumerate(frames):
            label = f"frames[{index}]"
            _keys(frame, ("t", "q"), label)
            times.append(_number(frame["t"], f"{label}.t"))
            positions.append(_vector(frame["q"], len(GROUPS[group]), f"{label}.q"))
        if times[0] != 0 or np.any(np.diff(times) <= 0):
            raise ValueError("动作时间必须从 0 开始并严格递增")
        return Action(group, times, positions)
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from error


def save_action(action, path) -> None:
    data = {
        "group": action.group,
        "frames": [{"t": float(t), "q": q.tolist()} for t, q in zip(action.t, action.q)],
    }
    with Path(path).open("x", encoding="utf-8", newline="\n") as output:
        json.dump(data, output, ensure_ascii=False, allow_nan=False, indent=2)
        output.write("\n")


def load_start(path) -> StartPose:
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as source:
            data = json.load(source)
        _keys(data, ("leg_q", "upper_q"), "启动姿态")
        return StartPose(
            _vector(data["leg_q"], 12, "leg_q"),
            _vector(data["upper_q"], 17, "upper_q"),
        )
    except ValueError as error:
        raise ValueError(f"{path}: {error}") from error


def action_curve(action, start_upper, speed) -> Trajectory:
    full_q = np.broadcast_to(np.asarray(start_upper, dtype=np.float64), (len(action.t), 17)).copy()
    full_q[:, GROUPS[action.group]] = action.q
    spline = CubicSpline(action.t / speed, full_q, axis=0, bc_type="clamped", extrapolate=False)
    coefficients = np.pad(spline.c, ((2, 0), (0, 0), (0, 0)))
    polynomial = PPoly(coefficients, spline.x, extrapolate=False)
    return Trajectory(polynomial, float(spline.x[-1]), full_q[0], full_q[-1])


def connection_curve(points17, seconds) -> Trajectory | None:
    points = np.asarray(points17, dtype=np.float64)
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    points = points[np.r_[True, lengths != 0]]
    if len(points) < 2:
        return None
    delta = np.diff(points, axis=0)
    lengths = np.linalg.norm(delta, axis=1)
    knots = np.r_[0.0, np.cumsum(seconds * lengths / lengths.sum())]
    knots[-1] = seconds
    times = np.diff(knots)[:, None]
    zeros = np.zeros_like(delta)
    coefficients = np.stack(
        (
            6 * delta / times**5,
            -15 * delta / times**4,
            10 * delta / times**3,
            zeros,
            zeros,
            points[:-1],
        )
    )
    polynomial = PPoly(coefficients, knots, extrapolate=False)
    return Trajectory(polynomial, float(seconds), points[0], points[-1])


def concatenate(parts: Iterable[Trajectory | None]) -> Trajectory:
    curves = [part for part in parts if part is not None]
    if not curves:
        raise ValueError("动作序列至少需要一段轨迹")
    coefficients, knots = [], [np.array([0.0])]
    offset = 0.0
    for curve in curves:
        coefficients.append(curve.polynomial.c)
        knots.append(curve.polynomial.x[1:] + offset)
        offset += curve.duration
    polynomial = PPoly(np.concatenate(coefficients, axis=1), np.concatenate(knots), extrapolate=False)
    return Trajectory(polynomial, offset, curves[0].first_q, curves[-1].last_q)
