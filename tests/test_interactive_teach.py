"""回车边界、终点保持、回位后释放；只用内存端口。"""

import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from fake_sdk import VirtualClock

from g1_action_studio.constants import DT, TEACH_SOFTEN_SECONDS, TEACH_TAKEOVER_SECONDS
from g1_action_studio.teach import teach

ROOT = Path(__file__).parents[1]


class InteractiveTeachTests(unittest.TestCase):
    def run_session(
        self,
        planning_error=None,
        eof=False,
        initial_collision=False,
        fault=None,
        close_error=False,
        arm_only=False,
        cancel=False,
    ):
        clock = VirtualClock()
        initial = json.loads((ROOT / "examples/transition/action-b.json").read_text())["samples"][0]["q"]
        if arm_only:
            initial = initial[:14]
        profile_path = ROOT / ("examples/profiles/teach-arm7.json" if arm_only else "examples/profiles/teach.json")
        q, writes = list(initial), []
        port = SimpleNamespace(
            wait_ready=lambda timeout, stop: True,
            check_services=Mock(),
            observed_commands=lambda: 0,
            start_publisher=lambda: None,
            close=lambda: None,
        )
        snapshot = lambda: {
            "at": clock.now,
            "q": tuple(q),
            "dq": (0.0,) * len(q),
            "motor_modes": (1,) * 14,
            "mode_pr": 0,
            "mode_machine": 5,
            "body_q": (0.0,) * 15,
        }

        def read_state():
            state = snapshot()
            if fault and len(writes) >= 2:
                if fault == "missing":
                    return None
                if fault == "send":
                    return state
                key, value = fault
                state[key] = value(clock) if callable(value) else value
            return state

        port.snapshot = read_state
        port.close = Mock(side_effect=RuntimeError("模拟关闭失败") if close_error else None)

        def write(target, hold_kp, hold_kd, weight, *, side, selected_kp, selected_kd, hand_kp=None, hand_kd=None):
            kp = selected_kp
            writes.append(
                {"q": list(target), "kp": kp, "hand_kp": hand_kp, "hand_kd": hand_kd, "weight": weight, "t": clock.now}
            )
            q[:] = target
            if fault == "send":
                return False
            return True

        port.write = write

        def keys(events, stop):
            began = clock.now
            sent = 0

            def on_sleep(_duration):
                nonlocal sent
                elapsed = clock.now - began
                if cancel and elapsed >= 0.4:
                    stop.set()
                if elapsed < 1.5:
                    q[0] += 0.0005
                    if not arm_only:
                        q[14] += 0.0005
                if elapsed >= 0.4 and sent == 0:
                    events.put(EOFError() if eof else "enter")
                    sent = 1
                elif elapsed >= 1.5 and sent == 1:
                    events.put("enter")
                    sent = 2

            clock.listeners.append(on_sleep)

        # 输入事件按虚拟时间产生；控制循环仍负责所有写入。
        class InputThread:
            def __init__(self, target, args, daemon):
                self.target, self.args = target, args

            def start(self):
                self.target(*self.args)

        def route(start, end, side, body, *, layout):
            if planning_error and list(start) != list(end):
                raise ValueError(planning_error)
            return [
                {"t": round(i * 0.02, 6), "q": [a + (b - a) * i / 50 for a, b in zip(start, end)]} for i in range(51)
            ]

        folder = tempfile.TemporaryDirectory(dir=ROOT / "tests")
        self.addCleanup(folder.cleanup)
        output = Path(folder.name) / "record.json"
        with (
            patch("g1_action_studio.teach.time", clock),
            patch("g1_action_studio.teach.create_sdk"),
            patch("g1_action_studio.teach.ReplaySdkPort", return_value=port),
            patch("g1_action_studio.teach.Thread", InputThread),
            patch("g1_action_studio.teach.read_record_keys", keys),
            patch("g1_action_studio.teach.plan_return", route),
            patch("g1_action_studio.collision.CollisionModel") as model,
            patch("g1_action_studio.teach.signal.signal"),
            patch("sys.stdout", io.StringIO()),
        ):
            model.return_value.clearance.return_value = {"valid": not initial_collision}
            if initial_collision:
                with self.assertRaisesRegex(ValueError, "初始姿态碰撞或余量不足"):
                    teach("unused", "left", output, profile_path, True)
                self.assertEqual(writes, [])
                self.assertFalse(output.exists())
                return
            if planning_error or eof or fault or cancel:
                expected = "发送失败.*模拟关闭失败" if close_error else "未自动释放"
                with self.assertRaisesRegex(RuntimeError, expected):
                    teach("unused", "left", output, profile_path, True)
                self.assertFalse(output.exists())
                if fault != "send":
                    self.assertGreater(writes[-1]["weight"], 0)
                port.close.assert_called_once()
                return
            result = teach("unused", "left", output, profile_path, True)
        port.check_services.assert_called_once()
        return initial, writes, result, json.loads(output.read_text(encoding="utf-8"))

    def test_manual_boundaries_freeze_endpoint_restore_return_then_release(self):
        initial, writes, result, data = self.run_session()
        self.assertTrue(result["release_sent"])
        self.assertAlmostEqual(data["samples"][0]["t"], 0)
        self.assertTrue(1.0 <= result["duration_s"] <= 1.2)
        end = data["samples"][-1]["q"]
        self.assertNotEqual(end, initial)
        self.assertNotEqual(end[14:21], initial[14:21])
        self.assertTrue(any(w["hand_kp"] == 0.0 and w["hand_kd"] == 0.05 for w in writes))
        restored = [w for w in writes if w["t"] > 8 and w["kp"] > 20 and w["q"] == end]
        self.assertTrue(restored)
        self.assertTrue(any(w["q"] == end and w["hand_kp"] == 1.5 for w in restored))
        releasing = [w for w in writes if w["t"] > 8 and w["weight"] < 1]
        self.assertTrue(releasing)
        self.assertTrue(all(w["q"] == initial for w in releasing))
        self.assertEqual(writes[-1]["weight"], 0)
        self.assertEqual(data["baseline_q"], initial)

    def test_failed_return_planning_does_not_release_at_endpoint(self):
        self.run_session(planning_error="无回位路径")

    def test_takeover_and_softening_each_last_two_seconds(self):
        initial, writes, result, data = self.run_session()
        count = round(TEACH_TAKEOVER_SECONDS / DT) + 1
        takeover, soften = writes[:count], writes[count : count + round(TEACH_SOFTEN_SECONDS / DT) + 1]
        self.assertAlmostEqual(takeover[-1]["t"] - takeover[0]["t"], 2)
        self.assertAlmostEqual(soften[-1]["t"] - soften[0]["t"], 2)
        self.assertEqual([takeover[0]["weight"], takeover[-1]["weight"]], [0, 1])
        self.assertTrue(all(w["q"] == initial for w in takeover))
        self.assertTrue(all(w["weight"] == 1 for w in soften))
        self.assertEqual(soften[-1]["hand_kp"], 0)
        self.assertTrue(1.0 <= result["duration_s"] <= 1.2)
        self.assertEqual(writes[-1]["q"], initial)
        self.assertEqual(writes[-1]["weight"], 0)

    def test_runtime_feedback_failures_stop_without_release_or_file(self):
        for fault in (
            "missing",
            ("at", lambda clock: clock.now - 2),
            ("mode_machine", 4),
            ("motor_modes", (0,) * 14),
            ("q", (float("nan"),) * 28),
            ("dq", (float("inf"),) * 28),
            ("q", (0.0,)),
            ("q", (4.0,) * 28),
        ):
            with self.subTest(fault=fault):
                self.run_session(fault=fault)

    def test_first_send_failure_is_not_treated_as_no_control(self):
        self.run_session(fault="send")

    def test_cleanup_failure_preserves_send_error(self):
        self.run_session(fault="send", close_error=True)

    def test_input_eof_is_not_treated_as_recording_finished(self):
        self.run_session(eof=True)

    def test_initial_collision_blocks_takeover(self):
        self.run_session(initial_collision=True)

    def test_arm_only_recording_and_failure_paths(self):
        initial, writes, result, data = self.run_session(arm_only=True)
        self.assertEqual(len(initial), 14)
        self.assertFalse(result["hand_control_enabled"])
        self.assertTrue(all(row["hand_kp"] is None for row in writes))
        self.assertTrue(all(len(row["q"]) == 14 for row in data["samples"]))
        for arguments in (
            {"eof": True},
            {"planning_error": "无回位路径"},
            {"fault": ("at", lambda clock: clock.now - 2)},
            {"cancel": True},
        ):
            with self.subTest(arguments=arguments):
                self.run_session(arm_only=True, **arguments)
