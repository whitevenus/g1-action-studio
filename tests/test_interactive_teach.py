"""回车边界、终点保持、回位后释放；只用内存端口。"""
from g1_action_studio.constants import JOINT_NAMES
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from g1_action_studio.teach import teach
from fake_sdk import VirtualClock

ROOT = Path(__file__).parents[1]


class InteractiveTeachTests(unittest.TestCase):
    def run_session(self, planning_error=None, eof=False, initial_collision=False):
        clock = VirtualClock()
        initial = json.loads((ROOT / "examples/transition/action-b.json").read_text())["samples"][0]["q"]
        q, writes = list(initial), []
        port = SimpleNamespace(ready=SimpleNamespace(wait=lambda _: True),
                               check_services=Mock(), observed_commands=lambda: 0,
                               start_publisher=lambda: None, close=lambda: None)
        port.snapshot = lambda: {"at": clock.now, "q": tuple(q), "dq": (0.0,)* 28,
                                 "motor_modes": (1,)* 14, "mode_pr": 0, "mode_machine": 5,
                                 "body_q": (0.0,)*15}
        def write(target, hold_kp, hold_kd, kp, kd, side, weight, hand_kp=None, hand_kd=None):
            writes.append({"q": list(target), "kp": kp, "hand_kp": hand_kp,
                           "hand_kd": hand_kd, "weight": weight, "t": clock.now})
            q[:] = target
        port.write = write
        def keys(events, stop):
            began = clock.now
            sent = 0
            def on_sleep(_duration):
                nonlocal sent
                elapsed = clock.now-began
                if elapsed < 1.5:
                    q[0] += 0.0005
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
        def route(start, end, side, body):
            if planning_error and list(start) != list(end):
                raise ValueError(planning_error)
            return [{"t": round(i*.02, 6), "q": [a+(b-a)*i/50 for a,b in zip(start,end)]}
                    for i in range(51)]
        folder = tempfile.TemporaryDirectory(dir=ROOT/'tests')
        self.addCleanup(folder.cleanup)
        output = Path(folder.name)/'record.json'
        with patch("g1_action_studio.teach.time", clock), \
                patch("g1_action_studio.teach.SdkArmPort", return_value=port), \
                patch("g1_action_studio.teach.Thread", InputThread), \
                patch("g1_action_studio.teach.read_record_keys", keys), \
                patch("g1_action_studio.teach.plan_return", route), \
                patch("g1_action_studio.collision.CollisionModel") as model, \
                patch("g1_action_studio.teach.signal.signal"), \
                patch("sys.stdout", io.StringIO()):
            model.return_value.clearance.return_value = {"valid": not initial_collision}
            if initial_collision:
                with self.assertRaisesRegex(ValueError, "初始姿态碰撞或余量不足"):
                    teach("unused", "left", 3, output, ROOT/'examples/profiles/soften.json', True)
                self.assertEqual(writes, [])
                self.assertFalse(output.exists())
                return
            if planning_error or eof:
                with self.assertRaisesRegex(RuntimeError, "未自动释放"):
                    teach("unused", "left", 3, output, ROOT/'examples/profiles/soften.json', True)
                self.assertFalse(output.exists())
                self.assertGreater(writes[-1]["weight"], 0)
                return
            result = teach("unused", "left", 3, output, ROOT/'examples/profiles/soften.json', True)
        port.check_services.assert_called_once()
        return initial, writes, result, json.loads(output.read_text(encoding='utf-8'))

    def test_manual_boundaries_freeze_endpoint_restore_return_then_release(self):
        initial, writes, result, data = self.run_session()
        self.assertTrue(result['release_sent'])
        self.assertAlmostEqual(data['samples'][0]['t'], 0)
        self.assertTrue(1.0 <= result['duration_s'] <= 1.2)
        end = data['samples'][-1]['q']
        self.assertNotEqual(end, initial)
        self.assertNotEqual(end[14:21], initial[14:21])
        self.assertTrue(any(w['hand_kp'] == 0.0 and w['hand_kd'] == .05 for w in writes))
        restored = [w for w in writes if w['t'] > 10 and w['kp'] > 20 and w['q'] == end]
        self.assertTrue(restored)
        self.assertTrue(any(w['q'] == end and w['hand_kp'] == 1.5 for w in restored))
        releasing = [w for w in writes if w['t'] > 10 and w['weight'] < 1]
        self.assertTrue(releasing)
        self.assertTrue(all(w['q'] == initial for w in releasing))
        self.assertEqual(writes[-1]['weight'], 0)
        self.assertEqual(data['baseline_q'], initial)

    def test_failed_return_planning_does_not_release_at_endpoint(self):
        self.run_session(planning_error='无回位路径')

    def test_input_eof_is_not_treated_as_recording_finished(self):
        self.run_session(eof=True)

    def test_initial_collision_blocks_takeover(self):
        self.run_session(initial_collision=True)
