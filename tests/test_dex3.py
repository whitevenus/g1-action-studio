"""Dex3真实关节顺序、断流和手指轨迹的边界检查。"""

import json
import tempfile
import unittest
from pathlib import Path
from threading import Event

from fake_sdk import SIMULATION_PROFILE, SimulatedSDK, VirtualClock
from test_executor import make_plan

from g1_action_studio.author import load_action
from g1_action_studio.executor import execute_replay
from g1_action_studio.replay_sdk import ReplaySdkPort
from g1_action_studio.trajectory import frame_digest

ROOT = Path(__file__).parents[1]


class Dex3Tests(unittest.TestCase):
    def test_feedback_and_command_right_index_middle_order(self):
        clock = VirtualClock()
        q = [i * 0.01 for i in range(28)]
        sdk = SimulatedSDK(clock, q)
        port = ReplaySdkPort(sdk, clock)
        self.addCleanup(port.close)
        self.assertEqual(list(port.snapshot()["q"]), q)
        port.hand_profile = SIMULATION_PROFILE
        port.start_publisher()
        self.assertTrue(port.write(q, 12, 1.2, 0.5))
        right = sdk.hand_writes[-1][1].motor_cmd
        self.assertEqual(right[3].q, q[26])  # 硬件3是食指0，模型中排在右中指之后。
        self.assertEqual(right[5].q, q[24])
        self.assertEqual([m.mode for m in right], [16 + i for i in range(7)])
        self.assertTrue(all(m.kp == 1.5 and m.kd == 0.2 for m in right))

    def test_missing_hand_never_becomes_ready_or_starts_publisher(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        sdk.drop_hands.add("right")
        port = ReplaySdkPort(sdk, clock)
        self.addCleanup(port.close)
        self.assertFalse(port.wait_ready(0.1, Event()))
        self.assertIsNone(port.snapshot())
        self.assertFalse(sdk.publisher_started)

    def test_one_hand_stale_cannot_be_masked_by_lowstate(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)

        def drop():
            if clock.now > 3.3:
                sdk.drop_hands.add("left")

        sdk.before_step = drop
        report = execute_replay(make_plan(), SIMULATION_PROFILE, lambda: ReplaySdkPort(sdk, clock), clock, Event())
        self.assertEqual(report["status"], "failed")
        self.assertIn("状态超时", report["errors"][0]["message"])
        self.assertFalse(report["release_sent"])
        self.assertTrue(all(r.closed for r in sdk.resources))

    def test_invalid_hand_clears_old_feedback(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        port = ReplaySdkPort(sdk, clock)
        self.addCleanup(port.close)
        sdk.q[14] = float("nan")
        sdk._publish_state()
        self.assertIsNone(port.snapshot())

    def test_failed_hand_write_is_not_reported_as_success(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        port = ReplaySdkPort(sdk, clock)
        self.addCleanup(port.close)
        port.hand_profile = SIMULATION_PROFILE
        port.start_publisher()
        port.hand_publishers["right"].Write = lambda command: False
        with self.assertRaisesRegex(RuntimeError, "部分命令可能已送达"):
            port.write([0.0] * 28, 12, 1.2, 1.0)

    def test_incomplete_target_sends_nothing_even_on_arm_channel(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        port = ReplaySdkPort(sdk, clock)
        self.addCleanup(port.close)
        port.hand_profile = SIMULATION_PROFILE
        port.start_publisher()
        with self.assertRaisesRegex(ValueError, "28"):
            port.write([0.0] * 14, 12, 1.2, 1.0)
        self.assertFalse(sdk.writes)
        self.assertFalse(sdk.hand_writes)

    def test_hand_failure_at_zero_arm_weight_still_requires_operator(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)

        def factory():
            port = ReplaySdkPort(sdk, clock)
            start = port.start_publisher

            def start_and_fail():
                start()
                port.hand_publishers["right"].Write = lambda command: False

            port.start_publisher = start_and_fail
            return port

        report = execute_replay(make_plan(), SIMULATION_PROFILE, factory, clock, Event())
        self.assertEqual(sdk.writes[-1].motor_cmd[29].q, 0)
        self.assertFalse(report["release_sent"])
        self.assertTrue(report["requires_operator"])
        self.assertFalse(report["hand_release_sent"])

    def test_finger_trajectory_replays_and_returns_to_original_hand_shape(self):
        plan = make_plan()
        for frame in plan["frames"]:
            if frame["phase"] == "replay":
                frame["q"][14] = frame["q"][3]
        plan["schedule_sha256"] = frame_digest(plan["frames"])
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        report = execute_replay(plan, SIMULATION_PROFILE, lambda: ReplaySdkPort(sdk, clock), clock, Event())
        self.assertEqual(report["status"], "completed")
        left = [msg for side, msg in sdk.hand_writes if side == "left"]
        self.assertGreater(max(msg.motor_cmd[0].q for msg in left), 0.09)
        self.assertEqual([m.q for m in left[-1].motor_cmd], [0.0] * 7)

    def test_old_arm_only_recording_is_rejected_without_zero_filling(self):
        data = json.loads((ROOT / "examples/synthetic-recording.json").read_text())
        data["schema"] = "g1-arm7-state-recording-v1"
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            path = Path(folder) / "old.json"
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "不支持"):
                load_action(path)
