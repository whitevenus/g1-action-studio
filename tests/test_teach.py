from g1_action_studio.constants import JOINT_NAMES
import json
import time
import unittest
import io
import signal
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from g1_action_studio.teach import SdkArmPort, load_profile, teach, validate_snapshot
from g1_action_studio.replay_sdk import ReplaySdkPort
from fake_sdk import SimulatedSDK, VirtualClock


# 仅用于假端口测试，不是 .165 的候选或已验收增益。
TEST_PROFILE = {"hold_kp": 12.0, "hold_kd": 1.2, "hand_hold_kp": 1.5, "hand_hold_kd": .2, "hand_teach_kp": 0.0, "hand_teach_kd": .05,
                "teach_kp": 2.0, "teach_kd": 0.6}


class FakeClock:
    def __init__(self):
        self.now = 100.0
        self.on_sleep = lambda: None

    def monotonic(self):
        return self.now

    def sleep(self, duration):
        self.now += duration
        self.on_sleep()


class FakePort:
    def __init__(self, clock):
        self.clock = clock
        self.ready = SimpleNamespace(wait=lambda timeout: True)
        self.writes = []
        self.snapshots = 0
        self.closed = False
        self.started = False
        self.fail_write = None
        self.invalid = None
        self.close_error = False

    def snapshot(self):
        self.snapshots += 1
        result = {"at": self.clock.now, "mode_pr": 0, "mode_machine": 5,
                  "motor_modes": (1,) * 14,
                  "q": (0.2 + self.snapshots * 0.0001,) * 28,
                  "dq": (0.02,) * 28}
        if self.invalid and len(self.writes) >= 2:
            key, value = self.invalid
            result[key] = value
        return result

    def check_services(self):
        return None

    def observed_commands(self):
        return 0

    def start_publisher(self):
        self.started = True

    def write(self, positions, hold_kp, hold_kd, teach_kp, teach_kd, side, weight, hand_kp=None, hand_kd=None):
        self.writes.append({"q": tuple(positions), "kp": teach_kp,
                            "kd": teach_kd, "weight": weight, "t": self.clock.now,
                            "hand_kp": hand_kp, "hand_kd": hand_kd})
        if len(self.writes) == self.fail_write:
            raise RuntimeError("模拟发送失败，可能已送达")

    def close(self):
        self.closed = True
        if self.close_error:
            raise RuntimeError("模拟关闭失败")


class TeachLifecycleTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.port = FakePort(self.clock)
        self.signals = {}
        self.stack = ExitStack()
        self.addCleanup(self.stack.close)
        module = "g1_action_studio.teach."
        self.stack.enter_context(patch(module + "load_profile", return_value=TEST_PROFILE))
        self.stack.enter_context(patch(module + "SdkArmPort", return_value=self.port))
        self.stack.enter_context(patch(module + "time.monotonic", self.clock.monotonic))
        self.stack.enter_context(patch(module + "time.sleep", self.clock.sleep))
        self.stack.enter_context(patch(module + "signal.getsignal", return_value="previous"))
        self.stack.enter_context(patch(module + "signal.signal", self.signals.__setitem__))
        self.stack.enter_context(patch.object(Path, "exists", return_value=False))
        self.output = io.StringIO()
        self.writer = self.stack.enter_context(patch.object(Path, "open"))
        self.writer.return_value.__enter__.return_value = self.output

    def run_teach(self, stage="hold", side="left", **timing):
        return teach("unused-offline", side, 1, "unused.json", "unused-profile.json",
                     execute=True, target_stage=stage, **timing)

    def test_custom_takeover_and_softening_durations_include_finger_gains(self):
        self.run_teach("soften", takeover_seconds=0.2, soften_seconds=0.4)
        takeover, soften = self.port.writes[:11], self.port.writes[11:32]
        self.assertEqual(len(self.port.writes), 11 + 21 + 101)
        self.assertAlmostEqual(takeover[-1]["t"] - takeover[0]["t"], 0.2)
        self.assertAlmostEqual(soften[-1]["t"] - soften[0]["t"], 0.4)
        self.assertEqual([takeover[0]["weight"], takeover[-1]["weight"]], [0, 1])
        self.assertTrue(all(w["weight"] == 1 for w in soften))
        self.assertEqual([soften[0]["kp"], soften[-1]["kp"]], [12, 2])
        self.assertEqual([soften[0]["hand_kp"], soften[-1]["hand_kp"]], [1.5, 0])
        self.assertAlmostEqual(soften[-1]["hand_kd"], .05)
        self.assertEqual(self.port.writes[-1]["hand_kp"], 1.5)
        self.assertEqual(self.port.writes[-1]["weight"], 0)

    def test_hold_uses_custom_takeover_but_does_not_soften(self):
        self.run_teach("hold", takeover_seconds=0.2, soften_seconds=0.4)
        self.assertEqual(len(self.port.writes), 11 + 51 + 101)
        self.assertTrue(all(w["kp"] == 12 and w["hand_kp"] == 1.5 for w in self.port.writes))

    def test_hold_and_release_targets_are_frozen_despite_measured_drift(self):
        result = self.run_teach()
        takeover = self.port.writes[:202]
        release = self.port.writes[202:]
        self.assertEqual(len(release), 101)
        self.assertEqual(len({w["q"] for w in takeover}), 1)
        self.assertEqual(len({w["q"] for w in release}), 1)
        self.assertNotEqual(takeover[0]["q"], release[0]["q"])
        self.assertEqual([takeover[0]["weight"], takeover[-1]["weight"]], [0, 1])
        self.assertEqual(release[-1]["weight"], 0)
        self.assertTrue(self.port.closed)
        self.assertFalse(result["recording_written"])
        self.assertEqual(result["trial_metrics"]["固定位置保持"]["samples"], 51)
        self.assertGreater(result["trial_metrics"]["固定位置保持"]["max_target_error_rad"], 0)
        self.writer.assert_not_called()
        self.assertEqual(self.signals[signal.SIGINT], "previous")
        self.assertEqual(self.signals[signal.SIGTERM], "previous")

    def test_failed_first_positive_send_still_attempts_release(self):
        self.port.fail_write = 2
        with self.assertRaisesRegex(RuntimeError, "模拟发送失败"):
            self.run_teach("hold")
        self.assertEqual(len(self.port.writes), 103)
        self.assertEqual(self.port.writes[2]["weight"], self.port.writes[1]["weight"])
        self.assertEqual(self.port.writes[-1]["weight"], 0)
        self.assertTrue(self.port.closed)
        self.writer.assert_not_called()

    def test_invalid_feedback_blocks_further_commands_and_preserves_both_errors(self):
        cases = (("at", 0.0), ("mode_machine", 4),
                 ("motor_modes", (0,) * 14), ("q", (float("nan"),) * 28),
                 ("dq", (float("inf"),) * 28), ("q", (0.0,)))
        for invalid in cases:
            with self.subTest(invalid=invalid):
                self.port.writes.clear()
                self.port.invalid = invalid
                with self.assertRaisesRegex(RuntimeError, "示教失败：.*释放失败"):
                    self.run_teach("hold")
                self.assertEqual(len(self.port.writes), 2)
                self.assertTrue(self.port.closed)
                self.writer.assert_not_called()

    def test_speed_and_offset_are_recorded_without_threshold_rejection(self):
        for field, value, metric in (("dq", (0.5,) * 28, "max_speed_rad_s"),
                                     ("q", (2.0,) * 28, "max_start_offset_rad")):
            with self.subTest(field=field):
                self.port.writes.clear()
                self.port.invalid = (field, value)
                result = self.run_teach()
                self.assertGreater(result["trial_metrics"]["固定位置保持"][metric], 0.4)
                self.assertEqual(self.port.writes[-1]["weight"], 0)

    def test_stop_releases_and_does_not_save(self):
        def stop():
            if len(self.port.writes) == 20:
                self.signals[signal.SIGINT](signal.SIGINT, None)
        self.clock.on_sleep = stop
        with self.assertRaisesRegex(RuntimeError, "操作员已请求停止"):
            self.run_teach("hold")
        self.assertEqual(self.port.writes[-1]["weight"], 0)
        self.assertTrue(self.port.closed)
        self.writer.assert_not_called()

    def test_release_send_failure_is_not_reported_as_success(self):
        self.port.fail_write = 203
        with self.assertRaisesRegex(RuntimeError, "释放失败"):
            self.run_teach()
        self.assertTrue(self.port.closed)
        self.writer.assert_not_called()

    def test_feedback_failure_during_release_stops_the_remaining_ramp(self):
        def invalidate():
            if len(self.port.writes) == 205:
                self.port.invalid = ("motor_modes", (0,) * 14)
        self.clock.on_sleep = invalidate
        with self.assertRaisesRegex(RuntimeError, "释放失败：手臂电机模式已改变"):
            self.run_teach()
        self.assertEqual(len(self.port.writes), 205)
        self.assertGreater(self.port.writes[-1]["weight"], 0)
        self.assertTrue(self.port.closed)
        self.writer.assert_not_called()

    def test_keyboard_interrupt_before_takeover_closes_without_sending(self):
        self.port.check_services = Mock(side_effect=KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            self.run_teach()
        self.assertFalse(self.port.started)
        self.assertFalse(self.port.writes)
        self.assertTrue(self.port.closed)

    def test_close_failure_preserves_primary_error(self):
        self.port.fail_write = 2
        self.port.close_error = True
        with self.assertRaisesRegex(RuntimeError, "模拟发送失败.*模拟关闭失败"):
            self.run_teach("hold")
        self.writer.assert_not_called()

    def test_preflight_failure_creates_no_publisher_or_commands(self):
        self.port.check_services = Mock(side_effect=RuntimeError("服务不符合条件"))
        with self.assertRaisesRegex(RuntimeError, "服务不符合条件"):
            self.run_teach()
        self.assertFalse(self.port.started)
        self.assertFalse(self.port.writes)
        self.assertTrue(self.port.closed)


    def test_preflight_checks_latest_feedback_once_after_observation(self):
        self.port.check_services = Mock()
        original_snapshot = self.port.snapshot
        checked_at = []

        def snapshot():
            checked_at.append(self.clock.now)
            return original_snapshot()

        self.port.snapshot = snapshot
        self.run_teach()
        self.port.check_services.assert_called_once()
        self.assertEqual(checked_at[0], 103.0)

    def test_mode_changed_during_observation_blocks_takeover(self):
        snapshot = self.port.snapshot

        def changed_mode():
            state = snapshot()
            state["motor_modes"] = (0,) * 14
            return state

        self.port.snapshot = changed_mode
        with self.assertRaisesRegex(RuntimeError, "电机模式"):
            self.run_teach()
        self.assertFalse(self.port.started)
        self.assertFalse(self.port.writes)


class PortCleanupTests(unittest.TestCase):
    def test_wire_command_keeps_body_slots_untouched_and_uses_separate_hand_commands(self):
        port = SdkArmPort.__new__(SdkArmPort)
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0]*28)
        ReplaySdkPort.__init__(port, sdk, clock)
        port.hand_profile = TEST_PROFILE
        port.start_publisher()
        port.write([0.25] * 28, 12, 1.2, 2, 0.6, "right", 0.5, 0, .05)
        command = sdk.writes[-1]
        self.assertEqual(command.motor_cmd[15].kp, 12)
        self.assertEqual(command.motor_cmd[22].kp, 2)
        self.assertEqual(command.motor_cmd[22].q, 0.25)
        self.assertEqual(command.motor_cmd[22].dq, 0)
        self.assertEqual(command.motor_cmd[22].tau, 0)
        self.assertEqual(command.motor_cmd[29].q, 0.5)
        self.assertEqual(command.motor_cmd[30].q, 0)
        self.assertEqual(command.crc, 1234)
        self.assertEqual(len(sdk.hand_writes), 2)
        self.assertEqual(sdk.hand_writes[0][1].motor_cmd[0].kp, 1.5)
        self.assertEqual(sdk.hand_writes[1][1].motor_cmd[0].kp, 0)
        port.close()

    def test_close_attempts_every_resource_even_if_publisher_close_fails(self):
        port = SdkArmPort.__new__(SdkArmPort)
        port.publisher = Mock()
        port.publisher.Close.side_effect = RuntimeError("发布者失败")
        arm, state = Mock(), Mock()
        port.arm_subscriber, port.subscriber = arm, state
        with self.assertRaisesRegex(RuntimeError, "发布者失败"):
            port.close()
        arm.Close.assert_called_once()
        state.Close.assert_called_once()
        port.close()

    def test_partial_initialization_is_closed(self):
        resource = Mock()
        def initialize(port, interface):
            port.subscriber = resource
            raise RuntimeError("初始化失败")
        with patch.object(SdkArmPort, "_initialize", initialize):
            with self.assertRaisesRegex(RuntimeError, "初始化失败"):
                SdkArmPort("unused")
        resource.Close.assert_called_once()

    def test_hold_profile_does_not_need_teaching_gains(self):
        data = {"schema": "g1-arm7-dex3-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-dex3-1", **TEST_PROFILE}
        del data["teach_kp"], data["teach_kd"]
        with patch.object(Path, "read_text", return_value=json.dumps(data)):
            profile = load_profile("unused", "hold")
            self.assertEqual(profile["teach_kp"], profile["hold_kp"])
            with self.assertRaisesRegex(ValueError, "teach_kp"):
                load_profile("unused", "soften")


class TeachTests(unittest.TestCase):
    def test_invalid_durations_fail_before_connecting(self):
        for name in ("takeover_seconds", "soften_seconds"):
            for value in (0, -1, .001, 31, float("nan"), float("inf"), True):
                with self.subTest(name=name, value=value), \
                        patch("g1_action_studio.teach.SdkArmPort") as sdk:
                    with self.assertRaisesRegex(ValueError, name):
                        teach("unused", "left", 1, "unused.json", "unused-profile.json",
                              execute=True, **{name: value})
                    sdk.assert_not_called()

    def test_cli_passes_timing_arguments(self):
        from g1_action_studio.cli import main
        with patch("g1_action_studio.teach.teach", return_value={}) as run, \
                patch("sys.stdout", io.StringIO()):
            self.assertEqual(main(["teach", "eth0", "--side", "left", "--stage", "soften",
                                   "--profile", "profile.json", "--output", "trial.json",
                                   "--takeover-seconds", "5", "--soften-seconds", "8"]), 0)
        self.assertEqual(run.call_args.kwargs, {"takeover_seconds": 5.0, "soften_seconds": 8.0})

    def test_preview_does_not_load_sdk_or_write_output(self):
        data = {
                "schema": "g1-arm7-dex3-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-dex3-1",
                "hold_kp": 12, "hold_kd": 1.2, "hand_hold_kp": 1.5, "hand_hold_kd": .2, "hand_teach_kp": 0.0, "hand_teach_kd": .05,
                "teach_kp": 2, "teach_kd": 0.6,
        }
        with patch.object(Path, "read_text", return_value=json.dumps(data)), \
                patch.object(Path, "exists", return_value=False), \
                patch("g1_action_studio.teach.SdkArmPort") as sdk, \
                patch.object(Path, "open") as writer:
            result = teach("eth0", "left", 10, "unused.json", "unused-profile.json")
            self.assertFalse(result["execute"])
            sdk.assert_not_called()
            writer.assert_not_called()

    def test_rejects_unbounded_profile(self):
        data = {
                "schema": "g1-arm7-dex3-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-dex3-1",
                "hold_kp": 100, "hold_kd": 1.2, "hand_hold_kp": 1.5, "hand_hold_kd": .2, "hand_teach_kp": 0.0, "hand_teach_kd": .05,
                "teach_kp": 2, "teach_kd": 0.6,
        }
        with patch.object(Path, "read_text", return_value=json.dumps(data)):
            with self.assertRaisesRegex(ValueError, "保持增益"):
                load_profile("unused-profile.json")

    def test_rejects_stale_or_changed_mode(self):
        snapshot = {"at": time.monotonic(), "mode_pr": 0,
                    "mode_machine": 5, "motor_modes": (1,) * 14,
                    "q": (0.0,) * 28, "dq": (0.0,) * 28}
        self.assertIs(validate_snapshot(snapshot), snapshot)
        snapshot["mode_machine"] = 4
        with self.assertRaisesRegex(RuntimeError, "模式"):
            validate_snapshot(snapshot, (1,) * 14)
        snapshot["mode_machine"] = 5
        snapshot["at"] -= 1
        with self.assertRaisesRegex(RuntimeError, "超时"):
            validate_snapshot(snapshot, (1,) * 14)


if __name__ == "__main__":
    unittest.main()
