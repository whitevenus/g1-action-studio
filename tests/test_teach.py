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


# 仅用于假端口测试，不是 .165 的候选或已验收增益。
TEST_PROFILE = {"hold_kp": 12.0, "hold_kd": 1.2,
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
                  "q": (0.2 + self.snapshots * 0.0001,) * 14,
                  "dq": (0.02,) * 14}
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

    def write(self, positions, hold_kp, hold_kd, teach_kp, teach_kd, side, weight):
        self.writes.append({"q": tuple(positions), "kp": teach_kp,
                            "kd": teach_kd, "weight": weight})
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

    def run_teach(self, stage="hold", side="left"):
        return teach("unused-offline", side, 1, "unused.json", "unused-profile.json",
                     execute=True, target_stage=stage)

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

    def test_record_follows_only_selected_side_and_freezes_both_on_release(self):
        for side, selected, held in (("left", 0, 7), ("right", 7, 0)):
            with self.subTest(side=side):
                self.port.writes.clear()
                result = self.run_teach("record", side)
                initial = self.port.writes[0]["q"]
                soften = self.port.writes[151:302]
                recording = self.port.writes[302:353]
                release = self.port.writes[353:]
                self.assertNotEqual(recording[0]["q"][selected], recording[-1]["q"][selected])
                self.assertTrue(all(w["q"][held] == initial[held] for w in soften + recording))
                self.assertEqual(len({w["q"] for w in release}), 1)
                self.assertEqual(release[0]["kp"], TEST_PROFILE["teach_kp"])
                self.assertEqual(release[50]["kp"], TEST_PROFILE["hold_kp"])
                self.assertEqual(release[50]["weight"], 1)
                self.assertEqual(release[-1]["weight"], 0)
                self.assertEqual(result["samples"], 51)
                payload = json.loads(self.output.getvalue())
                self.assertEqual(payload["qualification"], "raw_observation_not_executable")
                self.output.seek(0)
                self.output.truncate()

    def test_failed_first_positive_send_still_attempts_release(self):
        self.port.fail_write = 2
        with self.assertRaisesRegex(RuntimeError, "模拟发送失败"):
            self.run_teach("record")
        self.assertEqual(len(self.port.writes), 103)
        self.assertEqual(self.port.writes[2]["weight"], self.port.writes[1]["weight"])
        self.assertEqual(self.port.writes[-1]["weight"], 0)
        self.assertTrue(self.port.closed)
        self.writer.assert_not_called()

    def test_invalid_feedback_blocks_further_commands_and_preserves_both_errors(self):
        cases = (("at", 0.0), ("mode_machine", 4),
                 ("motor_modes", (0,) * 14), ("q", (float("nan"),) * 14),
                 ("dq", (float("inf"),) * 14), ("q", (0.0,)))
        for invalid in cases:
            with self.subTest(invalid=invalid):
                self.port.writes.clear()
                self.port.invalid = invalid
                with self.assertRaisesRegex(RuntimeError, "示教失败：.*释放失败"):
                    self.run_teach("record")
                self.assertEqual(len(self.port.writes), 2)
                self.assertTrue(self.port.closed)
                self.writer.assert_not_called()

    def test_speed_and_offset_are_recorded_without_threshold_rejection(self):
        for field, value, metric in (("dq", (0.5,) * 14, "max_speed_rad_s"),
                                     ("q", (2.0,) * 14, "max_start_offset_rad")):
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
            self.run_teach("record")
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
            self.run_teach("record")
        self.writer.assert_not_called()

    def test_preflight_failure_creates_no_publisher_or_commands(self):
        self.port.check_services = Mock(side_effect=RuntimeError("服务不符合条件"))
        with self.assertRaisesRegex(RuntimeError, "服务不符合条件"):
            self.run_teach()
        self.assertFalse(self.port.started)
        self.assertFalse(self.port.writes)
        self.assertTrue(self.port.closed)


class PortCleanupTests(unittest.TestCase):
    def test_wire_command_changes_arm_slots_and_weight_without_hand_commands(self):
        port = SdkArmPort.__new__(SdkArmPort)
        command = SimpleNamespace(motor_cmd=[SimpleNamespace(q=0, dq=0, tau=0,
                                  kp=0, kd=0) for _ in range(35)])
        port.command_type = lambda: command
        port.crc = SimpleNamespace(Crc=lambda msg: 1234)
        port.publisher = Mock()
        port.publisher.Write.return_value = True
        port.write([0.25] * 14, 12, 1.2, 2, 0.6, "right", 0.5)
        self.assertEqual(command.motor_cmd[15].kp, 12)
        self.assertEqual(command.motor_cmd[22].kp, 2)
        self.assertEqual(command.motor_cmd[22].q, 0.25)
        self.assertEqual(command.motor_cmd[22].dq, 0)
        self.assertEqual(command.motor_cmd[22].tau, 0)
        self.assertEqual(command.motor_cmd[29].q, 0.5)
        self.assertEqual(command.motor_cmd[30].q, 0)
        self.assertEqual(command.crc, 1234)
        port.publisher.Write.assert_called_once_with(command)

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
        data = {"schema": "g1-arm7-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-29dof", **TEST_PROFILE}
        del data["teach_kp"], data["teach_kd"]
        with patch.object(Path, "read_text", return_value=json.dumps(data)):
            profile = load_profile("unused", "hold")
            self.assertEqual(profile["teach_kp"], profile["hold_kp"])
            with self.assertRaisesRegex(ValueError, "teach_kp"):
                load_profile("unused", "soften")


class TeachTests(unittest.TestCase):
    def test_preview_does_not_load_sdk_or_write_output(self):
        data = {
                "schema": "g1-arm7-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-29dof",
                "hold_kp": 12, "hold_kd": 1.2,
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
                "schema": "g1-arm7-teach-profile-v1",
                "target_profile": "unitree-g1-arm7-29dof",
                "hold_kp": 100, "hold_kd": 1.2,
                "teach_kp": 2, "teach_kd": 0.6,
        }
        with patch.object(Path, "read_text", return_value=json.dumps(data)):
            with self.assertRaisesRegex(ValueError, "保持增益"):
                load_profile("unused-profile.json")

    def test_rejects_stale_or_changed_mode(self):
        snapshot = {"at": time.monotonic(), "mode_pr": 0,
                    "mode_machine": 5, "motor_modes": (1,) * 14,
                    "q": (0.0,) * 14, "dq": (0.0,) * 14}
        validate_snapshot(snapshot, (1,) * 14)
        snapshot["mode_machine"] = 4
        with self.assertRaisesRegex(RuntimeError, "模式"):
            validate_snapshot(snapshot, (1,) * 14)
        snapshot["mode_machine"] = 5
        snapshot["at"] -= 1
        with self.assertRaisesRegex(RuntimeError, "超时"):
            validate_snapshot(snapshot, (1,) * 14)


if __name__ == "__main__":
    unittest.main()
