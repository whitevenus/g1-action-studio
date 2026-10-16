import copy
import math
import sys
import unittest
from threading import Event
from unittest.mock import Mock, patch

from fake_sdk import SIMULATION_PROFILE, SimulatedSDK, VirtualClock, simulate_replay

from g1_action_studio.constants import ARM_SLOTS, COLLISION_MODEL, JOINT_NAMES, PROFILE
from g1_action_studio.executor import execute_replay
from g1_action_studio.replay import build_schedule, load_limits
from g1_action_studio.replay_sdk import ReplaySdkPort
from g1_action_studio.trajectory import frame_digest


def make_plan():
    source = [{"t": round(i * 0.02, 6), "q": [0.0] * 28} for i in range(101)]
    for i, frame in enumerate(source):
        frame["q"][3] = 0.1 * math.sin(math.pi * i / 100) ** 2
    frames, phases = build_schedule(source, [0.0] * 28, takeover=0.5, enter=1, return_seconds=1, release=0.5)
    return {
        "schema": "g1-arm7-dex3-offline-replay-plan-v1",
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "hardware_enabled": False,
        "qualification": "offline_only_not_executable",
        "status": "offline_numerical_pass",
        "baseline": {"q": [0.0] * 28},
        "model": {"limits_rad": load_limits(COLLISION_MODEL)},
        "thresholds": {"max_velocity_rad_s": 1, "max_acceleration_rad_s2": 20},
        "frames": frames,
        "phases": phases,
        "schedule_sha256": frame_digest(frames),
    }


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.plan = make_plan()
        self.profile = dict(SIMULATION_PROFILE)
        self.clock, self.stop = VirtualClock(), Event()
        self.sdk = SimulatedSDK(self.clock, [0.0] * 28)
        self.port = None

    def run_executor(self):
        def factory():
            self.port = ReplaySdkPort(self.sdk, self.clock)
            return self.port

        result = execute_replay(self.plan, self.profile, factory, self.clock, self.stop)
        self.assertTrue(all(r.closed for r in self.sdk.resources))
        return result

    def test_normal_motion_has_distinct_feedback_and_settles_before_release(self):
        with patch.dict(sys.modules, {"unitree_sdk2py": None}):
            result = self.run_executor()
        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["release_sent"])
        self.assertFalse(result["handover_confirmed"])
        self.assertGreater(result["metrics"]["max_tracking_error_rad"], 0)
        samples = result["samples"]
        self.assertTrue(any(s["target_q"] != s["q"] for s in samples))
        phases = [s["phase"] for s in samples]
        self.assertLess(phases.index("settle"), phases.index("release"))
        self.assertEqual(samples[-1]["weight"], 0)
        self.assertEqual(samples[-1]["target_q"], [0.0] * 28)
        self.assertLess(max(map(abs, self.sdk.q)), self.profile["return_tolerance_rad"])

    def test_rejects_plan_before_creating_sdk_port(self):
        for change in (
            {"status": "offline_numerical_rejected"},
            {"hardware_enabled": True},
            {"schedule_sha256": "tampered"},
        ):
            factory = Mock()
            with self.assertRaises(ValueError):
                execute_replay({**self.plan, **change}, self.profile, factory, self.clock, self.stop)
            factory.assert_not_called()

    def test_invalid_profile_is_rejected_before_any_sdk_port(self):
        for bad in (
            {},
            {**self.profile, "state_timeout_s": float("nan")},
            {**self.profile, "max_tick_gap_s": 0.001},
            {**self.profile, "return_tolerance_rad": 0.3},
        ):
            factory = Mock()
            with self.assertRaises(ValueError):
                execute_replay(self.plan, bad, factory, self.clock, self.stop)
            factory.assert_not_called()
        with self.assertRaises(ValueError):
            simulate_replay(self.plan, profile={})

    def test_missing_feedback_at_start_creates_no_publisher(self):
        self.sdk.drop_states = True
        report = self.run_executor()
        self.assertIn("状态缺失", report["errors"][0]["message"])
        self.assertFalse(self.sdk.publisher_started)

    def test_delayed_first_state_is_waited_for_before_publisher_creation(self):
        self.sdk.drop_states = True

        def arrive():
            if self.clock.now >= 0.1:
                self.sdk.drop_states = False
            if self.clock.now < 0.1:
                self.assertFalse(self.sdk.publisher_started)

        self.sdk.before_step = arrive
        report = self.run_executor()
        self.assertEqual(report["status"], "completed")

    def test_stale_feedback_after_zero_weight_is_not_completed(self):
        def drop(command):
            if self.clock.now > 3.3 and command.motor_cmd[29].q == 0:
                self.sdk.drop_states = True

        self.sdk.before_write = drop
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["release_sent"])
        self.assertFalse(report["handover_confirmed"])
        self.assertIn("未收到新反馈", report["errors"][0]["message"])

    def test_tracking_error_is_reported_and_recovery_is_frozen(self):
        report = simulate_replay(self.plan, "tracking-error")
        self.assertIn("跟踪误差超限", report["errors"][0]["message"])
        self.assertGreater(report["metrics"]["max_tracking_error_rad"], 0.12)
        self.assertTrue(report["recovery_release_sent"])
        targets = {tuple(s["target_q"]) for s in report["samples"] if s["phase"] == "recovery_release"}
        self.assertEqual(len(targets), 1)

    def test_rechecks_numeric_limits_even_with_recomputed_digest(self):
        self.plan["frames"][40]["q"][3] = 1
        self.plan["schedule_sha256"] = frame_digest(self.plan["frames"])
        with self.assertRaisesRegex(ValueError, "筛查"):
            self.run_executor()
        self.assertFalse(self.sdk.resources)

    def test_rejects_nonmonotonic_weights_and_invalid_phases(self):
        for mutation in (lambda p: p["frames"][2].update(weight=0), lambda p: p["frames"][30].update(phase="unknown")):
            plan = copy.deepcopy(self.plan)
            mutation(plan)
            plan["schedule_sha256"] = frame_digest(plan["frames"])
            with self.assertRaises(ValueError):
                execute_replay(plan, self.profile, Mock(), self.clock, self.stop)

    def test_initial_mismatch_services_and_external_writer_block_publisher(self):
        for fault in ("pose", "service", "writer"):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == "pose":
                    self.sdk.q[0] = 0.1
                elif fault == "service":
                    self.sdk.services_ok = False
                else:
                    self.sdk.external_commands = 1
                report = self.run_executor()
                self.assertEqual(report["status"], "failed")
                self.assertFalse(self.sdk.publisher_started)
                self.assertFalse(self.sdk.writes)

    def test_cancel_before_start_does_not_create_port(self):
        self.stop.set()
        factory = Mock()
        report = execute_replay(self.plan, self.profile, factory, self.clock, self.stop)
        factory.assert_not_called()
        self.assertEqual(report["status"], "cancelled")

    def test_cancel_freezes_one_recovery_target_and_ramps_to_zero(self):
        def cancel():
            if self.clock.now > 4.7:
                self.stop.set()

        self.sdk.before_step = cancel
        report = self.run_executor()
        self.assertEqual(report["status"], "cancelled")
        self.assertTrue(report["recovery_release_sent"])
        self.assertTrue(report["requires_operator"])
        recovery = [s for s in report["samples"] if s["phase"] == "recovery_release"]
        self.assertEqual(len({tuple(s["target_q"]) for s in recovery}), 1)
        self.assertEqual(recovery[-1]["weight"], 0)
        self.assertTrue(all(b["weight"] <= a["weight"] for a, b in zip(recovery, recovery[1:])))

    def test_first_positive_write_failure_still_recovers(self):
        fired = False

        def fail(command):
            nonlocal fired
            if not fired and command.motor_cmd[29].q > 0:
                fired = True
                self.sdk.fail_next_write = True

        self.sdk.before_write = fail
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["recovery_release_sent"])
        self.assertEqual(self.sdk.writes[-1].motor_cmd[29].q, 0)
        self.assertEqual(report["samples"][1]["write_confirmed"], False)

    def test_invalid_feedback_stops_without_blind_recovery_commands(self):
        cases = ("stale", "mode", "motors", "nan", "bounds", "speed", "offset")
        for case in cases:
            with self.subTest(case=case):
                self.setUp()
                if case == "offset":
                    self.profile["max_offset_rad"] = 0.2
                writes_at_fault = []

                def invalidate():
                    if self.clock.now >= 3.2 and not writes_at_fault:
                        writes_at_fault.append(len(self.sdk.writes))
                        if case == "stale":
                            self.sdk.drop_states = True
                        elif case == "mode":
                            self.sdk.mode_machine = 4
                        elif case == "motors":
                            self.sdk.motor_modes[0] = 0
                        elif case == "nan":
                            self.sdk.q[0] = float("nan")
                        elif case == "bounds":
                            self.sdk.q[0] = 10
                            self.sdk.stuck = True
                        elif case == "speed":
                            self.sdk.target[0] = 1
                            self.sdk.weight = 1
                        else:
                            self.sdk.q[0] = 0.3
                            self.sdk.stuck = True

                self.sdk.before_step = invalidate
                report = self.run_executor()
                self.assertEqual(report["status"], "failed")
                self.assertFalse(report["recovery_release_sent"])
                self.assertTrue(report["cleanup_errors"])
                self.assertTrue(report["requires_operator"])
                # 状态丢失在新鲜度窗口内可能继续发送，超时后不再恢复。
                if case != "stale":
                    self.assertEqual(len(self.sdk.writes), writes_at_fault[0])

    def test_tick_overrun_aborts_and_preserves_fixed_recovery(self):
        fired = False

        def stall():
            nonlocal fired
            if not fired and self.clock.now > 3.2:
                fired = True
                self.clock.now += 0.2

        self.sdk.before_step = stall
        report = self.run_executor()
        self.assertIn("控制周期超时", report["errors"][0]["message"])
        self.assertTrue(report["recovery_release_sent"])

    def test_return_timeout_does_not_report_completed(self):
        self.profile["return_tolerance_rad"] = 0.00001

        def stuck():
            if self.clock.now > 4.8:
                self.sdk.stuck = True

        self.sdk.before_step = stuck
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertIn("回位等待超时", report["errors"][0]["message"])
        self.assertTrue(report["recovery_release_sent"])

    def test_position_in_tolerance_but_speed_not_settled_blocks_release(self):
        original = self.sdk._publish_state

        def moving():
            original()
            if self.port is not None and self.clock.now > 7.4:
                self.port.latest = {**self.port.latest, "dq": (0.1,) * 14}

        self.sdk._publish_state = moving
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertIn("回位等待超时", report["errors"][0]["message"])
        self.assertFalse(report["release_sent"])

    def test_release_failure_preserves_primary_and_cleanup_errors(self):
        report = simulate_replay(self.plan, "release-failure")
        self.assertEqual(report["status"], "failed")
        self.assertFalse(report["release_sent"])
        self.assertFalse(report["recovery_release_sent"])
        self.assertTrue(report["errors"])
        self.assertTrue(report["cleanup_errors"])
        self.assertTrue(report["resources_closed"])

    def test_invalid_feedback_during_release_blocks_remaining_writes(self):
        count = []

        def invalidate():
            if self.clock.now > 7.8 and not count:
                self.sdk.mode_machine = 4
                count.append(len(self.sdk.writes))

        self.sdk.before_step = invalidate
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertEqual(len(self.sdk.writes), count[0])
        self.assertFalse(report["release_sent"])

    def test_close_failure_after_success_is_not_completed(self):
        self.sdk.close_failure = True
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["release_sent"])
        self.assertTrue(report["cleanup_errors"])

    def test_sdk_command_mapping_and_crc_with_no_hand_commands(self):
        self.run_executor()
        command = self.sdk.writes[60]
        self.assertEqual((command.mode_pr, command.mode_machine, command.crc), (0, 5, 1234))
        for slot in ARM_SLOTS:
            motor = command.motor_cmd[slot]
            self.assertEqual((motor.mode, motor.kp, motor.kd, motor.dq, motor.tau), (1, 12, 1.2, 0, 0))
        self.assertTrue(all(command.motor_cmd[i].kp == 0 for i in list(range(15)) + list(range(30, 35))))

    def test_partial_adapter_initialization_closes_created_subscription(self):
        original = self.sdk.ChannelSubscriber

        def fail(topic, kind):
            if topic == "rt/arm_sdk":
                raise RuntimeError("模拟订阅初始化失败")
            return original(topic, kind)

        self.sdk.ChannelSubscriber = fail
        report = self.run_executor()
        self.assertEqual(report["status"], "failed")
        self.assertFalse(self.sdk.publisher_started)

    def test_all_simulation_scenarios_close_resources(self):
        for scenario in ("normal", "tracking-error", "state-timeout", "mode-change", "send-failure", "cancel"):
            report = simulate_replay(self.plan, scenario)
            self.assertEqual(
                report["status"],
                "completed" if scenario == "normal" else "cancelled" if scenario == "cancel" else "failed",
            )
            self.assertFalse(report["hardware_enabled"])
            self.assertTrue(report["resources_closed"])


if __name__ == "__main__":
    unittest.main()
