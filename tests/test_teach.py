"""示教入口、共享通信与状态检查；不连接机器人。"""

import io
import json
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from fake_sdk import SimulatedSDK, VirtualClock

from g1_action_studio.cli import main
from g1_action_studio.config import load_replay_profile, load_teach_profile
from g1_action_studio.replay_sdk import ReplaySdkPort, validate_snapshot
from g1_action_studio.teach import teach
from g1_action_studio.unitree_binding import create_sdk

ROOT = Path(__file__).parents[1]


class TeachTests(unittest.TestCase):
    def test_preview_does_not_load_sdk_or_write_output(self):
        with (
            patch("g1_action_studio.teach.create_sdk") as sdk,
            patch.object(Path, "exists", return_value=False),
            patch("g1_action_studio.teach.write_json") as writer,
        ):
            result = teach("eth0", "left", "unused.json", ROOT / "examples/profiles/teach.json")
        self.assertFalse(result["execute"])
        sdk.assert_not_called()
        writer.assert_not_called()

    def test_invalid_side_or_existing_output_blocks_connection(self):
        with patch("g1_action_studio.teach.create_sdk") as sdk:
            with self.assertRaisesRegex(ValueError, "示教侧"):
                teach("unused", "both", "unused.json", "unused-profile.json", True)
            with patch.object(Path, "exists", return_value=True):
                with self.assertRaises(FileExistsError):
                    teach("unused", "left", "existing.json", "unused-profile.json", True)
            sdk.assert_not_called()

    def test_cli_only_accepts_full_teaching_flow(self):
        arguments = ["teach", "eth0", "--side", "left", "--profile", "teach.json", "--output", "recording.json"]
        with patch("g1_action_studio.teach.teach", return_value={}) as run, patch("sys.stdout", io.StringIO()):
            self.assertEqual(main(arguments), 0)
        run.assert_called_once_with("eth0", "left", "recording.json", "teach.json", False)
        for option, value in (
            ("--stage", "hold"),
            ("--seconds", "3"),
            ("--takeover-seconds", "5"),
            ("--soften-seconds", "8"),
        ):
            with self.subTest(option=option), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(arguments + [option, value])
                self.assertEqual(error.exception.code, 2)

    def test_teaching_requires_all_gains_but_replay_only_requires_hold_gains(self):
        replay = ROOT / "examples/profiles/replay.json"
        self.assertIn("hand_hold_kp", load_replay_profile(replay))
        with self.assertRaisesRegex(ValueError, "teach_kp"):
            load_teach_profile(replay)

    def test_rejects_unbounded_or_missing_gains(self):
        profile = json.loads((ROOT / "examples/profiles/teach.json").read_text(encoding="utf-8"))
        for changes, error in (
            ({"hold_kp": 100}, "保持增益"),
            ({"hand_teach_kp": -1}, "手指示教刚度"),
            ({"hand_teach_kd": None}, "hand_teach_kd"),
        ):
            with (
                self.subTest(changes=changes),
                patch.object(Path, "read_text", return_value=json.dumps({**profile, **changes})),
            ):
                with self.assertRaisesRegex(ValueError, error):
                    load_teach_profile("unused")

    def test_rejects_missing_stale_or_invalid_feedback(self):
        snapshot = {
            "at": 10.0,
            "mode_pr": 0,
            "mode_machine": 5,
            "motor_modes": (1,) * 14,
            "q": (0.0,) * 28,
            "dq": (0.0,) * 28,
        }
        self.assertIs(validate_snapshot(snapshot, 10.0), snapshot)
        with self.assertRaisesRegex(RuntimeError, "反馈"):
            validate_snapshot(None, 10.0)
        for key, value in (
            ("at", 8.9),
            ("at", float("nan")),
            ("mode_machine", 4),
            ("motor_modes", (0,) * 14),
            ("q", (0.0,) * 14),
            ("dq", (float("nan"),) * 28),
        ):
            with self.subTest(key=key, value=value), self.assertRaises(RuntimeError):
                validate_snapshot({**snapshot, key: value}, 10.0)


class PortTests(unittest.TestCase):
    def test_multiple_close_failures_still_attempt_all_channels_once(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        port = ReplaySdkPort(sdk, clock)
        port.hand_profile = load_teach_profile(ROOT / "examples/profiles/teach.json")
        port.start_publisher()
        for resource in sdk.resources:
            resource.Close = Mock(wraps=resource.Close)
        port.publisher.Close.side_effect = RuntimeError("手臂关闭失败")
        port.hand_publishers["right"].Close.side_effect = RuntimeError("手指关闭失败")
        with self.assertRaisesRegex(RuntimeError, "手臂关闭失败.*手指关闭失败"):
            port.close()
        port.close()
        for resource in sdk.resources:
            resource.Close.assert_called_once()

    def test_wire_command_keeps_body_slots_untouched_and_uses_separate_hand_commands(self):
        clock = VirtualClock()
        sdk = SimulatedSDK(clock, [0.0] * 28)
        port = ReplaySdkPort(sdk, clock)
        port.hand_profile = load_teach_profile(ROOT / "examples/profiles/teach.json")
        port.start_publisher()
        self.assertTrue(
            port.write([0.25] * 28, 12, 1.2, 0.5, side="right", selected_kp=2, selected_kd=0.6, hand_kp=0, hand_kd=0.05)
        )
        command = sdk.writes[-1]
        self.assertEqual(command.motor_cmd[15].kp, 12)
        self.assertEqual(command.motor_cmd[22].kp, 2)
        self.assertEqual(command.motor_cmd[22].q, 0.25)
        self.assertEqual(command.motor_cmd[22].dq, 0)
        self.assertEqual(command.motor_cmd[22].tau, 0)
        self.assertEqual(command.motor_cmd[29].q, 0.5)
        self.assertTrue(all(m.q == 0 for m in command.motor_cmd[:15] + command.motor_cmd[30:]))
        self.assertEqual(command.crc, 1234)
        self.assertEqual(len(sdk.hand_writes), 2)
        self.assertEqual(sdk.hand_writes[0][1].motor_cmd[0].kp, 1.5)
        self.assertEqual(sdk.hand_writes[1][1].motor_cmd[0].kp, 0)
        port.close()

    def test_close_attempts_every_resource_even_if_publisher_close_fails(self):
        port = ReplaySdkPort.__new__(ReplaySdkPort)
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
        sdk = Mock()
        resource = sdk.ChannelSubscriber.return_value
        resource.Init.side_effect = RuntimeError("初始化失败")
        with self.assertRaisesRegex(RuntimeError, "初始化失败"):
            ReplaySdkPort(sdk, VirtualClock())
        resource.Close.assert_called_once()

    def test_teaching_and_replay_share_explicit_sdk_connection(self):
        with patch("g1_action_studio.unitree_binding.UnitreeReplaySDK") as factory:
            sdk = create_sdk("eth0")
        self.assertIs(sdk, factory.return_value)
        sdk.initialize_network.assert_called_once_with("eth0", allow_network=True)
        sdk.initialize_service_clients.assert_called_once()
