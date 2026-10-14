import contextlib
import io
import tempfile
import unittest
from pathlib import Path

from g1_action_studio.dev_cli import main
from g1_action_studio.author import write_draft
from g1_action_studio.unitree_audit import (
    audit_unitree_replay, block_network, command_summary, fake_services,
)
from g1_action_studio.unitree_binding import UnitreeReplaySDK
from test_executor import make_plan


class UnitreeBindingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import cyclonedds
            import unitree_sdk2py
        except ImportError as error:
            raise unittest.SkipTest("真实 SDK 依赖不可用：{}；请在兼容 SDK 环境运行".format(error))

    def test_default_binding_creates_no_transport_and_refuses_network(self):
        with block_network() as (attempts, _):
            sdk = UnitreeReplaySDK()
            self.assertIsInstance(sdk.LowCmd(), sdk.LowCmdType)
            self.assertIsInstance(sdk.LowState(), sdk.LowStateType)
            for operation in (lambda: sdk.initialize_network("unused"),
                              lambda: sdk.ChannelPublisher("rt/arm_sdk", sdk.LowCmdType),
                              lambda: sdk.ChannelSubscriber("rt/lowstate", sdk.LowStateType),
                              sdk.initialize_service_clients, sdk.read_services):
                with self.assertRaises(RuntimeError):
                    operation()
            self.assertFalse(sdk.network_initialized)
            self.assertFalse(attempts)

    def test_actual_message_crc_roundtrip_and_tampering(self):
        with block_network():
            sdk = UnitreeReplaySDK()
            command = sdk.LowCmd()
            command.mode_machine = 5
            command.motor_cmd[15].mode = 1
            command.motor_cmd[15].q = 0.125
            command.motor_cmd[15].kp = 12
            command.motor_cmd[29].q = 0.5
            command.crc = sdk.CRC().Crc(command)
            report = command_summary(command)
            self.assertEqual(report["motor_count"], 35)
            self.assertEqual(report["crc_layout_bytes"], 1004)
            self.assertEqual(report["weight"], 0.5)
            command.motor_cmd[15].q += 0.125
            with self.assertRaisesRegex(RuntimeError, "CRC"):
                command_summary(command)

    def test_read_only_rpc_adaptation_errors_are_not_guessed(self):
        with block_network() as (attempts, _):
            clients = fake_services()
            sdk = UnitreeReplaySDK(service_clients=clients)
            self.assertEqual(sdk.read_services()[0], {"motion_service": "ai", "fsm_id": 501})
            clients[0].GetFsmId = lambda: (1, None)
            with self.assertRaisesRegex(RuntimeError, "GetFsmId"):
                sdk.read_services()
            clients[0].GetFsmId = lambda: (0, 501)
            clients[2].ServiceList = lambda: (0, [])
            self.assertIsNone(sdk.read_services()[1])
            self.assertFalse(attempts)

    def test_real_channel_network_guard_rejects_accidental_use(self):
        with block_network() as (attempts, _):
            sdk = UnitreeReplaySDK()
            with self.assertRaisesRegex(RuntimeError, "离线审计禁止"):
                sdk.channel.ChannelPublisher("rt/arm_sdk", sdk.LowCmdType)
            self.assertEqual(len(attempts), 1)

    def test_offline_binding_cannot_initialize_network_even_if_requested(self):
        with block_network() as (attempts, _):
            sdk = UnitreeReplaySDK(offline_transport=object())
            with self.assertRaises(RuntimeError):
                sdk.initialize_network("unused", allow_network=True)
            self.assertFalse(attempts)

    def test_real_message_executor_and_cli(self):
        plan = make_plan()
        report = audit_unitree_replay(plan)
        self.assertEqual(report["status"], "passed")
        self.assertFalse(report["dds_initialized"])
        self.assertEqual(report["network_calls_blocked"], 0)
        self.assertGreater(report["roundtrip_commands"], 0)
        self.assertGreater(report["roundtrip_states"], report["roundtrip_commands"])
        self.assertEqual(report["packet_samples"][-1]["weight"], 0)
        self.assertTrue(report["resources_closed"])
        self.assertFalse(report["handover_confirmed"])
        with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
            source, output = Path(folder) / "plan.json", Path(folder) / "audit.json"
            write_draft(plan, source)
            args = ["audit-unitree-replay", str(source), "--output", str(output)]
            with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(main(args), 0)
                before = output.read_bytes()
                self.assertEqual(main(args), 1)
                self.assertEqual(output.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
