import unittest
from unittest.mock import Mock, patch

import numpy as np
from cyclonedds.domain import Domain, DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic
from unitree_sdk2py.core import channel
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from g1_action_studio.robot import build_command


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.network_guards = []
        for constructor in (Domain, DomainParticipant, DataWriter, DataReader, Topic):
            self.network_guards.append(
                self.enterContext(
                    patch.object(constructor, "__init__", side_effect=AssertionError("DDS entities are forbidden"))
                )
            )
        for name in ("ChannelFactoryInitialize", "ChannelFactory", "ChannelPublisher", "ChannelSubscriber"):
            self.network_guards.append(
                self.enterContext(patch.object(channel, name, side_effect=AssertionError("SDK channels are forbidden")))
            )
        self.q = np.arange(1, 18, dtype=np.float64) / 32
        self.dq = -np.arange(1, 18, dtype=np.float64) / 64
        self.kp = np.arange(64, 81, dtype=np.float64)
        self.kd = np.arange(1, 18, dtype=np.float64) / 8
        self.crc = CRC()

    def tearDown(self):
        for guard in self.network_guards:
            guard.assert_not_called()

    def build(self, weight=0.375, crc=None):
        return build_command(self.q, self.dq, self.kp, self.kd, weight, self.crc if crc is None else crc)

    def assert_mapping(self, command, weight=0.375):
        self.assertIsInstance(command, LowCmd_)
        self.assertEqual(len(command.motor_cmd), 35)
        self.assertEqual((command.mode_pr, command.mode_machine), (0, 0))
        self.assertEqual(command.reserve, [0, 0, 0, 0])
        default = unitree_hg_msg_dds__LowCmd_()
        slots = (*range(15, 29), 12, 13, 14)
        self.assertEqual(len(set(slots)), 17)
        for index, slot in enumerate(slots):
            with self.subTest(slot=slot):
                motor = command.motor_cmd[slot]
                self.assertEqual(
                    (motor.q, motor.dq, motor.kp, motor.kd),
                    (self.q[index], self.dq[index], self.kp[index], self.kd[index]),
                )
                self.assertEqual((motor.mode, motor.tau, motor.reserve), (0, 0.0, 0))
        for slot in (*range(12), *range(30, 35)):
            with self.subTest(default_slot=slot):
                self.assertEqual(command.motor_cmd[slot], default.motor_cmd[slot])
        expected_weight = default.motor_cmd[29]
        expected_weight.q = weight
        self.assertEqual(command.motor_cmd[29], expected_weight)

    def test_official_message_maps_all_17_joints_and_preserves_default_slots(self):
        command = self.build()
        self.assert_mapping(command)
        self.assertTrue(all(command.motor_cmd[slot].dq != 0 for slot in (12, 13, 14)))
        self.assertEqual(command.crc, self.crc.Crc(command))

    def test_crc_is_calculated_after_fields_and_survives_real_xcdr_roundtrip(self):
        before_crc = []

        def calculate(command):
            before_crc.append(command.serialize())
            return self.crc.Crc(command)

        crc = Mock()
        crc.Crc.side_effect = calculate
        command = self.build(crc=crc)
        crc.Crc.assert_called_once_with(command)
        captured = LowCmd_.deserialize(before_crc[0])
        self.assert_mapping(captured)
        self.assertEqual(captured.crc, 0)
        self.assertEqual(command.crc, self.crc.Crc(captured))
        wire = command.serialize()
        self.assertEqual(len(wire), 1004)
        decoded = LowCmd_.deserialize(wire)
        self.assertEqual(decoded, command)
        self.assertEqual(decoded.crc, self.crc.Crc(decoded))
        decoded.motor_cmd[12].dq += 0.125
        self.assertNotEqual(decoded.crc, self.crc.Crc(decoded))

    def test_each_command_is_independent_and_inputs_are_unchanged(self):
        inputs = [values.copy() for values in (self.q, self.dq, self.kp, self.kd)]
        first = self.build()
        saved = first.serialize()
        second = self.build(weight=0.0)
        self.assert_mapping(second, weight=0.0)
        self.assertIsNot(first, second)
        self.assertIsNot(first.motor_cmd, second.motor_cmd)
        self.assertIsNot(first.reserve, second.reserve)
        for first_motor, second_motor in zip(first.motor_cmd, second.motor_cmd):
            self.assertIsNot(first_motor, second_motor)
        second.motor_cmd[15].q = 99.0
        second.reserve[0] = 1
        self.assertEqual(first.serialize(), saved)
        for actual, expected in zip((self.q, self.dq, self.kp, self.kd), inputs):
            np.testing.assert_array_equal(actual, expected)


if __name__ == "__main__":
    unittest.main()
