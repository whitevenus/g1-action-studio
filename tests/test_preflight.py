import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from g1_action_studio.preflight import (
    read_arm_service,
    read_control_state,
    report_summary,
)


class PreflightTests(unittest.TestCase):
    def test_mode_timeout_retries_readonly_query_then_requires_valid_result(self):
        loco = SimpleNamespace(GetFsmId=Mock(return_value=(0, 501)))
        switcher = SimpleNamespace(CheckMode=Mock(side_effect=[(3104, None), (0, {"name": "ai"})]))
        with patch("g1_action_studio.preflight.time.sleep") as sleep:
            result = read_control_state(loco, switcher)
        self.assertEqual(result, {"motion_service": "ai", "fsm_id": 501})
        self.assertEqual(switcher.CheckMode.call_count, 2)
        sleep.assert_called_once_with(0.5)

    def test_repeated_mode_timeout_stops_after_three_queries(self):
        loco = SimpleNamespace(GetFsmId=Mock())
        switcher = SimpleNamespace(CheckMode=Mock(return_value=(3104, None)))
        with patch("g1_action_studio.preflight.time.sleep") as sleep:
            with self.assertRaisesRegex(RuntimeError, "3104"):
                read_control_state(loco, switcher)
        self.assertEqual(switcher.CheckMode.call_count, 3)
        self.assertEqual(sleep.call_count, 2)
        loco.GetFsmId.assert_not_called()

    def test_other_mode_query_errors_are_not_retried(self):
        loco = SimpleNamespace(GetFsmId=Mock())
        switcher = SimpleNamespace(CheckMode=Mock(return_value=(3102, None)))
        with patch("g1_action_studio.preflight.time.sleep") as sleep:
            with self.assertRaisesRegex(RuntimeError, "3102"):
                read_control_state(loco, switcher)
        switcher.CheckMode.assert_called_once()
        sleep.assert_not_called()
        loco.GetFsmId.assert_not_called()

    def test_public_sdk_state_calls(self):
        class Loco:
            def GetFsmId(self):
                return 0, 501

        class Switcher:
            def CheckMode(self):
                return 0, {"name": "ai"}

        self.assertEqual(read_control_state(Loco(), Switcher()), {"motion_service": "ai", "fsm_id": 501})

    def test_service_list_reports_only_official_arm_service(self):
        class RobotState:
            def ServiceList(self):
                return 0, [
                    SimpleNamespace(name="other", status=1, protect=False),
                    SimpleNamespace(name="g1_arm_example", status=0, protect=True),
                ]

        self.assertEqual(read_arm_service(RobotState()), {"name": "g1_arm_example", "status": 0, "protect": True})

    def test_summary_never_claims_control_authority(self):
        report = {
            "path": "preflight.json",
            "qualification": "read_only_observation_control_authority_unconfirmed",
            "fsm_before": {"fsm_id": 501},
            "fsm_after": {"fsm_id": 501},
            "arm_service_before": {"name": "g1_arm_example", "status": 1},
            "arm_service_after": {"name": "g1_arm_example", "status": 1},
            "observation": {"arm_sdk_messages": 0, "lowstate_messages": 100},
            "errors": [],
        }
        summary = report_summary(report)
        self.assertNotIn("control_owner", summary)
        self.assertIn("unconfirmed", summary["qualification"])


if __name__ == "__main__":
    unittest.main()
