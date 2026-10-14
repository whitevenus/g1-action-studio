import json
import tempfile
import unittest
from pathlib import Path

from g1_action_studio import ARM_SLOTS, PROFILE
from g1_action_studio.author import audit_draft, build_draft, write_draft


TEST_ROOT = Path(__file__).resolve().parent


def source_file(path, *, gap=False, changed_mode=False):
    rows = []
    for index in range(51):
        t = index * 0.02
        if gap and 20 <= index <= 35:
            continue
        q = [0.0] * 14
        q[10] = 0.1 * (1 - abs(index - 25) / 25)
        rows.append({"t": t, "q": q, "dq": [0.0] * 14,
                     "mode_machine": 6 if changed_mode and index >= 25 else 5,
                     "mode_pr": 0})
    payload = {"schema": "g1-arm7-state-recording-v1",
               "target_profile": PROFILE, "arm_slots": list(ARM_SLOTS),
               "invalid_messages": 0, "samples": rows}
    path.write_text(json.dumps(payload), encoding="utf-8")


class DraftTests(unittest.TestCase):
    def test_build_is_unqualified_and_auditable(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            output = Path(folder) / "draft.json"
            source_file(source)
            draft = build_draft(source, 0.0, 1.0, "classroom.emphasis", 1, "重点")
            self.assertEqual(draft["qualification"], "draft_unqualified_not_executable")
            self.assertFalse(draft["hardware_enabled"])
            self.assertEqual(draft["metrics"]["frame_count"], 51)
            self.assertAlmostEqual(draft["metrics"]["start_end_max_error_rad"], 0)
            self.assertGreater(draft["metrics"]["max_excursion_from_start_rad"], 0)
            write_draft(draft, output)
            self.assertEqual(audit_draft(output), draft["metrics"])
            with self.assertRaises(FileExistsError):
                write_draft(draft, output)

    def test_rejects_missing_state_span(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            source_file(source, gap=True)
            with self.assertRaisesRegex(ValueError, "状态缺口"):
                build_draft(source, 0.0, 1.0, "classroom.emphasis", 1, "重点")

    def test_rejects_mode_change(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            source_file(source, changed_mode=True)
            with self.assertRaisesRegex(ValueError, "模式发生变化"):
                build_draft(source, 0.0, 1.0, "classroom.emphasis", 1, "重点")

    def test_rejects_modified_trajectory(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            output = Path(folder) / "draft.json"
            source_file(source)
            draft = build_draft(source, 0.0, 1.0, "classroom.emphasis", 1, "重点")
            draft["frames"][25]["q"][10] += 0.2
            output.write_text(json.dumps(draft), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "轨迹摘要不匹配"):
                audit_draft(output)


if __name__ == "__main__":
    unittest.main()
