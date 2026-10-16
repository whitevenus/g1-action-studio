import io
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from g1_action_studio.author import load_action
from g1_action_studio.cli import main
from g1_action_studio.config import write_json
from g1_action_studio.constants import ARM_SLOTS, JOINT_NAMES, PROFILE

TEST_ROOT = Path(__file__).resolve().parent


def source_file(path, *, gap=False, changed_mode=False):
    rows = []
    for index in range(51):
        t = index * 0.02
        if gap and 20 <= index <= 35:
            continue
        q = [0.0] * 28
        q[10] = 0.1 * (1 - abs(index - 25) / 25)
        rows.append(
            {"t": t, "q": q, "dq": [0.0] * 28, "mode_machine": 6 if changed_mode and index >= 25 else 5, "mode_pr": 0}
        )
    payload = {
        "schema": "g1-arm7-dex3-state-recording-v1",
        "target_profile": PROFILE,
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(JOINT_NAMES),
        "samples": rows,
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


class ActionTests(unittest.TestCase):
    def test_whole_recording_preserves_endpoints_and_partial_final_tick(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            source_file(source)
            data = json.loads(source.read_text(encoding="utf-8"))
            for row in data["samples"]:
                row["t"] += 7.005
            data["samples"][-1]["t"] += 0.005
            data["samples"][-1]["q"][10] = 0.2
            source.write_text(json.dumps(data), encoding="utf-8")
            original = source.read_bytes()
            action = load_action(source)
            self.assertEqual(source.read_bytes(), original)
        self.assertEqual(set(action["source"]), {"file_name", "sha256", "schema"})
        self.assertEqual(len(action["frames"]), 52)
        self.assertEqual([frame["t"] for frame in action["frames"]], [round(i * 0.02, 6) for i in range(52)])
        self.assertEqual(action["frames"][0]["q"], data["samples"][0]["q"])
        self.assertEqual(action["frames"][-1]["q"], data["samples"][-1]["q"])

    def test_build_cli_converts_entire_recording_with_metadata(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source, output = Path(folder) / "source.json", Path(folder) / "action.json"
            source_file(source)
            with patch("sys.stdout", io.StringIO()):
                status = main(
                    [
                        "build",
                        str(source),
                        "--output",
                        str(output),
                        "--code",
                        "classroom.emphasis",
                        "--version",
                        "2",
                        "--name",
                        "重点",
                    ]
                )
            self.assertEqual(status, 0)
            action = load_action(output)
        self.assertEqual(action["action_code"], "classroom.emphasis")
        self.assertEqual(action["action_version"], 2)
        self.assertEqual(action["display_name"], "重点")
        self.assertEqual(action["metrics"]["frame_count"], 51)
        self.assertEqual(action["metrics"]["duration_s"], 1.0)

    def test_build_cli_rejects_removed_options(self):
        for option in ("--start", "--end"):
            with self.subTest(option=option), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    main(["build", "unused.json", "--output", "unused-output.json", option, "0"])
                self.assertEqual(error.exception.code, 2)

    def test_build_produces_loadable_uniform_action(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            output = Path(folder) / "action.json"
            source_file(source)
            action = load_action(source, code="classroom.emphasis", version=1, name="重点")
            self.assertEqual(action["kind"], "single")
            self.assertFalse(action["synthetic"])
            self.assertEqual(action["metrics"]["frame_count"], 51)
            self.assertAlmostEqual(action["metrics"]["start_end_max_error_rad"], 0)
            self.assertGreater(action["metrics"]["max_excursion_from_start_rad"], 0)
            write_json(action, output)
            self.assertEqual(load_action(output), action)
            with self.assertRaises(FileExistsError):
                write_json(action, output)

    def test_rejects_missing_state_span(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            source_file(source, gap=True)
            with self.assertRaisesRegex(ValueError, "状态缺口"):
                load_action(source, code="classroom.emphasis", version=1, name="重点")

    def test_rejects_mode_change(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            source_file(source, changed_mode=True)
            with self.assertRaisesRegex(ValueError, "模式发生变化"):
                load_action(source, code="classroom.emphasis", version=1, name="重点")

    def test_rejects_modified_trajectory(self):
        with tempfile.TemporaryDirectory(dir=TEST_ROOT) as folder:
            source = Path(folder) / "source.json"
            output = Path(folder) / "action.json"
            source_file(source)
            action = load_action(source, code="classroom.emphasis", version=1, name="重点")
            action["frames"][25]["q"][10] += 0.2
            output.write_text(json.dumps(action), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "轨迹摘要不匹配"):
                load_action(output)


if __name__ == "__main__":
    unittest.main()
