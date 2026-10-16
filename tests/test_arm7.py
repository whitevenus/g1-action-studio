"""仅手臂配置、文件与通信的回归；默认不建立任何 DDS 网络连接。"""

import contextlib
import io
import json
import tempfile
import unittest
from dataclasses import FrozenInstanceError
from pathlib import Path
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch

from fake_sdk import SimulatedSDK, VirtualClock

from g1_action_studio.author import load_action
from g1_action_studio.cli import main
from g1_action_studio.config import load_teach_profile, validate_teach_profile, write_json
from g1_action_studio.constants import ARM_LAYOUT, ARM_SLOTS, DEX3_LAYOUT, RECORDING_SCHEMA_V2, get_layout
from g1_action_studio.replay import build_schedule
from g1_action_studio.replay_sdk import ReplaySdkPort, validate_snapshot
from g1_action_studio.trajectory import finite_vector, interpolate_positions
from g1_action_studio.unitree_binding import UnitreeReplaySDK

ROOT = Path(__file__).parents[1]


class ArmOnlySDK(SimulatedSDK):
    """任何手部协议访问都会失败，防止仅检查写入数而漏掉隐式依赖。"""

    def __getattribute__(self, name):
        if name in ("HandCmd", "HandCmdType", "HandStateType", "initialize_hand_types"):
            raise AssertionError("仅手臂流程访问手部协议：" + name)
        return super().__getattribute__(name)

    def ChannelSubscriber(self, topic, message_type):
        if "dex3" in topic:
            raise AssertionError("仅手臂流程创建手部订阅")
        return super().ChannelSubscriber(topic, message_type)

    def ChannelPublisher(self, topic, message_type):
        if "dex3" in topic:
            raise AssertionError("仅手臂流程创建手部发布器")
        return super().ChannelPublisher(topic, message_type)


def recording(q):
    return {
        "schema": RECORDING_SCHEMA_V2,
        "target_profile": ARM_LAYOUT.profile_id,
        "collision_hand_model": "dex3-1",
        "arm_slots": list(ARM_SLOTS),
        "joint_names": list(ARM_LAYOUT.joint_names),
        "samples": [{"t": i * 0.02, "q": list(q), "dq": [0.0] * 14, "mode_pr": 0, "mode_machine": 5} for i in range(4)],
    }


class Arm7Tests(unittest.TestCase):
    def test_layouts_are_independent_and_vectors_do_not_truncate(self):
        self.assertEqual(get_layout(ARM_LAYOUT.profile_id).joint_count, 14)
        self.assertEqual(DEX3_LAYOUT.joint_count, 28)
        self.assertEqual(ARM_LAYOUT.indices("right"), tuple(range(7, 14)))
        with self.assertRaises(FrozenInstanceError):
            ARM_LAYOUT.profile_id = "changed"
        for layout in (ARM_LAYOUT, DEX3_LAYOUT, ARM_LAYOUT):
            self.assertEqual(len(finite_vector([0] * layout.joint_count, size=layout.joint_count)), layout.joint_count)
        with self.assertRaises(ValueError):
            interpolate_positions([0] * 14, [0] * 28, 0.5)
        with self.assertRaises(ValueError):
            get_layout("unknown")

    def test_profiles_and_dry_run_need_no_hand_gains_or_sdk(self):
        profile = load_teach_profile(ROOT / "examples/profiles/teach-arm7.json")
        self.assertFalse(any(key.startswith("hand_") for key in profile))
        with patch("g1_action_studio.teach.create_sdk", side_effect=AssertionError("不应连接")):
            with contextlib.redirect_stdout(io.StringIO()):
                result = main(
                    [
                        "teach",
                        "unused",
                        "--side",
                        "left",
                        "--profile",
                        str(ROOT / "examples/profiles/teach-arm7.json"),
                        "--output",
                        "unused-new.json",
                    ]
                )
        self.assertEqual(result, 0)
        for changes in ({"hand_hold_kp": 1}, {"target_profile": "other"}, {"collision_hand_model": None}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_teach_profile({**profile, **changes})

    def test_build_roundtrip_and_malformed_layout(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            source, target = Path(folder) / "record.json", Path(folder) / "action.json"
            write_json(recording([0.0] * 14), source)
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(main(["build", str(source), "--output", str(target)]), 0)
            action = load_action(target)
            self.assertEqual(action["target_profile"], ARM_LAYOUT.profile_id)
            self.assertEqual(len(action["frames"][0]["q"]), 14)
            bad = recording([0.0] * 28)
            source.write_text(json.dumps(bad), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_action(source)

    def test_sdk_receives_writes_and_closes_without_hand_protocol(self):
        clock = VirtualClock()
        sdk = ArmOnlySDK(clock, [0.0] * 14)
        port = ReplaySdkPort(sdk, clock, layout=ARM_LAYOUT)
        self.addCleanup(port.close)
        self.assertTrue(port.wait_ready(1, Event()))
        self.assertEqual(len(port.snapshot()["q"]), 14)
        port.start_publisher()
        with self.assertRaises(ValueError):
            port.write([0.0] * 28, 12, 1, 0)
        self.assertEqual(sdk.writes, [])
        self.assertTrue(port.write([0.1] * 14, 12, 1, 0.5))
        self.assertEqual([sdk.writes[-1].motor_cmd[i].q for i in ARM_SLOTS], [0.1] * 14)
        self.assertEqual(sdk.writes[-1].motor_cmd[29].q, 0.5)
        self.assertEqual(sdk.hand_writes, [])
        sdk.drop_states = True
        clock.sleep(2)
        with self.assertRaises(RuntimeError):
            validate_snapshot(port.snapshot(), clock.now, layout=ARM_LAYOUT)
        port.close()
        self.assertTrue(all(resource.closed for resource in sdk.resources))

    def test_real_binding_does_not_import_hand_types_until_enabled(self):
        types = SimpleNamespace(LowCmd_=object, LowState_=object)
        defaults = SimpleNamespace(unitree_hg_msg_dds__LowCmd_=lambda: None)
        modules = {
            "unitree_sdk2py": SimpleNamespace(),
            "unitree_sdk2py.core": SimpleNamespace(channel=object()),
            "unitree_sdk2py.idl.default": defaults,
            "unitree_sdk2py.idl.unitree_hg.msg.dds_": types,
            "unitree_sdk2py.utils.crc": SimpleNamespace(CRC=object),
        }
        with patch.dict("sys.modules", modules):
            sdk = UnitreeReplaySDK()
            self.assertFalse(hasattr(sdk, "HandCmdType"))
            with self.assertRaises(ImportError):
                sdk.initialize_hand_types()
            types.HandCmd_ = types.HandState_ = object
            defaults.unitree_hg_msg_dds__HandCmd_ = lambda: None
            sdk.initialize_hand_types()
            sdk.initialize_hand_types()
            self.assertIs(sdk.HandCmdType, object)

    def test_cli_replay_arguments_are_not_shifted(self):
        with patch("g1_action_studio.live_replay.replay", return_value={"status": "preview"}) as replay:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(
                    main(
                        [
                            "replay",
                            "unused",
                            "action.json",
                            "--profile",
                            "profile.json",
                            "--output",
                            "result.json",
                            "--side",
                            "right",
                            "--speed-scale",
                            "0.3",
                        ]
                    ),
                    0,
                )
        replay.assert_called_once_with(
            "unused",
            ["action.json"],
            profile_path="profile.json",
            output="result.json",
            side="right",
            speed_scale=0.3,
            execute=False,
        )

    def test_legacy_source_provenance_and_bytes_are_preserved(self):
        source = ROOT / "examples/transition/action-b.json"
        original = source.read_bytes()
        action = load_action(source)
        self.assertEqual(source.read_bytes(), original)
        self.assertEqual(action["source"]["schema"], "g1-arm7-dex3-state-recording-v1")
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            legacy = {**action, "schema": "g1-arm7-dex3-action-v1"}
            path = Path(folder) / "old-action.json"
            write_json(legacy, path)
            self.assertEqual(load_action(path)["source_schema"], "g1-arm7-dex3-action-v1")

    def test_schedule_accepts_fourteen_dimensions(self):
        frames = [{"t": i * 0.02, "q": [i * 0.001] * 14} for i in range(4)]
        schedule, _ = build_schedule(frames, [0.0] * 14)
        self.assertTrue(all(len(row["q"]) == 14 for row in schedule))
