"""Linux 几何与仅手臂完整用户路径；CI 不允许缺少规划依赖后静默跳过。"""

import contextlib
import io
import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

from fake_sdk import VirtualClock
from test_arm7 import ROOT, ArmOnlySDK, recording

from g1_action_studio.author import load_action
from g1_action_studio.cli import main
from g1_action_studio.collision import CollisionModel
from g1_action_studio.config import write_json
from g1_action_studio.constants import ARM_LAYOUT, DEX3_LAYOUT
from g1_action_studio.trajectory import frame_digest


class YieldClock(VirtualClock):
    def sleep(self, seconds):
        super().sleep(seconds)
        time.sleep(0.0001)  # 让真实规划工作线程运行；控制周期仍由虚拟时钟驱动。


class Arm7FlowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import ompl  # noqa: F401

            cls.model = CollisionModel(layout=ARM_LAYOUT)
        except ImportError as error:
            if os.environ.get("G1_REQUIRE_PLANNING") == "1":
                raise RuntimeError("端到端验收缺少规划依赖") from error
            raise unittest.SkipTest(str(error)) from error
        cls.q = json.loads((ROOT / "examples/transition/action-b.json").read_text())["samples"][0]["q"][:14]

    def command(self, arguments):
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            return main(arguments)

    def test_envelope_proves_nonzero_motion_and_rejects_collision(self):
        self.assertTrue(self.model.clearance(self.q)["valid"])
        end = list(self.q)
        end[0] -= 0.01
        self.assertTrue(self.model.edge_clear(self.q, end, 0.02))
        self.assertFalse(self.model.clearance([0.0] * 14)["valid"])
        self.assertEqual(len(self.model.hand_geometry), 14)
        names = [(self.model.model.geom(int(a)).name, self.model.model.geom(int(b)).name) for a, b in self.model.pairs]
        self.assertTrue(any("left_hand_thumb_2" in b and "left_wrist_roll" in a for a, b in names))

    def test_envelope_lower_bound_covers_sampled_original_meshes(self):
        import numpy as np

        original = CollisionModel(layout=DEX3_LAYOUT)
        rng = np.random.default_rng(42)
        conservative = self.model.clearance(self.q)
        for _ in range(20):
            q = self.q + rng.uniform(original.limits[14:, 0], original.limits[14:, 1]).tolist()
            original._position(q)
            # 验证最紧的手与腕安装区；随机测试只是解析区间覆盖证明的辅助回归。
            for a, b in self.model.pairs:
                names = [original.model.geom(int(g)).name for g in (a, b)]
                if "_wrist_" in names[0] and "_hand_" in names[1] and names[0].split("_")[0] == names[1].split("_")[0]:
                    distance = original.mj.mj_geomDistance(original.model, original.data, int(a), int(b), 0.5, None)
                    self.assertGreaterEqual(distance + 1e-8, conservative["mount_gap_m"])

    def teach_record(self, side, source):
        selected = 0 if side == "left" else 7

        clock = YieldClock()
        sdk = ArmOnlySDK(clock, self.q)

        def keys(events, stop):
            began = clock.now
            sent = 0

            def input_step(_seconds):
                nonlocal sent
                elapsed = clock.now - began
                if elapsed < 0.8:
                    sdk.q[selected] -= 0.0004
                    sdk.target[selected] = sdk.q[selected]
                if elapsed >= 0.2 and sent == 0:
                    events.put("enter")
                    sent = 1
                if elapsed >= 0.8 and sent == 1:
                    events.put("enter")
                    sent = 2

            clock.listeners.append(input_step)

        class InputThread:
            def __init__(self, target, args, daemon):
                self.target, self.args = target, args

            def start(self):
                self.target(*self.args)

        with (
            patch("g1_action_studio.teach.create_sdk", return_value=sdk),
            patch("g1_action_studio.teach.time", clock),
            patch("g1_action_studio.teach.Thread", InputThread),
            patch("g1_action_studio.teach.read_record_keys", keys),
        ):
            self.assertEqual(
                self.command(
                    [
                        "teach",
                        "unused",
                        "--side",
                        side,
                        "--profile",
                        str(ROOT / "examples/profiles/teach-arm7.json"),
                        "--output",
                        str(source),
                        "--execute",
                    ]
                ),
                0,
            )

        return sdk

    def test_cli_teach_build_transition_preview_and_execution(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            folder = Path(folder)
            source, action = folder / "record.json", folder / "action.json"
            combined, preview, result = folder / "combined.json", folder / "preview.json", folder / "result.json"
            sdk = self.teach_record("left", source)
            self.assertEqual(sdk.hand_writes, [])
            self.assertTrue(all(resource.closed for resource in sdk.resources))
            recorded = json.loads(source.read_text())
            self.assertGreater(abs(recorded["samples"][-1]["q"][0] - recorded["samples"][0]["q"][0]), 0.001)
            self.assertTrue(all(len(row["q"]) == 14 and len(row["dq"]) == 14 for row in recorded["samples"]))
            self.assertEqual(self.command(["build", str(source), "--output", str(action)]), 0)
            built = load_action(action)
            self.assertEqual(built["trajectory_sha256"], frame_digest(built["frames"]))
            self.assertEqual(self.command(["transition", str(action), str(action), "--output", str(combined)]), 0)
            profile = ROOT / "examples/profiles/replay-arm7.json"
            with patch("g1_action_studio.live_replay.create_sdk", side_effect=AssertionError("预览不得初始化网络")):
                self.assertEqual(
                    self.command(
                        ["replay", "unused", str(combined), "--profile", str(profile), "--output", str(preview)]
                    ),
                    0,
                )
            preview_data = json.loads(preview.read_text())
            self.assertFalse(preview_data["hardware_enabled"])
            self.assertEqual(preview_data["plan"]["collision"]["hand_collision_policy"], "full_range_envelope")
            playback_clock = VirtualClock()
            playback_sdk = ArmOnlySDK(playback_clock, built["frames"][0]["q"])
            with (
                patch("g1_action_studio.live_replay.create_sdk", return_value=playback_sdk),
                patch("g1_action_studio.live_replay.time", playback_clock),
            ):
                self.assertEqual(
                    self.command(
                        [
                            "replay",
                            "unused",
                            str(action),
                            str(combined),
                            "--profile",
                            str(profile),
                            "--output",
                            str(result),
                            "--execute",
                        ]
                    ),
                    0,
                )
            report = json.loads(result.read_text())
            self.assertEqual(report["status"], "completed")
            self.assertEqual(report["controlled_joint_count"], 14)
            self.assertFalse(report["hand_control_enabled"])
            self.assertEqual(playback_sdk.hand_writes, [])
            self.assertTrue(all(resource.closed for resource in playback_sdk.resources))
            with patch("g1_action_studio.live_replay.create_sdk", side_effect=AssertionError("布局错误不得连接")):
                self.assertEqual(
                    self.command(
                        [
                            "replay",
                            "unused",
                            str(action),
                            "--profile",
                            str(ROOT / "examples/profiles/replay.json"),
                            "--output",
                            str(folder / "bad.json"),
                            "--execute",
                        ]
                    ),
                    1,
                )

    def test_body_stream_loss_fails_execution_and_closes_resources(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            source, output = Path(folder) / "record.json", Path(folder) / "failed.json"
            write_json(recording(self.q), source)
            clock = VirtualClock()
            sdk = ArmOnlySDK(clock, self.q)

            def drop_body():
                if sdk.publisher_started and clock.now > 3.3:
                    sdk.drop_states = True

            sdk.before_step = drop_body
            with (
                patch("g1_action_studio.live_replay.create_sdk", return_value=sdk),
                patch("g1_action_studio.live_replay.time", clock),
            ):
                self.assertEqual(
                    self.command(
                        [
                            "replay",
                            "unused",
                            str(source),
                            "--profile",
                            str(ROOT / "examples/profiles/replay-arm7.json"),
                            "--output",
                            str(output),
                            "--execute",
                        ]
                    ),
                    1,
                )
            report = json.loads(output.read_text())
            self.assertEqual(report["status"], "failed")
            self.assertFalse(report["release_sent"])
            self.assertFalse(report["hand_control_enabled"])
            self.assertTrue(all(resource.closed for resource in sdk.resources))
            self.assertEqual(sdk.hand_writes, [])

    def test_unknown_hand_interval_budget_is_not_sampling_success(self):
        from g1_action_studio.hand_envelope import swept_distance

        model = self.model
        pair = next(
            (int(a), int(b)) for a, b in model.pairs if int(a) in model.hand_geometry or int(b) in model.hand_geometry
        )
        model._position(self.q)
        # 不可能满足的净空须拒绝，即使某一个取样手型可远离其他部件。
        bound = swept_distance(model.mj, model.model, model.data, *pair, 10.0, model.hand_geometry)
        self.assertLess(bound, 10.0)

    def test_right_arm_teach_keeps_left_arm_at_baseline(self):
        with tempfile.TemporaryDirectory(dir=ROOT / "tests") as folder:
            source = Path(folder) / "right.json"
            sdk = self.teach_record("right", source)
            data = json.loads(source.read_text())
            self.assertTrue(all(row["q"][:7] == self.q[:7] for row in data["samples"]))
            self.assertGreater(abs(data["samples"][-1]["q"][7] - data["samples"][0]["q"][7]), 0.001)
            self.assertEqual(sdk.hand_writes, [])
            self.assertTrue(all(resource.closed for resource in sdk.resources))

    def test_interval_budget_cannot_be_mistaken_for_sampling_pass(self):
        from g1_action_studio.hand_envelope import swept_distance

        model = self.model
        pair = next(
            (int(a), int(b)) for a, b in model.pairs if int(a) in model.hand_geometry or int(b) in model.hand_geometry
        )
        model._position(self.q)
        with patch.object(model.mj, "mj_geomDistance", return_value=0.02001) as distance:
            lower = swept_distance(model.mj, model.model, model.data, *pair, 0.02, model.hand_geometry)
        self.assertLess(lower, 0.02)
        self.assertLessEqual(distance.call_count, 4096)
