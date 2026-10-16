"""真实录制产物贯通三条命令；可从任意工作目录使用已安装的包执行。"""

import io
import json
import signal
import sys
import threading
import unittest
from contextlib import nullcontext, redirect_stderr, redirect_stdout
from unittest.mock import patch

import numpy as np
import test_replay
import test_teach
from fake_channels import HOME, Channels

import g1_action_studio
from g1_action_studio import cli, preview, replay, teach
from g1_action_studio.model import ArmModel
from g1_action_studio.motion import Trajectory, load_action


@unittest.skipUnless(sys.platform == "linux", "完整安装贯通的交付平台为 Linux")
class WorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
        from unitree_sdk2py.utils.crc import CRC

        cls.crc = CRC()
        cls.message_type = LowCmd_

    def setUp(self):
        self.teaching = test_teach.TeachTests()
        self.teaching.setUp()
        self.addCleanup(self.teaching.doCleanups)
        self.replaying = test_replay.ReplayTests()
        self.replaying.setUp()
        self.addCleanup(self.replaying.doCleanups)
        self.replaying.crc = self.crc
        self.replaying.message_type = self.message_type
        self.replaying.config = self.teaching.config

    def draw(self, paths, start_path):
        import mujoco.viewer

        curves, times, drawn, viewer_data = [], [], [], []
        planning = preview.plan_sequence

        def observe_plan(*arguments):
            curve = planning(*arguments)
            curves.append(curve)
            times.extend(np.linspace(0, curve.duration, 25))
            return curve

        class Viewer:
            def __init__(self, model, data):
                viewer_data.append((model, data))

            def __enter__(self):
                return self

            def __exit__(self, *_):
                pass

            def is_running(self):
                return len(drawn) < len(times)

            def lock(self):
                return nullcontext()

            def sync(self):
                data = viewer_data[0][1]
                drawn.append((data.qpos.copy(), data.xpos.copy()))

        clock = []

        def monotonic():
            if not clock:
                clock.append(True)
                return 0.0
            return float(times[len(drawn)])

        stdout, stderr = io.StringIO(), io.StringIO()
        with Channels() as channels:
            with patch.object(preview, "plan_sequence", side_effect=observe_plan):
                with patch.object(mujoco.viewer, "launch_passive", side_effect=Viewer):
                    with patch.object(preview.time, "monotonic", side_effect=monotonic):
                        with redirect_stdout(stdout), redirect_stderr(stderr):
                            code = cli.main(["preview", *map(str, paths), "--start", str(start_path)])
            self.assertEqual(channels.calls, [])
            self.assertEqual(channels.commands, [])
        self.assertEqual((code, stderr.getvalue()), (0, ""))
        self.assertEqual(len(curves), 1)
        model = ArmModel(np.zeros(12))
        for (qpos, xpos), moment in zip(drawn, times):
            q, _ = curves[0].sample(float(moment))
            np.testing.assert_allclose(qpos[model.upper_qpos], q, atol=1e-13)
            np.testing.assert_array_equal(qpos[model.leg_qpos], np.zeros(12))
            model.set_pose(q)
            np.testing.assert_allclose(xpos, model.data.xpos, atol=1e-13)
        self.assertIsNot(viewer_data[0][1], model.data)
        return curves[0]

    def test_recorded_files_drive_preview_replay_cancellation_and_saved_return_failure(self):
        paths = {}
        start_path = self.teaching.directory / "start.json"
        for group in ("left_arm", "right_arm", "arms", "arms_waist"):
            with self.teaching.environment(group) as recording:
                if not start_path.exists():
                    start_path.write_text(
                        json.dumps(
                            {
                                "leg_q": recording.channels.legs.tolist(),
                                "upper_q": recording.channels.upper.tolist(),
                            }
                        ),
                        encoding="utf-8",
                    )
                code, _, stderr = self.teaching.run_cli(recording)
                self.assertEqual((code, stderr), (0, ""))
                self.teaching.assert_saved(recording)
                self.teaching.assert_release(recording)
                paths[group] = recording.output
        original_files = {path: path.read_bytes() for path in paths.values()}
        mixed = [paths["arms_waist"], paths["left_arm"]]
        for path in (paths["right_arm"], paths["arms"]):
            self.draw([path], start_path)
            with self.replaying.environment() as (channels, _):
                code, _, stderr = self.replaying.run_cli([path])
                self.assertEqual((code, stderr), (0, ""))
                self.replaying.assert_completed(channels)
        reference = self.draw(mixed, start_path)
        first_duration = load_action(mixed[0]).t[-1]
        left_start = 4.0 + first_duration
        right_and_waist = list(range(7, 17))
        self.assertTrue(
            any(
                np.any(reference.sample(float(t))[0][right_and_waist] != HOME[right_and_waist])
                for t in np.linspace(0, 2.0 + first_duration, 21)
            )
        )
        for moment in np.linspace(left_start, reference.duration, 21):
            q, dq = reference.sample(float(moment))
            np.testing.assert_array_equal(q[right_and_waist], HOME[right_and_waist])
            np.testing.assert_array_equal(dq[right_and_waist], np.zeros(10))

        sampled, pending, replay_curves = [], [], []
        sampling = Trajectory.sample
        planning = replay.plan_sequence

        def observe_plan(*arguments):
            curve = planning(*arguments)
            replay_curves.append(curve)
            return curve

        def observe_sample(curve, moment):
            q, dq = sampling(curve, moment)
            if threading.current_thread().name == "g1-action-sender":
                target, velocity = sampling(reference, moment)
                np.testing.assert_allclose(q, target, atol=1e-13)
                np.testing.assert_allclose(dq, velocity, atol=1e-13)
                item = (moment, q.copy(), dq.copy())
                sampled.append(item)
                pending.append(item)
            return q, dq

        with self.replaying.environment() as (channels, _):

            def compare_write(record):
                if pending:
                    _, q, dq = pending.pop(0)
                    np.testing.assert_array_equal(test_replay.positions(record.command), q)
                    np.testing.assert_array_equal(test_replay.velocities(record.command), dq)
                return True

            channels.on_write = compare_write
            with patch.object(replay, "plan_sequence", side_effect=observe_plan):
                with patch.object(Trajectory, "sample", observe_sample):
                    code, _, stderr = self.replaying.run_cli(mixed)
            self.assertEqual((code, stderr), (0, ""))
            self.replaying.assert_completed(channels)
            self.assertTrue(any(command.motor_cmd[12].dq != 0 for command in channels.commands))
        self.assertEqual(len(replay_curves), 1)
        np.testing.assert_array_equal(replay_curves[0].polynomial.x, reference.polynomial.x)
        np.testing.assert_array_equal(replay_curves[0].polynomial.c, reference.polynomial.c)
        self.assertGreater(len(sampled), 2)
        self.assertEqual(pending, [])
        self.assertTrue(any(moment >= left_start and np.any(q[:7] != HOME[:7]) for moment, q, _ in sampled))

        frozen = []
        with self.replaying.environment() as (channels, _):

            def cancel_recorded_motion(record):
                if not frozen and np.any(test_replay.velocities(record.command)):
                    frozen.append(test_replay.positions(record.command))
                    signal.getsignal(signal.SIGINT)(signal.SIGINT, None)
                return True

            channels.on_write = cancel_recorded_motion
            code, stdout, stderr = self.replaying.run_cli(mixed)
            self.assertEqual((code, stderr), (130, ""))
            self.assertNotIn("流程完成", stdout)
            self.assertEqual(len(frozen), 1)
            commands = channels.commands
            trigger = next(index for index, command in enumerate(commands) if np.any(test_replay.velocities(command)))
            for command in commands[trigger + 1 :]:
                np.testing.assert_array_equal(test_replay.positions(command), frozen[0])
                np.testing.assert_array_equal(test_replay.velocities(command), np.zeros(17))
            self.assertFalse(np.array_equal(frozen[0], HOME))
            self.assertEqual(commands[-1].motor_cmd[29].q, 0)
            self.replaying.assert_channel_lifecycle(channels)

        with self.teaching.environment("left_arm") as recording:
            for feedback in recording.samples:
                feedback[:7] = HOME[:7]
                feedback[2] = -2.4
            with patch.object(teach, "ArmModel", side_effect=test_teach.disconnected_return_model):
                code, _, stderr = self.teaching.run_cli(recording)
            self.assertEqual(code, 1)
            self.assertTrue("近似" in stderr or "未找到精确解" in stderr, stderr)
            self.teaching.assert_saved(recording)
            self.teaching.assert_release(recording)
            failed_recording = recording.output
        saved_bytes = failed_recording.read_bytes()
        self.draw([failed_recording], start_path)
        self.assertEqual(failed_recording.read_bytes(), saved_bytes)
        for path, contents in original_files.items():
            self.assertEqual(path.read_bytes(), contents)


if __name__ == "__main__":
    print(f"workflow package: {g1_action_studio.__file__}", flush=True)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(WorkflowTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    raise SystemExit(0 if result.wasSuccessful() and not result.skipped else 1)
