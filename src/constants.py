"""机型、关节布局、采样约定、固定示教时序与资源路径。"""

from dataclasses import dataclass
from pathlib import Path

PROFILE = "unitree-g1-arm7-dex3-1"
ARM_SLOTS = tuple(range(15, 29))
ARM_JOINT_COUNT = len(ARM_SLOTS)
WEIGHT_SLOT = 29  # SDK 控制权重，不是物理关节。
ARM_JOINT_NAMES = (
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)
HAND_JOINT_NAMES = tuple(
    f"{side}_hand_{finger}_{index}"
    for side in ("left", "right")
    for finger, count in (("thumb", 3), ("middle", 2), ("index", 2))
    for index in range(count)
)
# 动作统一顺序：左臂7、右臂7、左手7、右手7；手部顺序跟模型名称，不跟DDS槽位。
JOINT_NAMES = ARM_JOINT_NAMES + HAND_JOINT_NAMES
JOINT_COUNT = len(JOINT_NAMES)
SIDE_OFFSETS = {"left": (*range(7), *range(14, 21)), "right": (*range(7, 14), *range(21, 28))}
HAND_MOTOR_IDS = {"left": (0, 1, 2, 3, 4, 5, 6), "right": (0, 1, 2, 5, 6, 3, 4)}
HAND_STATE_TOPICS = {side: f"rt/dex3/{side}/state" for side in HAND_MOTOR_IDS}
HAND_COMMAND_TOPICS = {side: f"rt/dex3/{side}/cmd" for side in HAND_MOTOR_IDS}

DT = 0.02  # 50 Hz。
TEACH_TAKEOVER_SECONDS = 2.0  # 保持初始姿态，手臂权重从0渐变到1。
TEACH_SOFTEN_SECONDS = 2.0  # 示教侧手臂和手指从保持增益渐变到示教增益。
STATE_TIMEOUT = 1  # 状态超时时间（秒）。
EXPECTED_MODE = (0, 5)  # mode_pr、mode_machine。
EXPECTED_MOTOR_MODES = (1,) * ARM_JOINT_COUNT
LOWSTATE_TOPIC = "rt/lowstate"
ARM_COMMAND_TOPIC = "rt/arm_sdk"

RECORDING_SCHEMA = "g1-arm7-dex3-state-recording-v1"
ACTION_SCHEMA = "g1-arm7-dex3-action-v1"
PLAN_SCHEMA = "g1-arm7-dex3-offline-replay-plan-v1"
PROFILE_SCHEMA = "g1-arm7-dex3-teach-profile-v1"
COLLISION_MODEL = Path(__file__).resolve().parent / "resources/collision/g1_arm7_dex3.xml"

RECORDING_SCHEMA_V2 = "g1-action-studio-recording-v2"
ACTION_SCHEMA_V2 = "g1-action-studio-action-v2"
PLAN_SCHEMA_V2 = "g1-action-studio-replay-plan-v2"
PROFILE_SCHEMA_V2 = "g1-action-studio-control-profile-v2"


@dataclass(frozen=True)
class JointLayout:
    profile_id: str
    joint_names: tuple[str, ...]
    left_indices: tuple[int, ...]
    right_indices: tuple[int, ...]
    hand_model: str | None

    @property
    def joint_count(self):
        return len(self.joint_names)

    def indices(self, side):
        if side not in ("left", "right"):
            raise ValueError("side 必须为 left/right")
        return self.left_indices if side == "left" else self.right_indices


ARM_LAYOUT = JointLayout("unitree-g1-arm7", ARM_JOINT_NAMES, tuple(range(7)), tuple(range(7, 14)), None)
DEX3_LAYOUT = JointLayout(PROFILE, JOINT_NAMES, SIDE_OFFSETS["left"], SIDE_OFFSETS["right"], "dex3-1")


def get_layout(profile_id):
    if profile_id == ARM_LAYOUT.profile_id:
        return ARM_LAYOUT
    if profile_id == DEX3_LAYOUT.profile_id:
        return DEX3_LAYOUT
    raise ValueError("不支持的 target_profile：{}".format(profile_id))
