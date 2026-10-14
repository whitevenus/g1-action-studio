"""机型、关节布局与采样约定；不包含设备连接或运行时参数。"""

from pathlib import Path

PROFILE = "unitree-g1-arm7-29dof"
ARM_SLOTS = tuple(range(15, 29))
ARM_JOINT_COUNT = len(ARM_SLOTS)
LEFT = ARM_SLOTS[:ARM_JOINT_COUNT // 2]
RIGHT = ARM_SLOTS[ARM_JOINT_COUNT // 2:]
ARM_OFFSETS = {"left": range(len(LEFT)), "right": range(len(LEFT), ARM_JOINT_COUNT)}
WEIGHT_SLOT = 29  # SDK 控制权重，不是物理关节。
JOINT_NAMES = (
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)

DT = 0.02  # 50 Hz。
STATE_TIMEOUT = 0.25
EXPECTED_MODE = (0, 5)  # mode_pr、mode_machine。
EXPECTED_MOTOR_MODES = (1,) * ARM_JOINT_COUNT
LOWSTATE_TOPIC = "rt/lowstate"
ARM_COMMAND_TOPIC = "rt/arm_sdk"

RECORDING_SCHEMA = "g1-arm7-state-recording-v1"
DRAFT_SCHEMA = "g1-arm7-action-draft-v1"
PLAN_SCHEMA = "g1-arm7-offline-replay-plan-v1"
PROFILE_SCHEMA = "g1-arm7-teach-profile-v1"
RESOURCES = Path(__file__).resolve().parent / "resources"
DEFAULT_MODEL = RESOURCES / "g1_arm7_limits.xml"
COLLISION_MODEL = RESOURCES / "collision/g1_arm7_dex3.xml"
