"""G1 Arm7 动作制作工具；teach/replay --execute 会发布手臂命令。"""

__version__ = "0.1.0"

PROFILE = "unitree-g1-arm7-29dof"
ARM_SLOTS = tuple(range(15, 29))
WEIGHT_SLOT = 29
JOINT_NAMES = (
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)
