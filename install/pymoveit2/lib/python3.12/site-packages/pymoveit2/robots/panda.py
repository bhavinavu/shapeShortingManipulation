from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.13]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [-0.5]


def joint_names() -> List[str]:
    return [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6",
    ]


def base_link_name() -> str:
    return "base_link"


def end_effector_name() -> str:
    return "end_effector"


def gripper_joint_names() -> List[str]:
    return [
        "gripper_joint",
    ]
