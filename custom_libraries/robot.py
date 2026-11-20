"""The robot as read from the simulator: joints, limits, link frames, gripper size.

Nothing here is a scene constant (R4). `RobotModel.from_env` reads everything
once; the rest of the code asks the model.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotModel:
    body_id: int
    client_id: int
    joint_indices: tuple[int, ...]      # simulator joint index of each action entry
    lower: np.ndarray                   # (15,) joint limits
    upper: np.ndarray
    wraps: np.ndarray                   # (15,) bool: the env wraps this target to the nearest turn
    base: tuple[int, int, int]          # action indices of base x, y, yaw
    torso: int
    head: tuple[int, int]
    arm: tuple[int, ...]                # the 7 arm joints, shoulder to wrist
    fingers: tuple[int, int]
    gripper_link: int
    finger_links: tuple[int, int]
    base_origin: np.ndarray             # world (x, y, yaw) of the base when its joints are zero
    max_opening: float                  # clear gap between the pads, fully open, metres
    pad_size: tuple[float, float, float]  # pad length, width, thickness, metres

    @classmethod
    def from_env(cls, env) -> "RobotModel":
        """Read the joint layout, limits, base origin and gripper geometry."""
        raise NotImplementedError("segment 1")

    def base_pose(self, qpos) -> np.ndarray:
        """World (x, y, yaw) of the base for the joint values `qpos`."""
        raise NotImplementedError("segment 1")

    def link_pose(self, qpos, link: int) -> tuple[np.ndarray, np.ndarray]:
        """Forward kinematics: world position and xyzw quaternion of `link`."""
        raise NotImplementedError("segment 2")

    def gripper_pose(self, qpos) -> tuple[np.ndarray, np.ndarray]:
        """World pose of the point between the pads."""
        raise NotImplementedError("segment 2")

    def clip(self, action) -> np.ndarray:
        """`action` limited to the joint limits, as the env will limit it."""
        raise NotImplementedError("segment 1")
