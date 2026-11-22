"""The robot's joint layout as read from the env and its simulator (R4).

`RobotModel.from_env` reads everything once, without writing anything. Link
poses and the gripper's dimensions need joints to be moved, so they come from
the planning simulator (`planning_sim.PlanningSim`), not from here.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pybullet as p

ARM_JOINTS = ("shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
              "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint")


def wrap_angle(a):
    """Angle or array of angles brought into [-pi, pi)."""
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


@dataclass(frozen=True)
class RobotModel:
    body_id: int                        # in the env's simulator
    client_id: int
    joint_indices: tuple[int, ...]      # simulator joint index of each action entry
    joint_names: tuple[str, ...]
    lower: np.ndarray                   # (15,) joint limits
    upper: np.ndarray
    wraps: np.ndarray                   # (15,) bool: unlimited turning joint; the env wraps its target
    base: tuple[int, int, int]          # action indices of base x, y, yaw
    torso: int
    head: tuple[int, int]
    arm: tuple[int, ...]                # the 7 arm joints, shoulder to wrist
    fingers: tuple[int, int]
    gripper_link: int                   # simulator link the fingers are mounted on
    finger_links: tuple[int, int]
    base_origin: np.ndarray             # world (x, y, yaw) of the base when its joints are zero

    @classmethod
    def from_env(cls, env) -> "RobotModel":
        client, body = env.pb_physics_client, env.robot_id
        indices = tuple(int(j) for j in env.joint_indices)
        info = [p.getJointInfo(body, j, physicsClientId=client) for j in indices]
        names = tuple(i[1].decode() for i in info)

        def find(part: str) -> list[int]:
            return [k for k, name in enumerate(names) if part in name]

        fingers = find("finger")
        if len(fingers) != 2 or len(find("torso")) != 1 or len(find("head")) != 2:
            raise ValueError(f"unexpected joint layout: {names}")
        arm = tuple(names.index(name) for name in ARM_JOINTS)
        used = set(fingers) | set(arm) | set(find("torso")) | set(find("head"))
        base = [k for k in range(len(names)) if k not in used]
        kinds = [info[k][2] for k in base]
        if kinds != [p.JOINT_PRISMATIC, p.JOINT_PRISMATIC, p.JOINT_REVOLUTE]:
            raise ValueError(f"expected base joints x, y, yaw, found {[names[k] for k in base]}")

        lower = np.array([i[8] for i in info], dtype=float)
        upper = np.array([i[9] for i in info], dtype=float)
        position, orientation = p.getBasePositionAndOrientation(body, physicsClientId=client)
        yaw = p.getEulerFromQuaternion(orientation)[2]
        return cls(
            body_id=body, client_id=client, joint_indices=indices, joint_names=names,
            lower=lower, upper=upper, wraps=(upper - lower) > 4.0 * np.pi,
            base=(base[0], base[1], base[2]), torso=find("torso")[0], head=(find("head")[0], find("head")[1]),
            arm=arm, fingers=(fingers[0], fingers[1]),
            gripper_link=int(info[fingers[0]][16]),
            finger_links=(indices[fingers[0]], indices[fingers[1]]),
            base_origin=np.array([position[0], position[1], yaw]),
        )

    @property
    def plan_joints(self) -> tuple[int, ...]:
        """Action indices the arm planner moves: the torso, then the arm."""
        return (self.torso,) + self.arm

    def base_pose(self, qpos) -> np.ndarray:
        """World (x, y, yaw) of the base for the joint values `qpos`."""
        ox, oy, oyaw = self.base_origin
        x, y, yaw = (float(qpos[k]) for k in self.base)
        c, s = np.cos(oyaw), np.sin(oyaw)
        return np.array([ox + c * x - s * y, oy + s * x + c * y, oyaw + yaw])

    def with_base(self, qpos, pose) -> np.ndarray:
        """A copy of `qpos` with the base joints set for the world pose
        (x, y, yaw). The yaw is taken as the turn nearest the current one."""
        ox, oy, oyaw = self.base_origin
        c, s = np.cos(oyaw), np.sin(oyaw)
        dx, dy = pose[0] - ox, pose[1] - oy
        out = np.array(qpos, dtype=float)
        out[self.base[0]] = c * dx + s * dy
        out[self.base[1]] = -s * dx + c * dy
        current = out[self.base[2]]
        out[self.base[2]] = current + wrap_angle(pose[2] - oyaw - current)
        return out

    def clip(self, action) -> np.ndarray:
        """`action` limited to the joint limits, as the env will limit it."""
        return np.clip(np.asarray(action, dtype=float), self.lower, self.upper)

    def sample(self, rng: np.random.Generator, joints) -> np.ndarray:
        """Random values for the action indices `joints`, inside their limits
        (one full turn for the unlimited joints)."""
        joints = list(joints)
        lo = np.where(self.wraps[joints], -np.pi, self.lower[joints])
        hi = np.where(self.wraps[joints], np.pi, self.upper[joints])
        return rng.uniform(lo, hi)
