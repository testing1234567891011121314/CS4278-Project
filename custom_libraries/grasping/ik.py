"""Numerical inverse kinematics for the torso and arm (R3, R8).

Damped least squares on the geometric Jacobian. Link poses are read from the
planning simulator; the solver itself is ours.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pybullet as p

from custom_libraries.common import Budget
from custom_libraries.planning_sim import PlanningSim, rotation
from custom_libraries.robot import wrap_angle


def rotation_vector(matrix: np.ndarray) -> np.ndarray:
    """The rotation `matrix` as axis times angle."""
    cos = np.clip((np.trace(matrix) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos)
    axis = np.array([matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]])
    if angle < 1e-9:
        return axis / 2.0
    if np.pi - angle < 1e-6:                       # half a turn: the axis is in the diagonal
        axis = np.sqrt(np.maximum(0.0, (np.diag(matrix) + 1.0) / 2.0))
        k = int(np.argmax(axis))
        signs = np.sign(matrix[k] + matrix[:, k])
        signs[k] = 1.0
        return axis * signs * angle
    return axis / (2.0 * np.sin(angle)) * angle


def jacobian(sim: PlanningSim, qpos) -> np.ndarray:
    """(6, 8) Jacobian of the gripper frame (linear on top of angular) with
    respect to the torso and arm joints, at `qpos`."""
    robot = sim.robot
    tip, _ = sim.gripper_pose(qpos)
    out = np.zeros((6, len(robot.plan_joints)))
    for column, k in enumerate(robot.plan_joints):
        joint = robot.joint_indices[k]
        info = p.getJointInfo(sim.robot_id, joint, physicsClientId=sim.client)
        origin, turn = sim.link_pose(joint)
        axis = rotation(turn) @ np.array(info[13])
        if info[2] == p.JOINT_REVOLUTE:
            out[:3, column] = np.cross(axis, tip - origin)
            out[3:, column] = axis
        else:
            out[:3, column] = axis
    return out


def solve(sim: PlanningSim, position, turn: np.ndarray, seed_qpos, rng: np.random.Generator,
          budget: Budget, restarts: int = 6, position_tolerance: float = 0.001,
          angle_tolerance: float = 0.005, iterations: int = 80, damping: float = 0.05) -> Optional[np.ndarray]:
    """Joint values, within limits, that put the gripper frame at the world
    `position` with the 3x3 rotation `turn`. The search starts from
    `seed_qpos` and restarts from random arm configurations until `restarts`
    or the work `budget` is spent (one step per iteration). Only the torso and arm move. None if not found."""
    robot = sim.robot
    joints = list(robot.plan_joints)
    limited = ~robot.wraps[joints]
    lower, upper = robot.lower[joints], robot.upper[joints]
    seed = np.array(seed_qpos, dtype=float)
    target = np.asarray(position, dtype=float)
    q = seed.copy()
    for attempt in range(restarts + 1):
        if attempt:
            q = seed.copy()
            q[joints] = robot.sample(rng, joints)
        for _ in range(iterations):
            if budget.expired():
                return None
            budget.spend()
            here, facing = sim.gripper_pose(q)
            error = np.concatenate([target - here, rotation_vector(turn @ facing.T)])
            if np.linalg.norm(error[:3]) < position_tolerance and np.linalg.norm(error[3:]) < angle_tolerance:
                q[joints] = np.where(limited, q[joints], seed[joints] + wrap_angle(q[joints] - seed[joints]))
                return q
            # Bound the step so the linearisation holds.
            error[:3] *= min(1.0, 0.10 / max(np.linalg.norm(error[:3]), 1e-12))
            error[3:] *= min(1.0, 0.50 / max(np.linalg.norm(error[3:]), 1e-12))
            j = jacobian(sim, q)
            step = j.T @ np.linalg.solve(j @ j.T + damping ** 2 * np.eye(6), error)
            q[joints] = np.where(limited, np.clip(q[joints] + step, lower, upper), q[joints] + step)
    return None
