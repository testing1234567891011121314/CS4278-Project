"""Clearance queries for the arm and base planners.

Answered in the planning simulator, never in the env's (R30, R31).
"""
from __future__ import annotations

from typing import Optional

import numpy as np
import pybullet as p

from custom_libraries.planning_sim import PlanningSim, rotation

REACH = 1.6       # metres from the base within which fixed bodies can touch the robot


class CollisionChecker:
    def __init__(self, sim: PlanningSim, look: float = 0.06):
        """`look` is the distance in metres beyond which a gap is reported as
        simply `look`: nothing needs to know how far a far thing is."""
        self.sim, self.look = sim, look
        robot, client = sim.robot, sim.client
        count = p.getNumJoints(sim.robot_id, physicsClientId=client)
        parent = {j: p.getJointInfo(sim.robot_id, j, physicsClientId=client)[16] for j in range(count)}
        first_arm = robot.joint_indices[robot.arm[0]]

        def on_arm(link: int) -> bool:
            while link >= 0:
                if link == first_arm:
                    return True
                link = parent[link]
            return False

        self.arm_links = frozenset(j for j in range(count) if on_arm(j))
        self.hand_links = frozenset({robot.gripper_link, *robot.finger_links})
        # Arm links from the elbow on can reach the robot's own body.
        elbow = robot.joint_indices[robot.arm[3]]
        distal = [j for j in sorted(self.arm_links) if j >= elbow]
        body = [j for j in range(count) if j not in self.arm_links and self._has_shape(j)]
        self.self_pairs = [(a, b) for a in distal for b in body]
        self._near = list(sim.static_ids)
        self._held: Optional[tuple[np.ndarray, np.ndarray]] = None    # object pose in the gripper frame

    def _has_shape(self, link: int) -> bool:
        try:
            return bool(p.getCollisionShapeData(self.sim.robot_id, link, physicsClientId=self.sim.client))
        except p.error:
            return False

    # ------------------------------------------------------------------ set-up

    def focus(self, base_xy) -> None:
        """Only fixed bodies within the robot's reach of `base_xy` are checked
        until the next call."""
        near = []
        for body in self.sim.static_ids:
            lower, upper = p.getAABB(body, physicsClientId=self.sim.client)
            gap = np.maximum(0.0, np.maximum(np.subtract(lower[:2], base_xy), np.subtract(base_xy, upper[:2])))
            if np.linalg.norm(gap) < REACH:
                near.append(body)
        self._near = near

    @property
    def near(self) -> list[int]:
        """The fixed bodies being checked (see `focus`)."""
        return list(self._near)

    def hold(self, qpos, object_position, object_orientation) -> None:
        """From now on the object moves with the gripper, keeping the pose it
        has relative to the gripper at `qpos`."""
        position, turn = self.sim.gripper_pose(qpos)
        self._held = (turn.T @ (np.asarray(object_position, dtype=float) - position),
                      turn.T @ rotation(object_orientation))

    def release(self) -> None:
        self._held = None

    @property
    def holding(self) -> bool:
        return self._held is not None

    # ----------------------------------------------------------------- queries

    def clearances(self, qpos, with_object: bool = True) -> dict:
        """Smallest gaps in metres at `qpos`, each capped at `look`:
        scene   the robot against the fixed bodies (the floor only against the arm)
        own     the forearm and hand against the robot's own body
        object  the robot against the object (without the hand, if it is held)
        load    the held object against the fixed bodies
        """
        sim, look = self.sim, self.look
        sim.set_robot(qpos)
        out = dict(scene=look, own=look, object=look, load=look)
        for body in self._near:
            for c in p.getClosestPoints(sim.robot_id, body, look, physicsClientId=sim.client):
                if body == sim.floor_id and c[3] not in self.arm_links:
                    continue                        # the base stands on the floor
                out["scene"] = min(out["scene"], c[8])
        for a, b in self.self_pairs:
            for c in p.getClosestPoints(sim.robot_id, sim.robot_id, look, linkIndexA=a, linkIndexB=b,
                                        physicsClientId=sim.client):
                out["own"] = min(out["own"], c[8])
        if self._held is not None:
            position, turn = sim.link_pose(sim.robot.gripper_link)
            turn = rotation(turn)
            sim.set_object(position + turn @ self._held[0], _quaternion(turn @ self._held[1]))
            for body in self._near:
                for c in p.getClosestPoints(sim.object_id, body, look, physicsClientId=sim.client):
                    out["load"] = min(out["load"], c[8])
        if with_object:
            for c in p.getClosestPoints(sim.robot_id, sim.object_id, look, physicsClientId=sim.client):
                if self._held is not None and c[3] in self.hand_links:
                    continue
                out["object"] = min(out["object"], c[8])
        return out

    def is_free(self, qpos, margin: float, object_margin: Optional[float] = None,
                load_margin: Optional[float] = None) -> bool:
        """True if every gap at `qpos` is at least its margin. `object_margin`
        None skips the object; `load_margin` None skips the held object
        against the scene (it starts in contact with what it rests on)."""
        gaps = self.clearances(qpos, with_object=object_margin is not None)
        if gaps["scene"] < margin or gaps["own"] < margin:
            return False
        if object_margin is not None and gaps["object"] < object_margin:
            return False
        return load_margin is None or gaps["load"] >= load_margin

    def path_is_free(self, path: np.ndarray, margin: float, object_margin: Optional[float] = None,
                     load_margin: Optional[float] = None, step: float = 0.03) -> bool:
        """True if every configuration along the (K, 15) `path`, sampled so no
        joint moves more than `step` between samples, is free."""
        path = np.asarray(path, dtype=float)
        for a, b in zip(path[:-1], path[1:]):
            pieces = max(1, int(np.ceil(np.abs(b - a).max() / step)))
            for t in np.linspace(0.0, 1.0, pieces, endpoint=False):
                if not self.is_free(a + t * (b - a), margin, object_margin, load_margin):
                    return False
        return self.is_free(path[-1], margin, object_margin, load_margin)


def _quaternion(matrix: np.ndarray) -> np.ndarray:
    """xyzw quaternion of a 3x3 rotation matrix."""
    m = matrix
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(m)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k])
        q = [0.0, 0.0, 0.0, (m[k, j] - m[j, k]) / s]
        q[i], q[j], q[k] = 0.25 * s, (m[j, i] + m[i, j]) / s, (m[k, i] + m[i, k]) / s
    return np.array(q)
