"""The scene and the object as read from the observation and the simulator (R4)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pybullet as p


@dataclass(frozen=True)
class ObjectState:
    position: np.ndarray        # (3,) world
    orientation: np.ndarray     # (4,) xyzw
    cloud: np.ndarray           # (N, 3) world
    normals: np.ndarray         # (N, 3) world, outward, unit length

    @classmethod
    def from_obs(cls, obs) -> "ObjectState":
        normals = np.asarray(obs["object_normals"], dtype=float)
        length = np.linalg.norm(normals, axis=1, keepdims=True)
        return cls(position=np.asarray(obs["object_pos"], dtype=float),
                   orientation=np.asarray(obs["object_xyzw"], dtype=float),
                   cloud=np.asarray(obs["object_pc"], dtype=float),
                   normals=normals / np.where(length > 0, length, 1.0))

    @property
    def bottom(self) -> float:
        """Height of the cloud's lowest point."""
        return float(self.cloud[:, 2].min())

    @property
    def centre(self) -> np.ndarray:
        """Centre of the cloud's bounding box, a stand-in for the centre of mass."""
        return (self.cloud.min(axis=0) + self.cloud.max(axis=0)) / 2.0


@dataclass(frozen=True)
class Box:
    """A fixed, axis-aligned obstacle other than the maze walls."""

    lower: np.ndarray           # (3,)
    upper: np.ndarray           # (3,)


@dataclass(frozen=True)
class World:
    wall_centres: np.ndarray    # (M, 2) world xy of the maze cubes
    wall_size: float            # edge of one cube, metres
    obstacles: tuple[Box, ...]  # fixed bodies that are not maze walls and not the floor (the table)
    floor: float                # height of the floor
    goal: np.ndarray            # (3,)

    @classmethod
    def from_env(cls, env, obs) -> "World":
        """Read the walls, the other fixed bodies and the goal. Sizes are
        measured, not assumed; the env's simulator is only read."""
        client = env.pb_physics_client
        cubes = np.asarray(obs["cube_positions"], dtype=float).reshape(-1, 2)
        size = _pitch(cubes)
        obstacles, floor = [], 0.0
        for index in range(p.getNumBodies(physicsClientId=client)):
            body = p.getBodyUniqueId(index, physicsClientId=client)
            if body in (env.robot_id, env.object_id):
                continue
            try:
                if not p.getCollisionShapeData(body, -1, physicsClientId=client):
                    continue
            except p.error:
                continue
            lower, upper = (np.array(v) for v in p.getAABB(body, physicsClientId=client))
            span = upper - lower
            if len(cubes) and _covers(lower, upper, cubes):
                continue                                    # the maze: described by the cubes
            if span[0] > 50.0 and span[1] > 50.0:
                floor = float(upper[2]) if upper[2] < 0.05 else floor
                continue                                    # the ground plane
            obstacles.append(Box(lower, upper))
        return cls(wall_centres=cubes, wall_size=size, obstacles=tuple(obstacles), floor=floor,
                   goal=np.asarray(obs["goal_pos"], dtype=float))

    def support_under(self, point) -> float:
        """Height of the surface directly below `point`: the top of the highest
        obstacle under it, or the floor."""
        best = self.floor
        for box in self.obstacles:
            inside = np.all(box.lower[:2] <= point[:2]) and np.all(point[:2] <= box.upper[:2])
            if inside and box.upper[2] <= point[2] + 0.02:
                best = max(best, float(box.upper[2]))
        return best


def _pitch(cubes: np.ndarray) -> float:
    """Edge of one maze cube: the smallest spacing between cube centres."""
    if len(cubes) < 2:
        return 1.0
    gaps = np.abs(cubes[:, None, :] - cubes[None, :, :]).max(axis=2)
    return float(gaps[gaps > 1e-6].min())


def _covers(lower, upper, cubes: np.ndarray) -> bool:
    """True if the box spans the whole set of cube centres: it is the maze."""
    return bool(np.all(lower[:2] <= cubes.min(axis=0) + 1e-6) and np.all(upper[:2] >= cubes.max(axis=0) - 1e-6))


class RestDetector:
    """Tells when the object has stopped moving."""

    def __init__(self, ticks: int = 12, distance: float = 0.0005, angle: float = 0.005):
        """At rest after `ticks` consecutive ticks in which the object moved
        less than `distance` metres and turned less than `angle` radians."""
        self._need, self._distance, self._angle = ticks, distance, angle
        self._last = None
        self.still = 0

    def update(self, obj: ObjectState) -> bool:
        if self._last is not None:
            moved = np.linalg.norm(obj.position - self._last.position)
            dot = abs(float(np.dot(obj.orientation, self._last.orientation)))
            turned = 2.0 * np.arccos(min(1.0, dot))
            self.still = self.still + 1 if moved < self._distance and turned < self._angle else 0
        self._last = obj
        return self.still >= self._need
