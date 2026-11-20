"""The scene and the object as read from the observation and the simulator (R4)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ObjectState:
    position: np.ndarray        # (3,) world
    orientation: np.ndarray     # (4,) xyzw
    cloud: np.ndarray           # (N, 3) world
    normals: np.ndarray         # (N, 3) world, outward

    @classmethod
    def from_obs(cls, obs) -> "ObjectState":
        raise NotImplementedError("segment 1")


@dataclass(frozen=True)
class World:
    wall_centres: np.ndarray    # (M, 2) world xy of the maze cubes
    wall_size: float            # edge of one cube, metres
    wall_height: float
    table_box: np.ndarray       # (2, 3) lower and upper corner of the table
    table_top: float            # height of the tabletop's upper face
    goal: np.ndarray            # (3,)

    @classmethod
    def from_env(cls, env, obs) -> "World":
        """Read the walls, the table and the goal. Sizes are measured, not assumed."""
        raise NotImplementedError("segment 1")


class RestDetector:
    """Tells when the object has stopped moving."""

    def update(self, obj: ObjectState) -> bool:
        """Feed one tick's object state; True once it has been still long enough."""
        raise NotImplementedError("segment 1")


def support_of(world: World, obj: ObjectState) -> str:
    """Where the object rests: "table" or "floor"."""
    raise NotImplementedError("segment 1")
