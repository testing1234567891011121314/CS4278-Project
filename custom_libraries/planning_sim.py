"""The solver's own simulator for planning (R30, R31).

A second physics client holding a copy of the scene. Planning code sets poses
here and asks for distances; the env's simulator is only ever read. See
docs/adr/0001-plan-in-a-second-simulator.md.
"""
from __future__ import annotations


class PlanningSim:
    def __init__(self, env):
        """Open a second physics client and load a copy of the env's scene:
        robot, table, maze walls and object, found by reading the env's
        client. Nothing is written to the env's client."""
        raise NotImplementedError("segment 2")

    @property
    def client_id(self) -> int:
        raise NotImplementedError("segment 2")

    @property
    def robot_id(self) -> int:
        raise NotImplementedError("segment 2")

    @property
    def object_id(self) -> int:
        raise NotImplementedError("segment 2")

    def sync(self, obs) -> None:
        """Set the copy's robot joints and object pose to the observation."""
        raise NotImplementedError("segment 2")

    def set_robot(self, qpos) -> None:
        """Place the copy's robot at the joint values `qpos`."""
        raise NotImplementedError("segment 2")

    def set_object(self, position, orientation) -> None:
        """Place the copy's object at a world pose."""
        raise NotImplementedError("segment 2")

    def close(self) -> None:
        """Disconnect the second client."""
        raise NotImplementedError("segment 2")
