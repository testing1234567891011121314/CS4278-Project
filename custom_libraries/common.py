"""Small types shared by the grasping and navigation code."""
from __future__ import annotations

from enum import Enum


class Status(Enum):
    """What a tick-by-tick follower reports each tick."""

    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"


class Budget:
    """An amount of planning work allowed to one planning call (R26).

    Counted in cycles of work done, never in seconds: one step is one
    iteration of the arm solver, and a tree search or a parking search spends
    a few steps for each configuration it checks. The same inputs therefore
    give the same plan on any machine, however busy it is (R24). A time limit
    did not: the same seed picked the plate in one run and not in the next.
    """

    def __init__(self, steps: int, parent: "Budget" = None):
        self.steps = int(steps)
        self.spent = 0
        self._parent = parent

    def spend(self, steps: int = 1) -> None:
        self.spent += steps
        if self._parent is not None:
            self._parent.spend(steps)

    def expired(self) -> bool:
        return self.spent >= self.steps or (self._parent is not None and self._parent.expired())

    def part(self, steps: int) -> "Budget":
        """A smaller budget inside this one: work spent on it counts against both."""
        return Budget(steps, self)
