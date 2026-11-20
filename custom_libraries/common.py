"""Small types shared by the grasping and navigation code."""
from __future__ import annotations

import time
from enum import Enum


class Status(Enum):
    """What a tick-by-tick follower reports each tick."""

    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"


class Deadline:
    """A CPU-time budget for one planning call (R26).

    CPU time, not wall time: a busy machine must not change what gets planned.
    """

    def __init__(self, seconds: float):
        self._end = time.process_time() + seconds

    def expired(self) -> bool:
        return time.process_time() >= self._end

    def remaining(self) -> float:
        return max(0.0, self._end - time.process_time())
