"""Drives the base along a path, one target per tick."""
from __future__ import annotations

from typing import Optional

import numpy as np

from custom_libraries.common import Status
from custom_libraries.robot import wrap_angle


class BaseFollower:
    """The env moves the base by position control toward the commanded target
    and covers most of the gap within one tick, so the speed is set by how far
    ahead the target is put. The follower moves a reference pose along the
    path, ramping its speed up and down. The heading turns in proportion to
    the progress along the path, so the pose at every point of the drive is
    known beforehand and can be checked for collisions. One follower per path."""

    def __init__(self, path: np.ndarray, start_yaw: float, final_yaw: Optional[float], max_step: float,
                 max_turn: float, ramp: float, tolerance: float = 0.01, yaw_tolerance: float = 0.02,
                 max_lag: float = 0.15, settle_ticks: int = 40):
        """`max_step` metres and `max_turn` radians are the most the reference
        moves per tick; the step changes by at most `ramp` per tick. The drive
        fails if the base falls more than `max_lag` behind the reference, or
        has not come to rest `settle_ticks` after the reference stopped."""
        self.path = np.asarray(path, dtype=float).reshape(-1, 2)
        self._marks = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.path, axis=0), axis=1))])
        self._start_yaw = float(start_yaw)
        self._turn = 0.0 if final_yaw is None else float(wrap_angle(final_yaw - start_yaw))
        # The drive's length in metres of reference travel: a turn costs as
        # many ticks as the distance the base could cover meanwhile.
        self._span = max(self.length, abs(self._turn) * max_step / max_turn, 1e-9)
        self._travelled, self._step = 0.0, 0.0
        self._max_step, self._ramp = max_step, ramp
        self._tolerance, self._yaw_tolerance, self._max_lag = tolerance, yaw_tolerance, max_lag
        self._settle_ticks, self._waited = settle_ticks, 0
        self.failure: Optional[str] = None

    @property
    def length(self) -> float:
        return float(self._marks[-1])

    def pose_at(self, progress: float) -> np.ndarray:
        """The reference (x, y, yaw) at `progress` from 0 (start) to 1 (end)."""
        along = progress * self.length
        return np.array([np.interp(along, self._marks, self.path[:, 0]),
                         np.interp(along, self._marks, self.path[:, 1]),
                         self._start_yaw + progress * self._turn])

    def poses(self, spacing: float = 0.05) -> np.ndarray:
        """Reference poses along the whole drive, no more than `spacing`
        metres of reference travel apart."""
        count = max(2, int(np.ceil(self._span / spacing)) + 1)
        return np.array([self.pose_at(t) for t in np.linspace(0.0, 1.0, count)])

    def step(self, base_pose) -> tuple[np.ndarray, Status]:
        """The next world (x, y, yaw) target for the base, given its measured
        pose. FAILED if the base cannot keep up or does not come to rest. DONE
        when it rests at the end of the drive."""
        base_pose = np.asarray(base_pose, dtype=float)
        left = self._span - self._travelled
        # Fastest step from which the ramp can still stop at the end.
        stopping = np.sqrt(2.0 * self._ramp * max(left, 0.0))
        self._step = min(self._step + self._ramp, self._max_step, max(stopping, self._ramp))
        self._travelled = min(self._span, self._travelled + self._step)
        target = self.pose_at(self._travelled / self._span)

        lag = float(np.linalg.norm(target[:2] - base_pose[:2]))
        if lag > self._max_lag:
            self.failure = "base fell behind its path"
            return target, Status.FAILED
        if self._travelled >= self._span:
            turned = abs(float(wrap_angle(target[2] - base_pose[2])))
            if lag < self._tolerance and turned < self._yaw_tolerance:
                return target, Status.DONE
            self._waited += 1
            if self._waited > self._settle_ticks:
                self.failure = "base did not come to rest at the end of its path"
                return target, Status.FAILED
        return target, Status.RUNNING
