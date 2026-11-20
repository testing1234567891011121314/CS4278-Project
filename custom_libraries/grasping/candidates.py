"""Grasp candidates from the object's point cloud and surface normals (R7).

A candidate says where the gripper would hold the object; it does not say
whether the arm can get there. Only the cloud, the normals and the gripper's
size are used.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from custom_libraries.robot import RobotModel
from custom_libraries.world import ObjectState, World


@dataclass(frozen=True)
class GraspCandidate:
    kind: str                    # "antipodal", or the name of a fallback hold
    centre: np.ndarray           # (3,) world: the point between the two contacts
    closing_axis: np.ndarray     # (3,) unit: from one pad to the other
    approach_axis: np.ndarray    # (3,) unit: from the palm toward the object
    width: float                 # metres between the contacts
    depth: float                 # metres from the fingertips to the centre, along the approach
    score: float                 # higher is better

    def gripper_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """World position and xyzw quaternion of the gripper for this candidate."""
        raise NotImplementedError("segment 2")


def antipodal_candidates(obj: ObjectState, robot: RobotModel, rng: np.random.Generator,
                         limit: int) -> list[GraspCandidate]:
    """Pairs of cloud points whose normals oppose along the line between them,
    inside the friction cone and narrower than the open gripper, with several
    approach directions each. Best first, at most `limit`. Same cloud and
    generator state give the same list (R24)."""
    raise NotImplementedError("segment 1")


def fallback_candidates(obj: ObjectState, world: World, robot: RobotModel, rng: np.random.Generator,
                        limit: int) -> list[GraspCandidate]:
    """Holds for shapes no antipodal pair fits: thin rims, and objects wider
    than the gripper (R13). Chosen from the geometry, never from the object's
    id (R4). Which holds exist is decided by the experiments in docs/plan.md."""
    raise NotImplementedError("segment 2")


def reject_reason(candidate: GraspCandidate, obj: ObjectState, world: World,
                  robot: RobotModel) -> Optional[str]:
    """A cheap check run before any arm planning: None if the candidate may be
    tried, else why not (fingers do not fit, palm inside the object, fingertips
    below the supporting surface)."""
    raise NotImplementedError("segment 2")
