"""Grasp candidates from the object's point cloud and surface normals (R7).

A candidate says where the gripper would hold the object; it does not say
whether the arm can get there. Only the cloud, the normals, the gripper's
dimensions and the height of the supporting surface are used. Nothing here is
random: the same cloud gives the same candidates (R24).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from custom_libraries.planning_sim import Gripper
from custom_libraries.world import ObjectState


@dataclass(frozen=True)
class CandidateSettings:
    cone: float = np.radians(15.0)      # how far a surface normal may lean from the closing line
    min_width: float = 0.004            # metres between contacts; below this the cloud is one surface
    clearances: tuple = (0.012, 0.008, 0.006, 0.004)   # gap between a pad and the object before closing, per
                                        # side: the widest that fits is used
    palm_margin: float = 0.004          # least gap between the palm and the object
    tip_margin: float = 0.003           # least height of the fingers above the supporting surface
    tip_inset: float = 0.006            # the contact stays at least this far from the fingertips
    max_pairs: int = 800                # contact pairs kept after thinning
    rolls: int = 12                     # approach directions tried around each closing line
    per_pair: int = 2                   # approach directions kept per pair
    cell: float = 0.015                 # pairs whose centres share a cell this size are thinned to the best
    wall_reach: float = 0.012           # how far apart points of a thin wall's two faces may lie, metres
    max_wall: float = 0.012             # thickest wall treated as thin
    wall_quality: float = 0.5           # a thin wall's place among the pairs' alignment scores (0 to 1)
    tight_quality: float = 0.3          # a tight fit's alignment score is scaled by this
    central: str = "centre"             # what a grasp should be near: "centre", the middle of the bounding box
                                        # (as the pick was verified); "mass", the closing line passes the centre
                                        # of mass (tried for the carry's sake and withdrawn, GRASP.md section 8)
    rake_gap: float = 0.006             # a raking hand starts this far beyond the object's far edge, metres
    rake_height: float = 0.006          # ... with its fingertips this far above the support
    rake_leans: tuple = (0.0, np.radians(20.0), np.radians(40.0), np.radians(60.0))   # how far the raking
                                        # hand may lean back toward the puller


@dataclass(frozen=True)
class GraspCandidate:
    kind: str                    # "antipodal" (two opposed cloud points), "tight" (the same, but as wide as
                                 # the open hand), "rim" (across a thin wall) or "rake" (no grasp: the open
                                 # hand behind the object, to pull it along)
    centre: np.ndarray           # (3,) world: the point between the two contacts
    closing_axis: np.ndarray     # (3,) unit: from one pad to the other
    approach_axis: np.ndarray    # (3,) unit: from the palm toward the object
    width: float                 # metres between the contacts
    opening: float               # clear gap between the pads before they close
    offset: float                # where along the pads the contacts sit: 0 is the pads' middle, positive toward the tips
    score: float                 # higher is better

    def gripper_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """World position and 3x3 rotation of the gripper frame (x along the
        fingers, y from pad to pad) that puts the pads on the contacts."""
        turn = np.column_stack([self.approach_axis, self.closing_axis,
                                np.cross(self.approach_axis, self.closing_axis)])
        return self.centre - self.approach_axis * self.offset, turn


def antipodal_pairs(obj: ObjectState, max_width: float, settings: CandidateSettings):
    """Index pairs (i, j) of cloud points that two flat pads could pinch: the
    outward normals at both points lie within the cone around the line between
    them, and they are between the least and the greatest width apart.
    Returns the indices and each pair's alignment (cosine of the worse lean)."""
    points, normals = obj.cloud, obj.normals
    gap = points[None, :, :] - points[:, None, :]            # gap[i, j] = points[j] - points[i]
    distance = np.linalg.norm(gap, axis=2)
    i, j = np.nonzero(np.triu((distance >= settings.min_width) & (distance <= max_width), k=1))
    line = gap[i, j] / distance[i, j][:, None]
    lean_i = -np.einsum("ij,ij->i", normals[i], line)        # the normal at i points away from j
    lean_j = np.einsum("ij,ij->i", normals[j], line)
    alignment = np.minimum(lean_i, lean_j)
    keep = alignment >= np.cos(settings.cone)
    return i[keep], j[keep], alignment[keep]


def thin_walls(obj: ObjectState, settings: CandidateSettings):
    """Places where the object is a thin wall (a rim, a flange): cloud points
    that have, close by, points of the wall's other face (normals opposed, a
    little way behind). A sparse cloud has no two points exactly opposite each
    other across such a wall, so the far contact is placed straight through
    it. Returns the contact centres, the closing lines and the wall's
    thickness at each."""
    points, normals = obj.cloud, obj.normals
    gap = points[None, :, :] - points[:, None, :]
    distance = np.linalg.norm(gap, axis=2)
    depth = -np.einsum("ijk,ik->ij", gap, normals)           # how far j lies behind i's surface
    other = (distance < settings.wall_reach) & (normals @ normals.T < -0.7) & \
            (depth > 0.0) & (depth <= settings.max_wall)
    index = np.nonzero(other.any(axis=1))[0]
    thickness = np.array([np.median(depth[k][other[k]]) for k in index])
    return points[index] - normals[index] * (thickness / 2.0)[:, None], -normals[index], thickness


def antipodal_candidates(obj: ObjectState, gripper: Gripper, support, limit: int = 60,
                         settings: CandidateSettings = CandidateSettings()) -> list[GraspCandidate]:
    """Pinch grasps on the cloud, best first, at most `limit`: pairs of
    opposed points (including pairs only as wide as the fully open hand, which
    the fingers go round with no room to spare), and thin walls. `support` is
    the height of the surface the fingers must clear: a number, or a function
    of a world (x, y) for an object that overhangs the edge of what it rests on."""
    support_at = support if callable(support) else (lambda xy: support)
    points = obj.cloud
    roomy = gripper.max_opening - 2.0 * settings.clearances[-1]    # widest the fingers go round with room
    i, j, alignment = antipodal_pairs(obj, gripper.max_opening, settings)
    lines = points[j] - points[i]
    widths = np.linalg.norm(lines, axis=1)
    lines = lines / np.where(widths > 0, widths, 1.0)[:, None]
    aligned = (alignment - np.cos(settings.cone)) / (1.0 - np.cos(settings.cone))
    wall_centres, wall_lines, wall_widths = thin_walls(obj, settings)

    centres = np.vstack([(points[i] + points[j]) / 2.0, wall_centres])
    lines = np.vstack([lines, wall_lines])
    tight = widths > roomy                                          # fits the open hand with no room to spare
    aligned = np.where(tight, aligned * settings.tight_quality, aligned)
    kinds = ["tight" if t else "antipodal" for t in tight] + ["rim"] * len(wall_widths)
    aligned = np.concatenate([aligned, np.full(len(wall_widths), settings.wall_quality)])
    widths = np.concatenate([widths, wall_widths])
    if len(centres) == 0:
        return []
    # Where the centre of mass lies relative to the closing line. The object
    # can turn freely about that line: a centre of mass hanging below it is
    # stable, one to the side swings the object round, one above tips it over.
    off = obj.position - centres
    off = off - np.einsum("ij,ij->i", off, lines)[:, None] * lines          # from the line to the centre of mass
    level = np.cross(lines, [0.0, 0.0, 1.0])                                # horizontal, across the line
    flat = np.linalg.norm(level, axis=1)
    upright_line = flat < 1e-6                                              # a vertical line has no "beside"
    level = level / np.where(upright_line, 1.0, flat)[:, None]
    beside = np.where(upright_line, np.linalg.norm(off[:, :2], axis=1), np.abs(np.einsum("ij,ij->i", off, level)))
    above = np.where(upright_line, 0.0, np.maximum(0.0, off[:, 2]))
    if settings.central == "mass":
        central = np.exp(-beside / 0.02) * np.exp(-above / 0.03)
    else:                                                                   # nearness to the middle of the object
        central = np.exp(-np.linalg.norm(centres - obj.centre, axis=1) / 0.04)
    quality = 0.6 * aligned + 0.4 * central

    # Thin the contacts: the best of each cell of centre position and line direction.
    flipped = np.where(lines[:, [int(np.argmax(np.abs(lines).sum(axis=0)))]] < 0, -lines, lines)
    keys = np.concatenate([np.round(centres / settings.cell), np.round(flipped / 0.5)], axis=1).astype(int)
    best: dict[tuple, int] = {}
    for k in np.argsort(-quality, kind="stable"):
        best.setdefault(tuple(keys[k]), int(k))
    chosen = sorted(best.values(), key=lambda k: (-quality[k], k))[:settings.max_pairs]

    found = []
    angles = np.arange(settings.rolls) * (2.0 * np.pi / settings.rolls)
    for k in chosen:
        first, second = _across(lines[k])
        options = []
        for clearance in settings.clearances:
            opening = min(gripper.max_opening, widths[k] + 2.0 * clearance)
            if options and opening >= options[-1][3] - 1e-9:
                break                                   # the hand is already fully open: nothing new to try
            for angle in angles:
                approach = np.cos(angle) * first + np.sin(angle) * second
                offset = _contact_offset(points - centres[k], approach, lines[k],
                                         centres[k][2] - support_at(centres[k][:2]),
                                         opening, widths[k], gripper, settings)
                if offset is None:
                    continue
                down = (1.0 - approach[2]) / 2.0
                deep = 1.0 - offset / (gripper.pad_tip - settings.tip_inset)
                score = 0.35 * aligned[k] + 0.25 * central[k] + 0.25 * down + 0.15 * deep
                options.append((score, approach, offset, opening))
            # A pair of opposed points is as wide as the cloud says, so the
            # widest opening that fits is taken. A thin wall's true thickness
            # is not in the cloud: every opening is offered, and the check of
            # the hand against the real shape decides.
            if options and kinds[k] != "rim":
                break
        options.sort(key=lambda o: (-o[3], -o[0]))
        keep = settings.per_pair * (len(settings.clearances) if kinds[k] == "rim" else 1)
        for score, approach, offset, opening in options[:keep]:
            found.append(GraspCandidate(kinds[k], centres[k].copy(), lines[k].copy(), approach,
                                        float(widths[k]), float(opening), float(offset), float(score)))
    found.sort(key=lambda c: -c.score)
    return found[:limit]


def rake_candidates(obj: ObjectState, gripper: Gripper, support: float, toward,
                    settings: CandidateSettings = CandidateSettings()) -> list[GraspCandidate]:
    """Where the open hand goes to rake the object along the horizontal
    direction `toward`: just beyond the object's far edge, the two fingers
    side by side across the direction of the pull, their tips a little above
    the support. The hand points straight down, or leans back toward the
    puller, which needs less reach. Not grasps: nothing is gripped."""
    toward = np.asarray(toward, dtype=float)[:2]
    toward = np.array([*(toward / np.linalg.norm(toward)), 0.0])
    reach = (obj.cloud - obj.centre) @ toward
    far = float(reach.min())                               # the object's far edge, measured along the pull
    behind = far - gripper.pad_width / 2.0 - settings.rake_gap
    tip = np.array([obj.centre[0], obj.centre[1], support + settings.rake_height]) + behind * toward
    across = np.cross([0.0, 0.0, -1.0], toward)
    width = float(np.ptp((obj.cloud - obj.centre) @ across))
    opening = min(gripper.max_opening, max(0.04, 0.6 * width))
    found = []
    for lean in settings.rake_leans:
        pointing = np.array([0.0, 0.0, -np.cos(lean)]) - np.sin(lean) * toward    # down and away from the puller
        found.append(GraspCandidate("rake", tip, across, pointing, 0.0, float(opening), float(gripper.pad_tip),
                                    -float(lean)))
    return found


def _across(line: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors at right angles to `line` and to each other. The first
    points as far down as it can, so angle zero is the approach from above."""
    down = np.array([0.0, 0.0, -1.0])
    first = down - np.dot(down, line) * line
    if np.linalg.norm(first) < 1e-6:                # the line is vertical: any horizontal will do
        first = np.cross(line, [1.0, 0.0, 0.0])
    first = first / np.linalg.norm(first)
    return first, np.cross(line, first)


def _contact_offset(relative: np.ndarray, approach: np.ndarray, line: np.ndarray, height: float, opening: float,
                    width: float, gripper: Gripper, settings: CandidateSettings):
    """How far along the pads the contacts can sit for this approach: the
    position nearest the pads' middle at which the palm stays outside the
    object, the fingers stay above the support and no part of the object is
    inside a finger. None if there is no such position.

    `relative` is the cloud relative to the contact centre; `height` is the
    centre's height above the support."""
    side = np.cross(approach, line)
    along, across, lateral = relative @ approach, relative @ line, relative @ side

    # The object reaches this far back toward the palm, within the palm's outline.
    under_palm = (np.abs(across) <= gripper.palm_half[0]) & (np.abs(lateral) <= gripper.palm_half[1])
    back = float(max(0.0, -along[under_palm].min())) if under_palm.any() else 0.0
    least = back + settings.palm_margin + gripper.palm_front

    # Cloud points in the band the fingers occupy across and sideways. The
    # surface being gripped is not in the way, however tight the fit.
    outer = opening / 2.0 + gripper.pad_thickness
    inner = max(opening / 2.0 - 0.001, width / 2.0 + 0.002)
    in_band = (np.abs(across) >= inner) & (np.abs(across) <= outer + 0.001) & \
              (np.abs(lateral) <= gripper.pad_width / 2.0 + 0.002)
    band = along[in_band]

    spread = outer * abs(line[2]) + gripper.pad_width / 2.0 * abs(side[2])
    last = gripper.pad_tip - settings.tip_inset
    for offset in np.arange(0.0, last + 1e-9, 0.004):
        if offset < least:
            continue
        ends = np.array([gripper.pad_back - offset, gripper.pad_tip - offset])   # the pads, along the approach
        if height + (ends * approach[2]).min() - spread < settings.tip_margin:
            continue
        if band.size and np.any((band >= ends[0] - 0.001) & (band <= ends[1] + 0.001)):
            continue
        return float(offset)
    return None
