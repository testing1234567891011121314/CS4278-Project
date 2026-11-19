# Failure Modes

Observed outcomes of the grasp pipeline, kept separate from the bug list.
Source: `runs/20261111_202152_6415fb11` (pre-delay) and
`runs/20261111_205416_66b6bb65` (post-delay, 200-cycle spawn wait).
Analysis scripts: `analyze_runs.py`, `analyze_outcomes.py`,
`analyze_park_distance.py`.

## F-1 Grasp executes but never secures the object (dominant, 10/15)

Plan found, arm travels, fingers open and close — but the object's z never
rises in any trial (`obj_z_rise = 0.000` across all 15). The gripper closes
beside/around the object without bilateral pad contact; follower fails and
the run dies in state 4 (arm frozen extended). Contacts peak at 3–13 per
trial, so there is touching but no stable bilateral grip.
Suspects: TCP/pad calibration, descent clearance, approach direction.

## F-2 Arm bats the object away during approach (5/15 pre-delay, 2/15 post)

Object slides 0.5–2.9 m mid-grasp (pre-delay: can 1.56 m, apple 1.74 m,
mug 2.89 m; post-delay: marker 0.55 m, mug knocked 2.07 m off the table
onto the floor — the bounce-off-table case). The descending forearm/gripper
sweeps light objects before the pads arrive.

## F-3 Base never turns to face the object (all trials)

`qpos[2]` stays at spawn yaw (0) at park. Parking annulus positions the
base but never aligns it, so the object sits anywhere from dead-front
(+0.43) to 45°-lateral (apple +0.62) and the antipodal approach axis is
arbitrary relative to the base. Vertical-axis rotation is unimplemented.

## F-4 Grasp-park stand-off is inconsistent: 0.40–0.93 m (all trials)

The wall-projection (arm-inflated) occupancy grid plus the 0.25–0.90 m
annulus parks the base anywhere in a wide band. Can/bottle/mug/grasp-attempt
trials at 0.82–0.93 m are at (or beyond) the edge of comfortable reach;
closer cells may be blocked by the inflated table footprints. Two candidate
fixes under discussion:
(a) use a base-only grid for the grasp-park annulus (arm-projection kept
only for maze transit), letting the plan-time collision oracle veto bad
cells via the blacklist-retry loop;
(b) keep the inflated grid but shrink the annulus toward the reachable band.

## F-5 Flat/wide objects stall forever (bowl, plate: 1000-step cap)

Bowl and plate never enter state 4 and never progress: arm range ≈0.15–0.22
rad over 1000 steps with fingers cycling. Antipodal candidates for flat
objects (plate 0.005 m thick; bowl rim) either don't survive pad checks or
the follower never confirms bilateral contact — the run sits in the
settle/plan/hold cycle instead of failing forward.

## F-6 Pre-delay batch: object planned against while airborne

Without the 200-cycle delay the object was still at z≈5 during early
planning: 13/15 trials errored (state-4 class return) or targeted an
airborne goal. Fixed for batch 2 by the spawn-wait state (`state 999`).
