# Antipodal Grasp Debugging Journal

Session: 2026-11-01 → 2026-11-02
Scope: `custom_libraries/antipodal.py` (grasp planner), `custom_libraries/navigate_to_grasp.py` (base placement), `solver.py` (state machine).
Goal: pass state 1 (grasp execution) in `pick_nav_reach_env.py` — robot navigates to the YCB can, selects an antipodal grasp, reaches with IK, closes the gripper.

---

## Established ground truth (measured, not assumed)

These measurements settled most disputes. All reproducible with `p.connect(p.DIRECT)` + `assets/fetch/fetch.urdf`:

| Fact | Value | How measured |
|---|---|---|
| Gripper **closes along gripper_link y** | finger origins at y = ±0.015 (closed) → ±0.065 (open), x-offset 0 | FK of `r/l_gripper_finger_link` in gripper frame |
| Finger **pad tips extend to gripper x ≈ +0.04** | collision box: 0.063 long centered at x=+0.01 | `getCollisionShapeData` on finger joint |
| **wrist_flex = −1.57 → gripper x points UP** (backhand) | textbook top-down requires **wrist_flex ≈ +1.5** | FK of pose `[0, 1.3, 0, −1.5, 0, −1.57, 0]`: x-axis = (−0.2, 0, **0.98**) |
| Movable joints | 0,1,2 (root), 5 (torso), 6,7 (head), 13–19 (arm), 21,22 (fingers) | joint dump |
| Arm joints | 13–19 (shoulder_pan/lift, upperarm_roll, elbow, forearm_roll, wrist_flex/roll) | name prefix match |
| "Continuous" joints in this URDF | upperarm/forearm/wrist_roll: type 0, limits **±314** | joint dump |
| Fingers | prismatic 0 → 0.05 | joint dump |
| Object spawns at **z = 5.0**, needs ~200 steps to settle at 0.645 | `object_pc` z-range | probe stepping |
| `calculateInverseKinematics` returns one entry per **movable DOF** (15), regardless of limit list length | — | observed |
| **DLS starts from the current joint state**; `restPoses` is only a null-space spring | identical IK output from 4 different `restPoses` | seed matrix test |
| PyBullet 3.2.7: no `getMultiBodyInfoEx`, no `getNormalAtWorldPosition`, no `JOINT_CONTINUOUS` | — | AttributeError |
| pnr venv: numpy + pybullet only, **no scipy** | — | planner must stay scipy-free |

---

## Bugs found and fixed (in discovery order)

### Bug 1 — IK DOF mapping (pre-session, Astra review)
**Symptom:** IK checks compared a 15-entry solution against 7-entry limit arrays; `sol[:7]` slicing assumed arm-first ordering.
**Fix:** run IK over all movable DOFs; extract arm entries via `arm_pos = [movable_ids.index(j) for j in arm_ids]`; never slice.

### Bug 2 — Arm joints silently pinned (the `-0.9524` mystery)
**Symptom:** `arm joint 14=-0.8702 outside [-0.9524, -0.9524]` — a degenerate 1-point range for a joint whose real limit is [−1.221, 1.518].
**Cause:** limit-filling loop indexed with arm-order `k` into movable-indexed arrays (`lower[k]` instead of `lower[arm_pos[k]]`), so joint 14 never unlocked and was rejected for moving away from its stow value.
**Fix:** index by `arm_pos[k]`. (Note: −0.87 was legal all along — negative values never "broke the sim"; our own check was rejecting valid solutions.)

### Bug 3 — FK-verify read stale poses
**Symptom:** FK-verify passed solutions that were actually wrong.
**Cause:** used `getLinkState(...)[:2]` — the pose from the last `stepSimulation` — instead of `state[4]/state[5]` (the `computeForwardKinematics=True` results). Also used `abs(axis·x)` which let pad-flipped solutions pass.
**Fix:** read `state[4], state[5]`; signed axis check (later changed to abs on the correct axis — see Bug 5, jaws are sign-symmetric).

### Bug 4 — Pinning via `lower==upper==current` cripples DLS
**Symptom:** all candidates fail with saturated joints; identical results from different seeds.
**Evidence:** controlled matrix at a trivially reachable target (d=0.55):
```
A pin(1e-8):  pos_err=1.038  x_axis wrong        ← our original scheme
B free-all:   pos_err=0.610  orientation correct
D damp-pin:   pos_err=0.610  orientation correct (jointDamping=10 on pinned joints)
E pin+2k iter: pos_err=0.635 (not an iteration-budget issue)
```
**Diagnosis:** fake zero-width limits act as hard constraints DLS fights against and diverges. Soft-pinning via `jointDamping` on real limits behaves correctly.
**Fix (planned):** use real URDF limits + `jointDamping≈10` for base/torso/head/fingers.

### Bug 5 — Wrong gripper convention (Gemini's review, confirmed by measurement)
**Symptom:** IK solutions consistently wrapped even at reachable targets; the commanded orientation was unsatisfiable by design.
**Cause:** code assumed *closing along gripper x*: `_contact_orientations` mapped grasp axis → x, and `ee_target = center + 0.10·axis` offset along it. Measured truth: closing along **y**, fingers extend along **x**. So the jaws were aligned perpendicular to the contact line ("contradictory approach/closing axes") and the ee was offset sideways ("gripper offset sideways").
**Fix:** grasp axis → gripper y; approach/descent → gripper x; `ee_target = center − offset·descent` (gripper behind the pads). FK-verify now checks the **y**-axis against the grasp axis (sign-free: parallel jaws are symmetric).
**Also fixed along the way:** `ee_forward_offset=0.10` overestimates — pad tips are at x≈+0.04, so ~0.05 is correct. (Not yet changed.)

### Bug 6 — Scoring inverted: best grasps were table-punching pinches
**Symptom:** even after the convention fix, the top-25 candidates all failed IK with elbow→−3.14 / shoulder_pan→±2.7 saturation.
**Cause:** my "topdown" term was `1 − |axis·ẑ|` — rewarding **vertical closing axes**. A vertical closing axis means jaws above *and below* the can: the lower finger through the table. The base was parked fine (object straight ahead at 1.07 m); the sampler was ranking impossible grasps first.
**Fix:** `closz = |grasp_axis·ẑ|` — prefer horizontal closing (jaws around the sides, approach from above). After the fix the sampler ranks genuine side-pinch candidates first (9 of ~50, top score 0.031, width ~4 cm).

### Bug 7 — Navigation ignores arm reach (review item D, confirmed)
**Symptom:** robot parks "right in front of the object" but arm can't reach; frozen at travelling pose.
**Evidence:** measured base→object = **1.07 m**; Fetch shoulder→gripper reach ≈ 0.81–0.84 m. `nearest_free_goal()` picked the nearest free cell to the object with zero reach awareness.
**Fix (done):** reach annulus (0.25–0.90 m from object) when choosing goal cells; prefer cells nearest the robot; skip blacklisted cells; hard error if every cell is proven un-graspable.

### Bug 8 — State machine enters GRASP without IK input
**Symptom:** arrive→GRASP fires on distance-to-goal alone; IK failure raises `RuntimeError` and kills the run.
**Fix (done):** transition saves the parked goal; on IK failure `execute_grasp` returns `None` → state 1 demotes to 0, cell blacklisted via `mark_goal_failed`, A* picks another annulus cell. Verified in run 4: robot repositions and retries correctly (no crash).

### Bug 9 — `restPoses` seeding does nothing
**Symptom:** "multi-seed IK" produced byte-identical solutions across 4 different `restPoses`.
**Cause:** PyBullet DLS's starting point is the **current joint state**, not `restPoses`. `restPoses` only weights the null-space term.
**Fix (planned):** warm-start by `resetJointState` to the seed before calling IK (proven necessary by the FK→IK self-consistency test, which succeeded exactly because it started near the answer).

### Bug 10 — wrist_flex sign: the last remaining root cause
**Symptom:** every warm-start converged to `elbow≈−3.1, wrist_flex≈−3.1` (wrapped) with gripper x pointing away from straight-down; targets trivially in reach.
**Cause:** all seeds used wrist_flex ≤ 0. FK proved `wrist_flex=−1.57` points gripper x **up**; a top-down grasp needs wrist_flex ≈ **+1.5**. DLS could only achieve "x down" from negative-wrist starts by wrapping through ±π — producing exactly the garbage signatures we kept seeing (`elbow=−3.14`, `wrist_flex=−3.1`). The natural basin was never sampled because it was never seeded correctly.
**Evidence:** positive-wrist seeds: orientation perfect (x = (0,0,−1) exactly), position residual down to **0.032–0.039 m** (from 0.6+ m).

---

## Dead ends (hypotheses killed by evidence, not guesswork)

1. **"Negative values break the sim"** — No. URDF allows them; our own (mis-pinned) check rejected them.
2. **Base facing away / circle-around table** — disproved: base-frame diagnostic showed object straight ahead (x = 1.07, y ≈ 0).
3. **Torso height fixes reach** — disproved: unlocking `torso_lift` (rose to its 0.386 max) still left 0.18 m FK error; parked-at-bottom shoulder height was not the blocker. (Edit reverted.)
4. **Stale `.pyc` / wrong import** — on-disk code verified to carry the arm_pos fix.
5. **Frame mismatch in point cloud** — `object_pc` is mesh-sampled world frame (`canonical_pc @ rot.T + pos`), clean.
6. **DLS needs more iterations** — 200 vs 2000 iterations: same basin.
7. **URDF IK Jacobian broken** — FK→IK→FK self-consistency at the answer: pos_err 0.0000.
8. **Target genuinely unreachable from 0.55 m** — brute-force FK sweep (300 samples) found poses 0.34 m from target with near-right orientation: reachable.
9. **Debug script "hangs"** — stdout buffering to file; replay re-raised inside the loop. Fixed with `-u` and try/except around the replay.

---

## Current state

**Working:** sampler (correct geometry + scoring), navigation (annulus goals), state machine (blacklist + reposition), gripper convention (orientation solved exactly).
**Remaining:** DLS still leaves ~3–4 cm position residual and soft-violates wrist_flex past ±2.16 (over-flexes to +2.89) when the target is close-in (0.28 m from shoulder).

## Fixes applied (by file)

**`custom_libraries/antipodal.py`**
- `sample_antipodal_grasps`: friction cone / antiparallel / point-plane filters verified; scoring now `1.0·lever + 0.8·|closing·ẑ| + 0.4·mid-width` (horizontal closing ranks first).
- `_contact_orientations(axes, approaches)`: grasp axis → gripper **y**, descent → gripper **x**.
- `select_optimal_grasp`: full-DOF IK with `arm_pos` extraction; ee target `center − offset·descent`; FK-verify on `state[4]/state[5]` checking the y-axis; per-check `reasons[]` in `ik_stats`; multi-seed loop (positive-wrist top-down seeds), warm-started via `resetJointState`; one polish re-solve per seed.
- IK inputs: real URDF limits (clamped ±3.14 to tame the ±314 fake-continuous rolls) + `jointDamping` (arm 0.05, everything else 10) instead of `lower==upper==current` pinning.
- `ee_forward_offset` default 0.10 → 0.03 (measured pad tip at gripper x +0.04; 0.10 left the pads floating above the can).

**`custom_libraries/navigate_to_grasp.py`**
- `nearest_free_goal`: reach annulus (`min_reach..max_reach`, 0.25–0.90 m), preference for cells nearest the robot, `failed_goals` blacklist, hard error when every free cell is proven un-graspable.
- `mark_goal_failed(goal)`: blacklist entry point.

**`solver.py`**
- `get_action` state 0: passes `self.cell_size` into `navigate_grasp.update`; saves `grasp_attempt_goal` on transition.
- `execute_grasp`: returns `None` on IK failure (no `RuntimeError`).
- state 1: on `None`, blacklists the parked cell, resets grasp plan, demotes to state 0.

## Suggested future actions

1. **Iterative warm-restart tuning** — if the single polish pass doesn't pull `wrist_flex` back in-limit on close-in targets, iterate 3–5× (reset to solution, re-solve) or raise `maxNumIterations`; alternatively add a mild shoulder-distance penalty by biasing `restPoses`.
2. **A/B `score_all` vs `greedy_first`** (brainstorm decision #9): same seeds, compare grasp success + arm effort. `score_all` costs ~4× IK calls but adds the `joint_damping·‖sol−current‖` min-effort tiebreak.
3. **Grasp-height sanity**: the can is 3.7 cm tall on a table — pads (4 cm reach) can clip the table at closure. Consider rejecting candidates whose `p1/p2` z is within ~0.5 cm of the table, or adding table clearance to `ee_forward_offset`.
4. **Collision-aware candidates**: sampler ignores the table; a candidate pair whose closing sweep intersects the table plane should be scored down.
5. **Clean up probe scripts** (`quick_reach_probe.py`, `debug_ik_once.py`, `finger_check.py`, `gripper_frame_check.py`, `joint_dump.py`) — promote the FK→IK self-consistency and gripper-frame checks into `test_antipodal_smoke.py`, delete the rest (ask before deleting).
6. **Seed the rolls non-zero occasionally** — all roll seeds are 0; a roll≠0 seed may help side-approach grasps around the cylinder.
7. **Verify `NavigateToGrasp.get_base_pose`'s `qpos[0] − 2.0` offset** matches the world frame used for `navigation_goal` (maze-frame vs PyBullet world) — annulus distances are computed in grid/world coordinates.

## Files
- `custom_libraries/antipodal.py` — planner (Bugs 1,2,3,5,6 fixed; 4,9 fixed by Bug 11)
- `custom_libraries/navigate_to_grasp.py` — annulus goals + blacklist (Bug 7)
- `solver.py` — IK-gated state transitions (Bug 8)
- `quick_reach_probe.py`, `sampler_debug.py`, `debug_ik_once.py`, `finger_check.py`, `gripper_frame_check.py`, `joint_dump.py` — throwaway probes (delete or fold into tests when done)
- Logs: `grasp_run{,2,3,4,5}.log`, `probe{1..36}.log`, `dbg{2,3}.log`

---

# Session 2 (2026-11-02/03): from "no reachable grasp" to a verified grasp

## Bugs 11-16 (discovery order)

### Bug 11 — Stock `calculateInverseKinematics` CANNOT be trusted (root cause of the whole saga)
**Evidence (probe 14/17/21):** from every standoff (0.59/0.80/1.07 m) DLS either
(a) pushed `wrist_flex` past its ±2.16 limit (returned 2.164–2.18 soft-violations)
or (b) "solved" an out-of-reach target by **sliding the free base joint 0.8–3.14 m
toward the object** (probe 21: `damp10` and `damp1000` identical — damping does not
stop it; it's a gradient direction, not a limit).
The FK sweep (probe 18, 10k samples, real limits) PROVED the top-down target is
reachable in-limit (e.g. `arm=[-0.4,-0.62,1.13,0.91,-0.65,1.88,-1.74]`,
wrist_flex 1.88, oy=0.98) — the basin exists; the stock solver just cheats instead
of finding it.
**Fix:** hand-rolled `AntipodalGraspPlanner.solve_ik` — Damped Least Squares Newton
over the **7 arm joints only**, numerical Jacobian via FK finite-differences
(PyBullet 3.2.7 has no `getJacobian`), sign `dq = -J⁺·err` (the `+` climbs the
error hill — seen in probe 22), backtracking line search, per-joint clipping.
Base/torso/head/fingers are structurally immobile. **Never call the stock IK again.**
Verified (probe 23): converges to pe=0.0000 oy=1.000 in-limit from sweep seeds.

### Bug 12 — Approach direction could point UPWARD (hand under the table)
**Symptom:** under the env's position motors the descent stalled one finger-length
(0.04 m) above the IK target; contact diagnostics showed table contact at the
tabletop z=0.626.
**Cause:** `descent = obj_com − center` flips UP when the contact pair sits below
the object's mid-height; `ee_target = center − offset·descent` then parked the
wrist below the table surface.
**Fix:** descent is always the downward direction projected ⊥ axis; a vertical
closing axis has no downward approach → reject the candidate outright (jaws above
and below → lower finger through the table).

### Bug 13 — Point cloud contains INTERIOR points (jaws closing on air)
**Symptom:** p1/p2 pairs sat on the same side of the can or inside its silhouette
(both y>0.03 etc.); closing swept through empty space (0 object contacts) or shoved
the can sideways instead of gripping.
**Cause:** the YCB `coacd_decomposed_object_one_link.urdf` mesh carries interior
faces (folded flaps, inner walls); the sampler happily built antipodal pairs from
interior samples.
**Fix (sampler):** outer-shell filter — a point is "surface" only if no sampled
point lies beyond its tangent plane (`(p_j − p_i)·n_i <= outer_tol`, watch the
sign: the flipped version kills everything, probe 33).

### Bug 14 — Top-bottom "grasps" on a 3.7 cm slab
**Symptom:** surviving pairs were lid-vs-base pairs (dz ≈ box height) whose lower
finger would need to slide under the object into the table.
**Fix:** level filter `|dz| <= level_tol·dist` (0.1). Relative, not absolute:
an absolute 0.01 m would kill legitimate pairs after Bug 15's tilt reality.

### Bug 15 — The mesh's cap normals are TILTED ~15°; tight shell/cone tolerances starve the sampler
**Evidence (sampler_debug):** object is a slab x∈[−0.038,0.039] y∈[−0.10,0.10]
z∈[0.622,0.668]; end-cap normals like (0.96,0.25,0.11). With `outer_tol=0.01` the
tilted tangent planes put every higher-y point "beyond" → 0 candidates. Sweeping
`outer_tol×mu` (probe matrix) found `outer_tol=0.03, mu=0.5` yields ~11 pairs /
4 straddling the pinch axis; `mu=1.0` yielded more but fakes friction — rejected.
**Fix:** `outer_tol=0.03` default. Also: straddling check worth adding to scoring
(midpoint near com along the axis) if candidates still drift to one side.

### Bug 16 — Probe-side lessons (not planner bugs)
- `resetJointState` + `stepSimulation` fights the env's position motors: they pull
  toward the last commanded target (this produced the "robot did nothing / started
  at origin" observations). Drive the base with `env.step(target_pose)`, and command
  the fingers OPEN during descent (the solver's `open_pose` does this; a probe that
  left them at 0 closed them mid-air).
- Teleporting the base inside the table gets collision-ejected back — settle via
  motors and verify where it actually stopped (`settled at D`).
- DIRECT mode trips this env's eglRenderer plugin; probes must use `gui=True`.
- GUI sweeps (5k random FK configs) LOOK like the robot flailing at the table —
  harmless (kinematic resets), but warn before running.

## Physics-execution findings (probe 28–36)

- With custom `solve_ik` + support_z depth, the motor-tracked reach pose lands
  within **1 mm** of target (probe 28+: `VISUALIZED ee ≈ target`).
- Gripper does close ON the object (fingers stop at 0.0188 = slab half-width,
  5–6 robot-object contacts) — but the **LIFT still fails** (obj z unchanged):
  contact pair sits at z≈0.649-0.651 (upper slab region; slab top 0.668, bottom
  0.622), jaws clamp the thin top edge; friction can't hold on lift.
  support_z depth computed tips to ~5 mm above table, but the *pair* z (from the
  interior-filtered cloud) is what governs the jaw band — see future actions.
- Closing fast (force 2e6) shoves the light slab out from between the pads
  (obj moved +1.3/+0.7 cm during close). Ramp the close (linspace over ~40 steps)
  — that's now the probe recipe; the solver's 3-waypoint `close_pose` needs the
  same treatment.

## Current status (end of session 2)

**Working:** sampler (outer-shell + level filters tuned to this mesh), custom
`solve_ik` (exact convergence, no base cheat), IK-verified grasp FOUND from parked
base at 0.70 m (probe 36: candidates=2 FOUND in 1.1 s), motor-tracked reach ±1 mm,
fingers close ON the object with contacts.
**NOT yet working:**
1. **LIFT** — grip position sits too high on the 3.7 cm slab; lift slips (Bug 16
   / physics section). Candidate z is interior-cloud-driven even after shell filter
   (outer_tol=0.03 admits near-top points).
2. **Full env run 5 (grasp_run5.log): state 0 never picks a grasp** —
   `NAVIGATE_TO_OBJECT` forever; base wandered to qpos[0]=3.05 (world x≈1.05)
   and `dist_to_goal` 18→17.5 slowly. The grasp is never sampled in the real loop.
   Suspects to check first (next session):
   a. `update()` calls `sample_antipodal_grasps(self.env.object_id, points=...)`;
      with the tuned filters candidates may be 0 at the *initial* (far) viewpoint
      → `grasp is None` → Case 1 forever. But state should still reach arrived→GRASP.
   b. The `arrived` check: base wanders because `nearest_free_goal` keeps changing
      target (annulus + nearest-to-robot flip-flop?) or A* path churns; check
      `navigation_goal` values across steps in the log.
   c. `qpos[0] − 2.0` frame consistency between base_xy and goal_world
      (journal future action #7 — still unverified!).
3. **Solver close is instant** (`close_pose` = one waypoint) — needs the ramped
   close like the probe, else the same shove-and-slip.

## Session-2 fixes applied (by file)

**`custom_libraries/antipodal.py`**
- NEW `solve_ik`: arm-only DLS Newton (num. Jacobian, backtracking, clipping).
  Stock `calculateInverseKinematics` fully removed from the planner.
- `select_optimal_grasp`: uses solve_ik per candidate per seed (5 seeds incl.
  negative-shoulder_lift basin `[0,-0.6,1.1,0.9,-0.7,1.9,-1.7]`); keeps best
  (pos_err, axis_err); `finally` restores joints; `support_z` param chooses
  descent depth (`depth = max(0, ee_forward_offset + support_z + 0.005 − center_z)`)
  so pad tips hug the table instead of rim-only grasps; vertical-axis candidates
  rejected; per-candidate **failure points** stored in
  `ik_stats[strategy]["failures"]` (center, ee_target, pos_err, axis_err_deg, arm)
  and `reasons[]`.
- `_fk_verify` returns `(pos_err, axis_err_deg)` instead of bool (feeds failures).
- Sampler: `outer_shell` filter (outer_tol=0.03), level filter (level_tol=0.1),
  module docstring documents the arm-only IK rationale.

**`GRASP_DEBUG_JOURNAL.md`** — this session appended.

## Session-2 future actions (priority order)
1. Debug grasp_run5 state-0 starvation (suspects a/b/c above; instrument the
   `decision` dict each step: goal_world, candidates count at park).
2. Fix LIFT: score/pref candidates whose `center_z` is low (pinch the slab's mid
   height, not the top edge); consider rejecting pairs with center_z > com_z.
3. Ramp the finger close in `solver.execute_grasp` (multi-waypoint close_pose).
4. A/B score_all vs greedy_first still pending.
5. Fold probe checks into `test_antipodal_smoke.py`; delete probes after asking.

---

# Session 3 (2026-11-03): first live grasp, lift machinery, three blockers isolated

## Progress ladder (run by run)

| Run | Outcome | Lesson |
|---|---|---|
| run5–12 | fixed: falling-object IK poisoning (200-step settle + `object_pos[2]>1.0` deferral), blacklist only on `calls>0`, cached-grasp clear on failure, table-TOP AABB sealing the annulus (`lower[2]>=0.35: continue` in `get_extra_obstacle_bounds`) | never plan against the z=5 spawn; 2-D base map must ignore slabs above base height |
| run12/13-era | added SIGNED approach check to `_fk_verify` (`cos_x = rot[:,0]·descent`): catches roll-180 backhand the sign-free y-check cannot see | jaws are y-symmetric; ONLY the x-axis check distinguishes fingers-down from fingers-up |
| **run13** | **FIRST LIVE GRASP: `GRASP CLOSED: contacts=7 fingers=[0.0173 0.0228] object z=0.646`** — fingers commanded to 0 but stopped at slab thickness = gripping. Then froze: `grasp_path=[open,reach,close]` has no lift; held forever; `dist_to_goal` frozen 17.56 | grasp execution PROVEN in the full env |
| run14 | added LIFT waypoint (4th, `lift_pose[3]=upper[3]`, torso up fingers closed) → **catastrophic**: torso jumped 0→0.386 in ONE step (the `action[:6]=target[:6]` override bypasses the ±0.05 scale), arm oscillated ±0.5 rad, slab batted out of pads: `contacts=0 fingers=[0.1006 -0.0811] object z=0.886` (airborne, flung) | torso must RAMP like the probe did (30 steps), never slam |
| run15 | torso ramped 0.004/step — grasp engaged ([0.0079 0.0198]) but the 700-step test loop expired mid-lift (qpos[3]=0.093) | loop budget: settle 200 + park ~430 + close ramp 160 + lift ramp ~100 ⇒ extended test loop 500→900 |
| run16 | loop extended; LIFT COMPLETED CLEANLY (torso 0.386, arm held, no oscillation) — but close missed: `contacts=0 fingers≈[0.000 0.002]` (fingers fully shut on air), object stayed on table z=0.645 | close still slams: ±0.05 action scale closes the 0.024-range prismatic fingers in ONE step → bats the light slab aside before pads arrive (the old probe Bug-16 shove, unsolved in solver) |
| run17 | added CLOSE ramp (`action[-2:] += clip(Δ,±0.0004)` over 160 ticks) — but slab LANDED TILTED this run: `sampler starved m_level:0` all 900 steps, robot wandered at point cloud (looked "stuck" to the user; run completed fine) | GUI physics settle is NON-DETERMINISTIC despite fixed seed: flat vs tilted landing changes candidate count 0↔2 |
| run18 | flat landing, close ramp armed — but the single candidate kept failing IK with `approach_err 76–102°` (backhand basin) at 12 different parking cells; never reached close | the good near-horizontal-axis candidates need fingers-down seeds (see Blocker 3) |

## Current progress (state of the code)

- **Grasp + close: PROVEN in full env** (run13: 7 contacts, fingers stopped at slab thickness).
- **Lift waypoint: PROVEN mechanically** (run16: torso to 0.386 clean with ramped
  close; ramp rate 0.008/step in solver now).
- Per-run diagnostics all in place: `sample_stats` starvation print, per-candidate
  failure points (`center, ee_target, pos_err, axis_err_deg, approach_err_deg, arm`),
  `GRASP CLOSED` evidence line, annulus-empty print.
- Test loop: `pick_nav_reach_env.py __main__` extended 500→900 steps
  (GUI timeout ≥ 1300 s).
- `landing_probe.py` (new): settles the object GUI-style then dumps landing
  orientation, AABB, and full sampler cascade — use it BEFORE any full run.

## Blockers (all diagnosed, none yet fixed)

### Blocker 1 — CARRY IS DEAD CODE (hard blocker for success)
`self.state = 2` is **never assigned anywhere in solver.py**. The state-2
"navigate to goal" branch is unreachable. Even a perfect lift leaves
`dist_to_goal` frozen at ~17.5 and `success` (dist<0.1) can never fire.
**Fix:** on lift-waypoint completion set `self.state = 2` (and reset path/nav),
so the robot carries the object to the goal marker.

### Blocker 2 — Close engages only by luck (candidate scarcity)
The slab's only viable top-down grasp is a pinch across the thin 3.5 cm
dimension at mid-height. Landing-orientation chaos (GUI non-determinism) moves
the level filter's survivors run-to-run:
```
flat landing probe:  m_plane=10740, level_tol=0.1 -> 0..2 pairs   (varies!)
level_tol=1.0: 32 candidates; near-horizontal (|axis.z|<=0.2): 15
level_tol sweep on plane pairs: <=0.1 -> 2, <=0.2 -> 15, <=0.35 -> 132
plane-pair |axis.z| percentiles: [0.01 0.71 0.90 0.96 1.0]  <- most ARE vertical
```
The dz filter (`|dz| <= 0.1·dist`) is a PROXY for closing-axis horizontality
that mostly works (vertical-axis pairs have large dz) but the interaction with
the tilted-cap plane filter leaves 0–2 survivors on the knife edge.
**Fix (planned):** replace/augment the dz proxy with a direct
`|axis·ẑ| <= ~0.2` filter (probe-verified 15 surviving pairs, all sane).
Optionally settle longer (300–400 steps) or `p.resetBasePositionAndOrientation`
to a flat pose for deterministic runs.

### Blocker 3 — IK lands in the BACKHAND basin for near-horizontal axes
Near-horizontal closing-axis candidates (the GOOD ones) fail verification with
`pos_err=0.000 axis_err=20–80° approach_err=94–102°` — the arm reaches the
position but the wrist ends fingers-UP (roll-180 about the grasp axis).
The existing 5 seeds include both shoulder_lift signs but apparently no
wrist_flex-flipped variant; `greedy_first` returns the first IK-accepted
candidate, and for these targets only the backhand basin converges.
**Fix (planned):** add wrist_flex-flipped seeds (mirror seeds 1–2 in joints
5/6) so both roll basins are sampled; keep the signed approach check (it is
doing its job — run18 rejected 12 backhands it should reject).

## Physics-execution findings (session 3)

- **Single-step slams are the recurring failure mode** (3 of them):
  1. torso: `action[:6]=target[:6]` override bypasses the ±0.05 clip → arm oscillation shook the grip out (run14);
  2. fingers: ±0.05 closes the 0.024-range prismatic joints in one step → slab batted aside (run16);
  3. (probe-era) same shove on fast close.
  All three need ramps, not jumps. Rule of thumb: any joint whose range is
  smaller than `action_scale` MUST be ramped by the waypoint machine.
- Headless (`gui=False`) is DEAD on this machine: eglRenderer plugin fails to
  load → RuntimeError. All probes/runs must be `gui=True`, foreground, bounded
  (timeout), per the laptop rule.
- GUI settle non-determinism persists despite `np.random.seed(seed)` — the
  z=5 drop's landing orientation is not reproducible across processes. Plan
  around it (deterministic pose or filter robustness), don't fight it.
- `tasklist` python.exe rows with <1 MB memory are MCP helpers, not stuck sims —
  do not misread them as hung runs.

## Session-3 fixes applied (by file)

**`solver.py`**
- `execute_grasp` grasp_path: 3 → 4 waypoints `[open, reach, close, lift]`;
  `lift_pose = close_pose.copy(); lift_pose[3] = upper[3]`.
- Waypoint machine: CLOSE branch now ramps fingers (±0.0004/step, 160 ticks);
  LIFT branch ramps torso (±0.008/step) and waits for
  `|Δqpos[3]|<0.02 and |qvel[3]|<0.05`.
- (Earlier this session) airborne deferral, blacklist guard `calls>0`,
  cached-grasp clear, table-top AABB skip `lower[2]>=0.35`,
  `GRASP CLOSED` evidence print.

**`pick_nav_reach_env.py`**
- `__main__` test loop 500 → 900 steps (settled: 200 settle + ~430 park +
  160 close + 50 lift + margin).

**`custom_libraries/antipodal.py`**
- `_fk_verify` returns `(pos_err, axis_err, approach_err)`; SIGNED
  `cos_x = rot[:,0]·descent` check rejects fingers-up roll-180.

**`landing_probe.py`** (new) — settle + landing orientation + sampler cascade dump.

## Session-3 future actions (priority order)
1. **Wire state 2**: lift complete → `self.state = 2` + nav reset → carry to
   goal. THE blocker for `success=True`; everything else is reliability.
2. **Fingers-down wrist seeds** in `select_optimal_grasp` (Blocker 3).
3. **Closing-axis filter** `|axis·ẑ| <= 0.2` replacing/aiding the dz proxy
   (Blocker 2); re-run `landing_probe.py` to confirm stable candidate counts
   across 3 landings.
4. Then A/B `score_all` vs `greedy_first` (now actually meaningful — both
   paths get real candidates).
5. Then the carry phase itself: does the slab stay in the pads through driving
   17 m through the maze? (Expect new physics findings — corners, speed.)
6. Probe cleanup still pending: `quick_reach_probe.py`, `sampler_debug.py`,
   `landing_probe.py`, `debug_ik_once.py`, `finger_check.py`,
   `gripper_frame_check.py`, `joint_dump.py` — fold the keeper checks into
   `test_antipodal_smoke.py`, delete the rest (ask first).
7. Journal/code `ee_forward_offset` doc inconsistency (0.03 vs 0.04) still open.
