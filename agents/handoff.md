# Handoff: Pick-Navigation-Reach debugging

Read `agents/fixes.md` first. It explains every change made so far and why.
This file covers where things stand, how to test, and what to do next.

## Session 3 update (rubric scorer, lift height, planning budgets, goal reach)

Date: 2026-11-18. Read this section first; the Session 2 section and the
rest of the file are kept for reference, and the Session 2 results table is
superseded by the one below.

**Files moved.** `handoff.md` and `fixes.md` now live in `agents/` (commit
1bb5896).

**Rubric scorer** (commit 3514757; tweaks a7173ac and 5d2abd8):
- `python tools/score.py PROJECT_DIR OBJ SEED [MAX_STEPS]` runs one headless
  trial and prints one `SCORE:{json}` line. "Step" means one `env.step` call
  (an assumption; the teaching team has not confirmed it). Fields:
  - `lift_step`: step at which the solver entered state 3 (grasp and lift
    done);
  - `lift10_step`: first step the object was >= 10 cm above its resting
    height;
  - `lift_cm`: height above the resting position when state 3 was entered;
  - `hold_min_cm`: lowest height during the 50 steps after that;
  - `hold_50`: bilateral finger contact on every one of those 50 steps, and
    the object still clear of the table (> 2 cm) at the end;
  - `grasp_pos_err` (metres, also reported in mm) and `grasp_ang_err`
    (radians): planned grasp centre and gripper orientation, fixed in the
    object frame at plan time, against the actual ones after the 50-step
    hold;
  - `success_step`: first step with the object within 10 cm of the goal;
  - `drop`: null, or `{step, state}` for the first run of >= 30 steps
    without bilateral finger contact after the lift;
  - `stopped`: why the trial ended (success, state4, dropped, stall,
    max_steps). It stops at the first success, 50 steps after a drop, on
    state 4, on a stall (no joint moved by > 0.1 mm for 400 steps after the
    lift) or at MAX_STEPS (default 15,000);
  - `points`: rubric points, maximum 40 per trial.
- Points per trial: lift within 3,000 steps 6; lift >= 10 cm 6 (judged on the
  lowest height over the 50-step hold, measured from the object's resting
  height); hold 50 steps 6; grasp pose 6 (within 10 cm); reach within 10 cm
  6; within 15,000 steps 6; no drop 4.
- `python tools/batch.py OUT_DIR [--objects ...] [--seeds ...] [-j N]
  [--repeat N]` copies the working tree to `OUT_DIR/snap`, runs the trials
  in parallel and prints a markdown table plus totals. `--report` reprints
  the table for an existing `OUT_DIR`.

**Machine limits (measured).**
- Each simulation process uses about 900 MB of RAM. On the 5.9 GB, 4-core
  (8-thread) dev machine, run at most 4 simulations at once in total; a 5th
  and 6th pushed it into swap and background shells were killed.
- One collision check (`GraspPlanner._valid`) costs about 30 ms of CPU.

**Planning budgets were the main source of run-to-run noise** (two commits):
- a7173ac: every planning deadline (grasp planner and goal reach) now uses
  process CPU time (`planning_clock = time.process_time` in `grasp.py`)
  instead of wall time. Evidence: six parallel banana trials all returned
  "No plan found" with reason `time_budget` after about 20 candidates, and a
  first baseline batch run 6-wide scored 2/10 with 3 trials failing at
  planning.
- ec604ba: `rrt_seconds` 1 -> 8 and `planning_seconds` 30 -> 60 (per grasp
  depth). With a generous budget, 28 of 29 banana transit plans were plain
  straight-line edges that took 1-2.5 CPU-s just to collision-check (50-70
  samples), so a 1 s budget rejected them on a busy machine. The planner then
  failed or fell through to a late candidate (banana picked candidate 12 or
  23 instead of 1, and was then dropped). After the change no candidate in
  the reference batch was rejected for transit.
- Planning time costs no simulation steps.

**Lift height** (7b93fc6). `lift_distance` 0.08 -> 0.15, and the lift is
accepted when the object has risen `min_lift_fraction` (0.8) of it, 12 cm
(it was 0.5 x 8 cm = 4 cm). Lifts now measure 14.8-15.5 cm and sag <= 3 mm
over the 50-step hold, so the "hold target + integral in state 3" idea from
the old next-steps list was not needed and was not done.

**Goal reach** (commits 6e5ca76, b8caf19, 380d21a, in `Solver._solve_reach`).
Done by a subagent with a harness that froze the episode at the start of the
reach and called `_solve_reach` repeatedly.
- Soup can seed 7, root cause: not IK. The can, held top-down, creeps up
  during the carry until it is 0.3 mm from the palm (`gripper_link`), inside
  the planner's 2 mm collision margin. Fingers were exempt from the
  held-object check but the palm was not, and since the object moves rigidly
  with the gripper in the planner, every configuration including the start
  was invalid (0 of 6 plan calls succeeded). Fix: the held object is not
  checked against the gripper links; it is still checked against walls and
  against the rest of the robot.
- Soup can seed 42 was a budget problem. One RRT used 6 s of the 20 s budget,
  so candidates with a clear straight-line move were never tried (the old
  search failed 2 of 5 calls). Fix: IK for all candidates first, return the
  first with a collision-free straight move, then RRT nearest-first; budget
  20 -> 60 CPU-s.
- Standoff distance, base heading and goal height were not the cause (the
  base stops 0.62 m from the goal). The old ideas "several standoffs and
  headings", "stop 0.7 m short" and the backoff wall check/timeout were not
  needed and were not done.
- Offline: seed 7 plans 0/6 -> 8/8; seed 42 3/5 -> 8/8, 2.5-7.2 CPU-s each.

**Results.** Reference batch: objects 5 6 7 14 2 x seeds 42 and 7 (10
trials, 4 in parallel), one row per code state. Columns from lift<=3000 on
are the number of trials (of 10) that earned that item.

| Code state | Delivered | Mean points /40 | lift<=3000 | lift>=10cm | hold 50 | grasp pose | reach | no drop |
|---|---|---|---|---|---|---|---|---|
| Baseline, CPU clock only (a7173ac) | 5/10 | 25.4 | 9 | 0 | 9 | 9 | 5 | 8 |
| + lift 15 cm only (not a commit state; 1 s RRT budget) | 4/10 | 29.2 | 9 | 9 | 9 | 9 | 4 | 7 |
| + planning budgets + lift 15 cm (7b93fc6) | 6/10 | 34.0 | 10 | 10 | 10 | 10 | 6 | 7 |
| + goal reach fix (380d21a) | 7/10 | 35.2 | 10 | 10 | 10 | 10 | 7 | 7 |
| + squeeze 2 mm past measured contact, ramped (REJECTED, not committed) | 2/10 | 12.2 | 3 | 4 | 4 | 4 | 2 | 2 |

Per trial at the current committed state (380d21a batch):

| Object | Seed 42 | Seed 7 |
|---|---|---|
| Banana (5) | delivered, step 10,727 | delivered, 10,291 |
| Strawberry (6) | delivered, 11,313 | lost during the final arm move (state 68, step 11,276) |
| Apple (7) | dropped in state 67 (step 3,555) | dropped in state 67 (step 5,245) |
| Foam brick (14) | delivered, 10,747 | delivered, 10,722 |
| Soup can (2) | delivered, 11,842 | delivered, 10,530 |

The lift normally completes at step 1,035-1,250, but at 1,581, 2,425 and
2,494 in three trials (see open issues).

**Tried and rejected: squeeze relative to the measured contact width.** The
hold command was the finger positions measured when the close phase ends
minus 2 mm per finger, ramped at 0.2 mm per tick during the lift. Result:
2/10 delivered.
- Banana (both seeds), apple seed 7 and foam brick seed 7 lost bilateral
  contact during the lift.
- Soup can (both seeds): "arm reached lift pose but the object did not rise".
- Apple seed 42 and foam brick seed 42 slid 26-29 mm in the fingers and were
  dropped.
- Why: the finger position gain is 0.03 (arm joints 0.3), so a 2 mm command
  offset gives far too little force; the old command (7-9 mm inside the
  collision hull) gives 45-60 N. An intermediate value, and the ramp alone,
  are untested. The patch is kept outside the repo and was not committed.

**Mid-maze drops: what is known now (work in progress).**
- They are intermittent. In batches at the committed state, 3 of 10 trials
  lose the object (apple both seeds in one batch; apple seed 42, foam brick
  seed 7 and soup can seed 7 in the previous one).
- Twelve single traced runs of the same code (apple seed 42 x4, apple seed 7
  x4, foam brick seed 7 x2, strawberry seed 7, soup can seed 7): nine
  delivered, two apple seed 7 runs dropped mid-maze (steps 3,560 and 5,196)
  and one apple seed 42 run ended in state 4 before the lift ("Object moved;
  stop and obtain a fresh plan", not investigated).
- In the runs that delivered, drift in the gripper stayed at 1-6 mm (soup
  can 28 mm, into the palm, still delivered) and grip force was steady at
  45-51 N per finger.
- The two caught drops (`tools/trace_carry.py`) show creep, not a jolt. The
  base moves smoothly at 3.33 mm per step and no command changes. Drift
  grows from about 3 mm to 9-14 mm over two to three thousand steps of
  driving, speeds up over the last ~10 steps, and the grip force then falls
  from 50-60 N to zero in about 3 steps. The object does not creep while the
  base stands still (re-tuck).
- Likely mechanism (measured geometry, not yet confirmed by a fix): in the
  carry pose the gripper's closing axis points straight down (world z
  component -0.96), so one finger is above the other and the object can slide
  sideways off the pads. The pads are 24 mm tall, so 12 mm of sideways drift
  is the edge; the losses come at 10-14 mm.
- Next: a carry pose with the fingers pointing up and the palm underneath,
  inside the same base footprint (0.289 m), so creep runs into the palm. A
  random joint-space search (180,000 samples) found none; an IK search over
  gripper positions above the base was running at the time of writing.
- `tools/trace_carry.py PROJECT_DIR OBJ SEED MAX_STEPS OUT.csv` logs every
  carry step (base displacement measured and commanded, arm joint velocity,
  finger forces, navigator status, drift) and prints EVENT lines on
  grip-force jumps.

## Session 2 update (grasp hold during transport)

Superseded by the Session 3 section above (its results table in particular);
kept for reference. The rest of the file is from session 1.

**Why objects were dropped** (measured with `tools/trace_hold.py`):
1. **Contacts at the fingertip edge.** The planner aimed the contact 6.5 mm
   from the pad tip (`tcp_offset` 0.025 on a 63 mm pad). During the lift the
   object slid onto the tip edge and hung there. Fetch's own grasp frame
   (MoveIt / simple_grasping) is the pad centre.
2. **Friction creep.** Bullet solves friction at velocity level, so a held
   object creeps along the pads even with ~10× the friction it needs. Grip
   force is not the problem (20–50 N on objects of 0.05–0.32 kg).
   - PyBullet's `frictionAnchor` on the finger links removes the creep
     completely (0 mm in 1,500 ticks), but it changes the robot's contact
     model. **The team decided it must stay off; it is not in the code.**
3. **Off-centre grasps pivot.** Spinning friction is 0, so a grasp line that
   misses the centre of mass by 3–4 cm lets gravity rotate the object out.
4. **Collisions.** The final reach interpolated in joint space and swung the
   object into the 1 m-wide corridor walls (maze walls are 2 m tall). One
   banana still touched the table after the lift and was dragged off.
5. **Ejection** at high squeeze: twice, contact went from 40–50 N to 0 in 4
   ticks with no outside contact and the object was flung metres. Probably
   the harder squeeze (below) storing penetration energy. Not yet fixed.

**Changes (uncommitted on `fix/pick-nav-reach` at the time of writing):**
- `grasp.py`:
  - `tcp_offsets = (0.010, 0.025)`: plan with the contact 21 mm from the tip
    first, and fall back to the old depth only if nothing plans.
  - `com_weight = 4.0`: candidates are ranked down by the distance from the
    centre of mass (`obs["object_pos"]`) to the grasp line.
  - `squeeze` 0.004 → 0.012.
- `navigate_to_grasp.py`: passes the centre of mass to the sampler.
- `solver.py`:
  - state 68 plans the reach with RRT-Connect, with the object attached and
    the maze walls as obstacles, aims at the goal or points 5 cm from it, and
    retries 3 times; before, it did a blind joint-space move;
  - states 5/67/68 fail into state 4 after 30 ticks without bilateral finger
    contact, instead of driving on empty (`grasp_error` says where).
- `tools/trace_hold.py` (new): per-tick finger contact, grip force, drift of
  the object in the gripper frame, contacts with other bodies, and a history
  dump before the first contact loss. Ends with `DROP:` and `RESULT:` lines.

**Results** (clean batch of this code: 5 pickable objects × seeds 42 and 7,
friction anchor off): **5 / 10 delivered**, up from 2 / 10 at the start of the
session. All 10 picks succeeded.

| Object | Seed 42 | Seed 7 |
|---|---|---|
| Banana (5) | delivered, 10,273 steps | delivered, 10,992 |
| Strawberry (6) | delivered, 10,250 | delivered, 10,443 |
| Foam brick (14) | delivered, 10,416 | dropped mid-maze (t = 7,501) |
| Apple (7) | dropped mid-maze (t = 5,523) | dropped mid-maze (t = 7,284) |
| Soup can (2) | held, reach never planned | held, reach never planned |

- The three drops were in state 67 with no contact with another body
  beforehand; grip force was 37–51 N just before two of them, and the object
  had drifted 2.5–4 cm in the gripper.
- Soup can: IK fails for every orientation from the fixed 7-cell backoff; the
  robot holds the object 1.0 m from the goal until the step cap.

**Settling after `env.reset()`** (object dropped from z = 5 m; the seed only
changes the maze, so this is the same on every seed):
- **mug (10)** bounces off the table and lands on the floor at about
  (−2.007, −2.072, 0.045);
- **large marker (12)** settles at z = 0.595, below the tabletop (0.626): it
  sinks partly into the table, which likely explains its "no plan" results;
- **bleach cleanser (8)** is still sliding (~2 cm/s) at step 200, when the
  solver starts, and ends 0.52 m from the table centre.

## Where things stand (session 1)

- **Branch:** `fix/pick-nav-reach`, on top of `last` (`ac3da6a`). Already on
  `origin`; open a PR into `last` once the team is happy.
- **Files changed:**
  - `solver.py`
  - `custom_libraries/grasp.py`
  - `custom_libraries/navigate_to_grasp.py`
  - `fixes.md`, `handoff.md` (now in `agents/`), and the scripts in `tools/` (new)
- **Results.** From headless runs on seeds 42 and 7. Runs are **not deterministic**, so judge by rates, not single runs.

  | Stage | Result |
  |---|---|
  | Valid grasp start state | 30 / 30 runs (15 objects × 2 seeds) |
  | Pick (lifted, held 100 steps) | 9 / 30 in the full sweep. The five small objects picked 10 / 10 in the later end-to-end runs. |
  | Full task (object within 10 cm of goal) | 2 / 10: banana and strawberry, both seed 42 |

  Per-object picture from the last sweep:
  - **Pick reliably:** banana (5), strawberry (6), apple (7), foam brick (14), tomato soup can (2).
  - **Plan found, but execution fails** (object pushed or slips): mustard (3), hammer (13), potted meat (4).
  - **No plan** (`ik_fail` / `transit_fail`): sugar box (1), large marker (12).
  - **Not re-tested after the fixes:**
    - master chef can (0): the wide-object opening fallback was added after the sweep. It planned again, but no full run was done.
    - bowl (9) and plate (11): rim-grasp fix.
  - **Never leaves state 0, no error:** bleach cleanser (8). Not investigated.
  - **Mug (10):** the environment drops it on the floor after `env.reset()`, so it is unreachable and the solver goes to state 4.

## How to run and test

Setup:
```bash
pip install pybullet numpy scipy trimesh matplotlib
```

For rubric scoring and batches use `tools/score.py` and `tools/batch.py`
(Session 3 section): `python tools/batch.py OUT_DIR -j 4`. The older tools
below still work.

Single headless episode. This prints a `RESULT:{json}` line with:
- the states visited and the final state;
- `grasp_error`, `lift_cm` and `planner_stats` (per-reason reject counts);
- `held_100`, `min_goal_dist` and `stopped`.
```bash
python tools/run_one.py . --object 5 --seed 42 --max-steps 16000 --maze-copy /tmp/m_5_42.urdf
```

Notes on the runner:
- It stops early when the object is within 10 cm of the goal, or when the solver enters state 4.
- A full run with transport takes about 9,000–10,000 steps, or 1–2 minutes (measured on a 2-core machine in session 1).
- Use `--max-steps 3000` when you only care about the pick.
- Give each parallel run its own `--maze-copy` path. The env writes the maze URDF there, and parallel runs clobber each other otherwise.

Watching the carry phase:
```bash
python tools/trace_nav.py . 5 42 14000
```
It prints the base, the object, the distance to the goal, the waypoint and the reach phase every 400 ticks in states 5, 67 and 68. Edit the condition at the top of its loop to trace other states or intervals.

Sweep and classify. Logs must be named `o{idx}_s{seed}.log`:
```bash
mkdir -p logs
for s in 42 7; do for i in $(seq 0 14); do echo "$i $s"; done; done | \
  xargs -P 2 -n 2 sh -c 'timeout 600 python tools/run_one.py . --object $0 --seed $1 --max-steps 3000 --maze-copy /tmp/m_$0_$1.urdf > logs/o$0_s$1.log 2>&1'
python tools/summarize_sweep.py logs
```
It labels each run PICK_OK, START_INVALID, SEARCH_EXHAUSTED, EXEC_FAIL, CRASH, TIMEOUT or OTHER.

## State machine (solver.py)

| State | Meaning | Exit |
|---|---|---|
| 999 | Wait 200 steps for the object to land | → 0 |
| 0 | `NavigateToGrasp` decides NAVIGATE / PREPARE / GRASP | → 1 (GRASP), → 4 (no usable grasp) |
| 1 | Hold the latched pose, wait to settle, `GraspPlanner.plan()` | → 2 or 4 |
| 2 | `GraspFollower`: approach, close, lift | → 3 or 4 |
| 3 | Hold the lifted object for `hold_before_transport` ticks (60) | → 5 |
| 5 | Retreat `retreat_distance` (1.0 m) straight back, arm frozen | → 67 |
| 67 | Re-tuck into `carry_joint_targets` slowly, rebuild the map with the table as an obstacle, then A* + `CurvedNavigator` with base speed capped at `carry_speed` | → 68 when `curved_nav.status == "reached"` |
| 68 | `reach_goal()`: back off `reach_backoff_cells` (7) along the path, solve IK so the object (not the gripper) lands on the goal, move the arm slowly | holds |
| 4 | Failed; holds the last pose | terminal |

Tunables live in `Solver.__init__`:
- `hold_before_transport`
- `retreat_distance`
- `carry_speed` (4 mm/tick)
- `reach_backoff_cells`
- `travel_joint_targets` (driving to the table)
- `carry_joint_targets` (carrying through the maze)

The tuck rate while carrying is `_tuck_rate`, set to 0.003 rad/tick when entering 67.

## Things that are easy to get wrong

- **Action format.** The action is 15 absolute joint targets.
  - Base world x = `qpos[0] − 2.0`.
  - Index 2 is base yaw.
  - Index 3 is the torso.
  - Indices 6–12 are the arm.
  - Indices 13 and 14 are the fingers.
- **`env.step` and `time.sleep`.** In `pick_nav_reach_env_multi.py` it only sleeps (1/240 s per substep) when `gui=True`; the original env always sleeps. The tools still monkeypatch `time.sleep`; do the same in any new script that might use the original env.
- **No integral action in the controller.** The env's position controller has gains of 0.3 and no integral term. Commanding the *measured* pose as a hold target makes the arm sag; this was bug 2. Always latch targets.
- **Navigation runs at a fixed heading.** The wall inflation uses the robot's *current* projected footprint. Any arm pose wider than about 0.3 m from the base centre seals the maze bottleneck. Always measure a new pose with `custom_libraries/wall_projection.robot_xy_polygons(env)`.
- **The held object isn't in the footprint.** The footprint ignores the object, so a long object can still brush the walls.
- **The parking map leaves the tabletop out on purpose.** `create_map(env, is_2d=True)` skips boxes whose lower z is ≥ 0.35, so the base can tuck under the table. For transport, `create_map(env, False)` is called to include them.
- **`a_star` returns `[]` on failure, not `None`.**
- **Line endings.** `grasp.py` uses CRLF; the other files use LF. Preserve them when editing with scripts.

## Continuing in Claude Code

**Setup** (Python 3.10+, no GPU needed for the tools):

```
git clone https://github.com/testing1234567891011121314/CS4278-Project
cd CS4278-Project && git checkout fix/pick-nav-reach
pip install pybullet numpy scipy trimesh matplotlib
python tools/trace_hold.py . 5 42 16000 400     # banana, seed 42: ~2-5 min
```

**Rules agreed with the team**
- Control and planning only. No changes to the environment, the object state
  or any contact/dynamics parameter: no `frictionAnchor`, no `changeDynamics`,
  no constraints, no re-settling the object.
- Do not edit `pick_nav_reach_env*.py` behaviour.
- `custom_libraries/grasp.py` has CRLF line endings; keep them.

**Working method**
- Run batches with `python tools/batch.py OUT_DIR -j 4`. It runs from a
  frozen copy of the repo (`OUT_DIR/snap`), so editing while a batch runs does
  not mix code versions. Never run more than 4 simulations at once on the
  5.9 GB dev machine (see the Session 3 section).
- The reference batch is objects 5 6 7 14 2 × seeds 42 and 7 (10 runs).
  Current score (380d21a): 7 / 10 delivered, 35.2 / 40 mean rubric points.
- `trace_hold.py` ends each log with `DROP:<json|null> RESULT:<json>`; a run
  is delivered when `goal_dist < 0.10` (it stops at the first success, so
  delivered runs read ~0.099).
- The claims marked "review" below come from reading the code, not from
  runs. Confirm each with a batch before keeping the change.

**Rubric** (readme.md): pick is 64 %, navigation + reach 36 %. Per trial:
lift within 3,000 sim steps, lift ≥ 10 cm above the table, hold ≥ 50 steps,
final pose close to the target grasp; reach within 10 cm, within 15,000 sim
steps, no drop. Graded on 5 objects from the provided set and 5 unseen seeds.
- **Open question for the teaching team:** "sim steps" is most likely
  `env.step` calls (the only counter in the env; our pick takes ~1,100 calls
  and a full run ~10,500). If it means physics substeps (5 per call) neither
  budget is reachable.

## Next steps, in priority order

Updated in session 3 (steps 1, 2 and the soup can part of 4 are done).

1. **Rubric scorer.** Done: `tools/score.py` and `tools/batch.py`.
2. **Lift height.** Done (7b93fc6): lifts measure 14.8-15.5 cm.
3. **Stop the mid-maze drops** (3 of 10 trials at the committed state;
   open, work in progress). The facts are in the Session 3 section: the
   object creeps sideways off the 24 mm pads while the base drives. First
   try a fingers-up carry pose (see there). Other untested ideas:
   - ramp the finger command alone (it jumps from +0.4 mm to the full
     squeeze at the start of the lift), without the contact-relative offset;
   - an intermediate squeeze between the old command and 2 mm past contact
     (see "Tried and rejected" above);
   - `GraspFollower` integral: clip per joint (torso 0.01 m, arm 0.03 rad),
     integrate only near the target, reset at each phase, freeze during
     close; leave finger errors out of the common speed scale (review);
   - smooth the transport: slew-limit the base command, slow down before
     stops and in corners, move all tuck joints on one time scale with a
     ramp, blend the reach waypoints (review);
   - tilt the approach axis upward right after the lift, so creep runs
     toward the palm.
4. **Goal reach.** Done for the soup can (380d21a); the apple reach works in
   traced runs. Strawberry seed 7 was lost during the reach arm move once
   (item 6).
5. **Slow grasp execution.** State 2 normally takes about 750 steps but took
   1,300-2,200 in three trials (the lift completed at step 2,425 and 2,494
   against the 3,000 limit). Probably long, unsmoothed RRT transit paths now
   that the larger budget lets them through. Add path shortcutting and/or
   speed up the follower.
6. **Strawberry lost during the reach arm move** (seed 7, state 68, step
   11,276, once). Not investigated.
7. **Rubric interpretations.** `score.py`'s grasp-pose tolerance (10 cm) and
   "lift height measured from the resting height" are our readings of the
   rubric.
8. **Ask the teaching team.** The rubric text gives carry stability as
   "4 pts" and also as "2 pts each (max 8)"; ask this together with the
   "sim steps" question above.
9. **Widen the set of pickable objects.**
   - Rank by the *horizontal* distance from the centre of mass to the grasp
     line, `|((com − p) × z) · d|`; the current 3-D distance penalises top
     grasps on tall boxes by ~0.27 against a quality spread of ~0.1 (review).
   - Check the pregrasp pose before RRT and count it separately (it is
     reported as `transit_fail` today); add a palm-clearance test to the
     sampler; give side approaches a chance for tall boxes.
   - Choose the grasp depth per candidate: as deep as possible with the
     fingertips ≥ 4 mm above the table.
   - Plan with friction 0.25: fingers and objects are both 0.5 and Bullet
     multiplies them (measured with `getDynamicsInfo`); the planner assumes 0.5.
   - After a failed plan, retry from another parking spot
     (`mark_goal_failed` exists but is never called); add a timeout to state 1.
   - `contact_penetration` 6 mm → ~3 mm, pad-face contacts only (review).
10. **Awkward starts.**
   - Bleach (8): still sliding at step 200. In state 999 wait until the
     object has stopped moving (with a cap) instead of a fixed 200 steps.
   - Large marker (12): sunk in the tabletop (z = 0.595 < 0.626). Fail fast.
   - Master chef can (0): 102 mm across, gripper opens ~95 mm. Fail fast.
   - Untested since session 1: bowl (9), plate (11).
11. **Floor pick for the mug (object 10)**, which `env.reset()` bounces off
   the table onto the floor at about (−2.007, −2.072, 0.045). Today
   `NavigateToGrasp.update()` raises "Grasp height is outside the configured
   coarse height range" (`min_z` = 0.20 m in the base frame) and the solver
   stops in state 4. Steps:
   - **check feasibility first:** with the torso at 0, can the planner's IK
     model put the gripper at z ≈ 0.05–0.10 m, and from what base distance?
   - lower `min_z` or give floor objects their own height range and parking
     distance; park on the floor map;
   - add a low travel / pre-grasp arm pose, measured with
     `wall_projection.robot_xy_polygons(env)`;
   - treat the plane (body 0) as the support surface and keep the fingertips
     clear of it;
   - if it is not feasible, fail fast with a clear message.
12. **Robustness.**
   - Navigator deadlock: on `blocked_or_off_path` the navigator returns zeros
     forever; add a replan or timeout (review).
   - `_check_holding` does not cover state 3; a loss during the 60-tick hold
     goes unnoticed (review).
   - Declare the `hasattr`/`getattr` solver attributes in `__init__`.
13. **Housekeeping**, if the team agrees: the committed `pnr/` virtualenv,
   the stale `test.patch`, the `navigation_p4000_*` jsonl/mp4 files at the
   root, `custom_libraries/old/`, noisy prints in `create_map` and state 67.

## Session 1 next-steps list (superseded by the list above)



1. **Re-measure** the session 2 code in a clean batch (anchor off), and run
   batches from a copy of the repo so edits don't mix code versions mid-batch.
2. **Finish the grasp hold without the friction anchor.** Creep and ejection
   during transport are still the main losses. Ideas:
   - tune `squeeze` between 0.004 and 0.012 (harder squeeze slows creep but
     seems to cause the ejections);
   - make creep run toward the palm: keep the gripper's approach axis tilted
     upward whenever the object is held (the carry pose already does this;
     the re-tuck from the top-down lift pose passes through tip-down poses);
   - shorten the time between lift and carry pose, since creep is per tick;
   - re-grasp when the object is lost onto the table (it is detected now).
3. **Attempt a floor pick for the mug (object 10)**, which `env.reset()`
   bounces off the table onto the floor at about (−2.007, −2.072, 0.045).
   Today `NavigateToGrasp.update()` raises "Grasp height is outside the
   configured coarse height range" (its `min_z` is 0.20 m in the base frame)
   and the solver stops in state 4. Steps:
   - **check feasibility first:** with the torso at 0, can the planner's IK
     model put the gripper at z ≈ 0.05–0.10 m at all, and from what base
     distance? Fetch only just reaches the floor;
   - lower `min_z`, or give floor objects their own height range and parking
     distance, and park on the floor map (no table near the mug);
   - add a low travel / pre-grasp arm pose for floor grasps, measured with
     `wall_projection.robot_xy_polygons(env)` like the other poses;
   - the floor is the support surface: check the plane (body 0) is treated
     like the table in `support_contacts`, and keep the fingertips clear of
     it (the deep `tcp_offset` puts the tips ~20 mm below the grasp centre);
   - likely grasps are the rim or handle from above; lift straight up before
     the retreat;
   - if it is not feasible, fail fast with a clear message instead.
4. **Verify the untested fixes** with a full sweep: master chef can, bowl and plate.
5. **Fix the planner for tall or long objects** (sugar box, large marker, potted meat). Their failures are `ik_fail` plus `transit_fail` where the forearm or wrist hits the table. Things to look at:
   - the large marker first: it settles partly sunk into the table (z = 0.595 < 0.626), so its start state may be the real problem;
   - candidate ranking;
   - side (horizontal) approaches for boxes;
   - the base standoff distance.
6. **Investigate the bleach cleanser** sitting in state 0. It is still sliding when state 999 ends after 200 steps; wait until the object has stopped moving instead of a fixed count. Also add a print of `decision` / `decision_state` in solver state 0.
7. **Episode length.** Check whether the grader has a step limit. Transport takes about 10k steps at `carry_speed` = 4 mm/tick, so raise the speed carefully once the grasp is firmer.
8. **Housekeeping**, if the team agrees:
   - the committed `pnr/` virtualenv;
   - the deleted `requirements.txt` and tests on `last`;
   - the stray `test.patch`.