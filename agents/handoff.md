# Handoff: Pick-Navigation-Reach debugging

Read `agents/fixes.md` first. It explains every change made so far and why.
This file covers where things stand, how to test, and what to do next.

## Session 3, part 4: carrying with base yaw

Date: 2026-11-19. Newest section. Branch `worktree-agent-a0d15e881c9cb4b97`
(off e57127d on `fix/pick-nav-reach`), local only. Fixes 31-34 in fixes.md
explain the changes; this section has the numbers.

**What changed.** `Solver.carry_mode = "yaw"` (default). After the retreat
the arm stays as it is after the lift (forward, gripper down, object hanging
0.75-0.77 m ahead of the base), the torso lifts the object to 0.9 m, the base
turns on the spot to face the route, and the object's centre is driven along
a smooth centre-line route with the base trailing it like a trailer; the
route ends with the object on the goal, so there is no arm reach. The
wrist keeps the closing axis along the object's motion. Heavy objects are
carried slower (weighed from the contact forces); a slip governor (slow
down once the object slides toward the fingertips) exists but is off.
`carry_mode = "fixed"` is the old
carry; the yaw carry falls back to it by itself when the gripper does not
point down or no route passes the swept check. Code: `solver.py`
(`_yaw_carry_plan`, `_yaw_carry_step`, `_adapt_to_weight`,
`_slip_toward_tips`), `custom_libraries/yaw_carry.py` (geometry, no
PyBullet), `tests/test_yaw_carry.py`.

**Before / after.** Objects 5 6 7 14 2, seeds 42 and 7. "Before" is
e57127d run from an extracted copy of that commit (30 trials); "fixed" is
this branch with `SOLVER_SET="carry_mode=fixed"`, which runs the same code
path as e57127d.

| carry | trials | delivered | delivery steps (median) |
|---|---|---|---|
| before: fixed yaw, e57127d | 30 | 18 (60 %) | 1,674-1,924 (1,806) |
| base yaw, slip governor off (yaw_v1, yaw_v2, yaw_v2b, final4_std) | 90 | 75 (83 %) | 1,171-1,388 (about 1,250) |
| base yaw, slip governor on (final_std, final2_std, final3_std) | 90 | 71 (79 %) | 1,224-2,165; 8 over 1,500 |
| seeds 1 2 3, fixed (`carry_mode=fixed`) | 15 | 12 | 1,670-1,905 (1,744) |
| seeds 1 2 3, base yaw | 15 | 12 | 1,196-1,374 (1,262) |
| independent re-run of the final code, seeds 42 7 1 2 3, twice each | 50 | 37 (74 %) | 1,198-1,380 |

In the independent re-run: soup can 10/10, strawberry 9/10, foam brick 9/10,
apple 6/10, banana 3/10. Identical code varies between 21 and 27 of 30 from
batch to batch, so differences under about 5 in 30 are noise.

Steps by phase, medians over delivered trials:

| phase | fixed yaw | base yaw |
|---|---|---|
| pick (to end of lift) | about 312 | about 340 |
| hold | 60 | 60 |
| retreat | 46 | 46 |
| setup (wrist roll 40, turn on the spot 40, wrist roll 40) | - | 119 |
| arm swing + maze / drive | about 1,094 | about 715 |
| reach | about 278 | 0 (the object is driven onto the goal) |

**Where drops happen now.** Mostly in the drive. The apple creeps toward the fingertips
(downward, gripper pointing down) over hundreds of steps and goes suddenly
at 20-23 mm. The banana goes suddenly after corners or during the wrist
rolls and the turn on the spot. A few banana and strawberry losses are in
the setup; a few are in the retreat, which both carries share.

**Heavy objects** (1 sugar box, 13 hammer, 4 potted meat can, 8 bleach
cleanser; seeds 42 and 7). Fixed carry: 1/8 (potted meat can, 1,959 steps).
Yaw carry with the weight scaling: 2/8 (bleach 2,263, hammer 1,252); with
uniformly gentle settings on 1, 13, 8: 2/6 (both bleach, 2,216 and 2,414).
The sugar box was lost in all 4 yaw trials (early in the drive) and both
fixed ones. Potted meat can: one grasp was a side grasp (fell back to the
fixed carry and was lost), one plan failed. Bleach cleanser now picks
(lift at 567-610) but one trial failed the grasp ("Object moved").

**What did not work** (details in fixes.md, part 4): base on the path with
yaw along the tangent (the arm swings out and hits the walls at every
corner); turning on the spot at corners; retracting the arm (not needed);
slower everything for light objects (no clear gain, 200-500 more steps);
no wrist roll around the turn on the spot (inconclusive).

**Not verified.**
- Only maze seeds 42, 7, 1, 2, 3; routes were found and passed the swept
  check on all of them, with the post-lift pose of objects 2, 5, 6, 7, 14.
- The swept check uses the convex hull of each collision link projected to
  the floor (walls are 2 m tall, so height is ignored) and a 4 cm margin;
  tracking error is not modelled. No wall contact was seen in the traces,
  but contacts were not logged systematically.
- Side grasps: the fallback path was tested only by forcing it
  (`yaw_carry_margin=0.6`: soup can delivered by the fixed carry at 1,786).
- The weight estimate reads contact forces from the simulator; whether that
  is allowed is the same open question as reading the mass.

**Next.**
1. Apples and bananas remain the weak objects (creep toward the fingertips;
   angular kicks on long objects). A grasp choice that holds round objects
   lower on the pads, or carrying them gripper-up without the swing, would
   address the cause.
2. About 120 steps go to the turn on the spot and its two 90-degree wrist
   rolls; a curved start (combined retreat and turn) would remove most of
   it, and the 60-step hold could overlap the retreat.
3. The sugar box needs more than a slower carry.

## Session 3, part 3: toward 1,500 steps

Date: 2026-11-19. Placed above part 2 because this file lists the newest
work first; part 4 above supersedes its carry description. Read it after
part 4, then part 2.

**New target.** The user now wants the whole episode (pick, carry and reach)
within 1,500 `env.step` calls for full credit; it was 3,000. The code at
84202b1 finishes in about 2,700-3,000. Unanswered questions for the course
staff and the user:
- how credit scales with steps;
- whether the lift limit scales too;
- which objects count;
- whether reading the object mass from the simulator is allowed.

**Where it stands.**
- 818e59b (local, not pushed) raises `GraspConfig.squeeze` in
  `custom_libraries/grasp.py` to 0.060: the fingers are commanded 30 mm per
  finger past contact from the close phase onward, so the object is held at
  full force (about 250 N, 110-135 N for narrow objects; 6.6 N per mm of
  finger command offset) before the lift and the carry. Cause of the earlier
  losses: the object crept across the pads during the retreat and arm swing
  while the grip was still light. This is fix 30 in fixes.md.
- Sugar box, hammer, strawberry and apple (2 seeds each): delivered 2 of 8
  before, 6 of 8 after.
- Batch objects 1, 13, 6, 7, 14 x seeds 42 and 7: 7 of 10 delivered.
  Failures: sugar box seed 7 lost in state 67 at step 779; strawberry seed 42
  lost in state 68 at step 2,593; foam brick seed 42 lost in state 67 at step
  2,033.
- Objects 5 and 2 x both seeds: 4 of 4 delivered, at 2,709-2,929 steps.
- At 84202b1 on the user's machine (all 15 objects, maze seed 42, their
  `run.sh`): 3 of 15 delivered (foam brick 3,036 steps, banana 2,981, tomato
  soup can 3,043). Strawberry, potted meat can, apple, hammer and sugar box
  were lost in state 67 (carry), mostly near the start; mustard bottle and
  large marker found no grasp plan; master chef can, bowl and plate had no
  grasp candidates; bleach cleanser never starts; the mug ended on the floor.
  The earlier beec9b4 on the same machine delivered 5 of 15 at about
  12,200-12,700 steps (see part 2). A wider batch here at 84202b1 (objects 1,
  4, 13, 6, 7 x seeds 42 and 7) delivered 2 of 10.
- Still failing at planning: potted meat can (4) and master chef can (0). All
  60 candidates are rejected with IK failures and approach failures; master
  chef can seed 42 goes to state 4 at step 68.

**Tested, not adopted.**
- Slower arm swing (`tuck_rate` 0.005 instead of 0.012): sugar box and apple
  delivered but in more than 3,000 steps; hammer and strawberry still lost.
- Gripper-straight-down carry pose. The maze is passable with a robot
  footprint radius up to 0.38 m (plus the 5 cm margin) on maze seeds 42, 7, 1,
  2 and 3; the route is about 20-21 m (197-212 cells). The fingers-up pose has
  radius 0.289 m. A pose search up to 0.38 m found no pose with radius <=
  0.38 m and at least 0.10 m clearance between the held object and the robot;
  the closest had radius 0.372 m and clearance 0.098 m (gripper height 0.95 m,
  gripper xy (0.15, 0.25) in the base frame). The held object ends up 7-10 cm
  from the torso, or the arm hits the head. The fingers-up pose stays. (Older
  notes say no gripper-down pose fits; that was against the 0.289 m footprint.)

**Experiment: twice the carry speed (not committed).** Traces with
`tools/trace_carry.py` and `SOLVER_SET="carry_speed=0.036,carry_accel=0.002"`
(twice the driving speed, four times the ramp), on top of 818e59b:

| Trial | Result | Drift in grip |
|---|---|---|
| Apple seed 7 | delivered, step 1,989, no drop | 9.2 mm at the end of the carry |
| Soup can seed 42 | delivered, step 1,791 | 3.5 mm |
| Banana seed 42 | delivered, step 1,799 | 4.9 mm |
| Foam brick seed 7 | lost in state 67 at step 831 | 15.5 mm by step 700, 21.5 mm after 80 steps of driving |

- The foam brick had slid 15.5 mm (about 10 mm downward) before the base
  started driving, i.e. during the retreat and arm swing, then slipped out
  during an acceleration from a stop. So this loss starts in the swing, not at
  top speed.
- In the three deliveries the maze took 811-856 moving steps at a mean 26-27
  mm per step (peak 30 mm per step, the commanded 36 mm times the 83 % the
  base covers per step); before, about 1,700-1,900 steps.
- Step breakdown: lift done at about 350-410, state 67 starts at about
  395-460, navigation ends (state 68) at about 1,490-1,680, reach about
  280-310 steps.
- **Result (kept):** ten-trial reference batch at `carry_speed` 0.036,
  `carry_accel` 0.002: 8 of 10 delivered, mean 36.8/40, the same score as
  at 18 mm per tick, in 1,712-1,970 steps instead of 2,712-3,028. Lost:
  banana seed 7 (state 67, step 1,271) and tomato soup can seed 7 (state
  67, step 1,016), both mid-maze. Both foam brick trials were delivered.

**Next, toward 1,500.** The full-force grip holds at twice the speed on
straights and corners for three of four objects. The remaining 300-500 steps
must come from:
- the hold (60 steps);
- the arm swing before driving (state 67 starts about 290 steps before the
  base moves fast); the swing is also where the foam brick is lost;
- the reach (drive onto the goal cell, back off 7 cells, then reach).

**Paper reviews** are in `paper_review/` (`FInal_extraction.md`,
`FInal_synthesis.md`). Takeaways used: losses come from torsion about the
closing axis and from the object acting as an inverted pendulum in the
fingers-up pose, so the offset between the grasp centre and the centre of
mass matters.

**Operational notes.**
- `/tmp` is a 2.9 GB RAM-backed filesystem: snapshots must be code-only
  (`tools/batch.py` links `assets`).
- Each simulation uses about 900 MB; run at most 4 at once.
- Commits 47dfcb0 and 818e59b are local only. Do not push unless the user
  asks.

## Session 3, part 2: a 3,000-step episode

Date: 2026-11-19. Read this section after part 3, then the Session 3 section below
it. The goal changed after the first Session 3 day: the user requires the
WHOLE episode (pick, carry and reach) to fit in 3,000 `env.step` calls. It
took about 10,500. All the work below is on `fix/pick-nav-reach` and pushed
up to 84202b1. "Step" still means one `env.step` call.

**Where it stands.** Reference batch at 84202b1 (objects 5 6 7 14 2 x seeds
42 and 7, 10 trials, 4 in parallel, `tools/score.py`): 8/10 delivered, 36.8
of 40 mean points, deliveries at steps 2,712-3,028. One delivery (apple seed
7, step 3,028) is over the limit. The rows are in the results table in the
Session 3 section.

Per trial at 84202b1 (lift step / delivery step):

| Object | Seed 42 | Seed 7 |
|---|---|---|
| Banana (5) | 269 / 2,712 | 271 / 2,762 |
| Strawberry (6) | 351 / 2,719 | 364 / 2,829 |
| Apple (7) | 309 / 2,771 | 350 / 3,028 (over 3,000) |
| Foam brick (14) | 260 / lost in state 67 at step 563 (during the arm swing) | 264 / lost in state 67 at step 1,342 (mid-maze) |
| Soup can (2) | 266 / 2,721 | 278 / 2,846 |

At 897a4b2 the three losses were foam brick seed 42 and strawberry seed 7 in
the reach arm move (state 68) and strawberry seed 42 mid-maze (step 1,176).

### Why the mid-maze drops happened (replaces "cause not identified")

Measured with `tools/trace_carry.py` (added in a139817, extended in 61e2151).
It logs, per step: base displacement, finger forces, drift of the object in
the gripper frame split into x (toward the fingertips), y (along the closing
axis) and z (across the pads), and the angle between the closing axis and the
base motion. It prints EVENT lines on grip-force jumps, and the environment
variable `SOLVER_SET="name=value,..."` overrides solver attributes for
experiments without editing code.

- The env's position controller moves each joint 30 % of its remaining error
  per physics substep (5 substeps per step). Within one step the base's
  velocity therefore goes 1, 0.7, 0.49, 0.34, 0.24 of its first-substep value
  and then jumps back: a forward kick at the start of every step, which grows
  with speed (a velocity jump of about 66 x v m/s for v metres per step).
- Two caught apple drops (old carry pose, 3.3 mm per step): smooth constant
  base motion and no command changing. Drift grew from about 3 mm to 9-14 mm
  over 2,000-3,000 steps of driving, then the grip force fell from 50-60 N
  to zero in about 3 steps. There was no creep while the base stood still.
- In that pose the closing axis pointed straight down (world z component
  -0.96), so the kick ran across the pads. The pads are only 24 mm tall (12
  mm either side of centre). Delivered runs had drifted 6 mm or less.
- This corrects the Session 2 item 5 "ejection at high squeeze": it was most
  likely the same slide-off. A tighter grip was measured to be steady (see
  below).

### Changes, in commit order

1. **a70a4b7.** The solver no longer prints its state and every robot
   contact on each step.
2. **f5b81f3.** `GraspPlanner._valid` checks self-collision pair by pair
   (304 checked link pairs), skipping pairs whose bounding boxes are apart,
   instead of one robot-vs-robot closest-point query (about 20 ms, 95 % of a
   check). Same answer on 400 of 400 random poses (52 in collision); 0.9 ms
   per check instead of 13 ms.
3. **61e2151: the carry.** Judged by single traced runs when committed.
   - New carry pose (`carry_joint_targets`): gripper pointing straight up at
     1.6 m, inside the base footprint (0.289 m), so creep runs into the
     palm. Found by an IK search over gripper positions above the base; a
     random joint-space search found none in 180,000 samples. No
     gripper-straight-down pose fits the footprint (best 0.308 m, with the
     object against the torso). `shoulder_lift` is kept at -1.20, off its
     -1.221 limit: a joint held exactly at a limit is measured a hair past
     it, and the reach planner then rejected every configuration
     ("joint_limit" on the start state). `_held_object_validity` now also
     clips the start to the limits.
   - `_plan_tuck` / `_tuck_with_object`: the arm moves into the carry pose
     along a collision-checked path with the object attached (a straight
     joint-space edge if valid, else RRT-Connect). It shares
     `_held_object_validity` with the goal reach.
   - `_align_grip`: the wrist roll turns the closing axis to the direction of
     travel (modulo pi), during the hold before the retreat and through the
     maze, at 0.1 rad per tick. The base waits (speed factor 0) while the
     error is above 0.3 rad; below that, speed scales with cos^2 of the
     error. The state 3 hold lasts until it is aligned (cap 200 ticks).
   - `carry_grip_extra` = 0.030: during the state 3 hold the finger command
     is ramped a further 30 mm in (0.5 mm per tick). Measured on the held
     apple and soup can, grip force is linear in how far the finger command
     sits inside the object, 6.6 N per mm, and steady (min = max over 20
     steps) at every level: 50 N at the planner's squeeze, 76 N at +4 mm,
     103 N at +8, 129 N at +12, 182 N at +20, 218-248 N with the fingers
     commanded shut. Narrow objects such as the banana reach only about
     125-135 N because the command cannot go below 0.
   - `_ramped_speed` with `carry_accel` = 0.0005 m per tick^2 for the
     retreat, the maze and the reach backoff. `carry_speed`, `tuck_rate` and
     `reach_rate` are solver attributes.
4. **ec66857 and fc56bf5: pick speed-up** (done by a subagent).
   - `GraspFollower` follows the planned path with a moving target point:
     0.04 rad / 6 mm per step in transit, 0.01 rad / 3 mm in approach and
     lift, ramped over 8 steps, waiting only at phase ends and path corners.
     The integral term acts only on the error perpendicular to the path.
     The fingers open during the transit. The close phase ramps the finger
     command at 0.5 mm per step to the full squeeze and ends after 12 steps
     of bilateral contact, or when the fingers stall with both touching; it
     fails early with "Fingers stopped closing without bilateral contact".
     A one-step contact flicker at the top of the lift is tolerated.
   - State 999 tucks the arm while the object falls and ends once the object
     has stayed within 3 mm and 0.05 rad over 15 steps (200-step cap). State
     0 is now entered at step 62-160 instead of 200.
   - Lift step on the 10 reference trials: 253-364, median 297 (was
     1,035-1,267). Hammer (13) seeds 42 and 7 also pick now (lift 13.3-13.8
     cm).
   - Tried and not kept: lift at 3 mm per step without the ramp (banana
     lifted 13.5 cm, 11.6 mm grasp error); a per-finger opening check (it
     never converged on some grasps, because gravity shifts both fingers
     about 4.5 mm sideways); a 0.2 mm per step for 10 steps stillness test
     for state 999 (strawberry never met it).
5. **2db7f27 then 897a4b2: speeds.** `carry_speed` 0.004 -> 0.024 -> 0.018 m
   per tick commanded (the base covers 83 % of it); `tuck_rate` and
   `reach_rate` 0.003 -> 0.02 -> 0.012 rad per tick.
6. **a71f790, 82c09fd, beec9b4: `tools/batch.py`.** It does not copy
   `OUT_DIR` into its own snapshot (an `OUT_DIR` inside the repo used to nest
   without end), leaves `.claude` out, and links `assets/` into the snapshot
   instead of copying 150 MB.
7. **84202b1: reach order.** Candidate poses are tried least-gripper-rotation
   first (orientation outer loop, aim inner), because turning the gripper
   over again in the final move lost objects.

### Single-run evidence along the way

These are traced single runs, not rates.
- Fingers-up pose without alignment, 3.3 mm per step: the banana slid 38 mm
  into the palm and stayed, and was delivered. Apple seed 7: of the runs
  made, 1 delivered and 1 was lost mid-maze.
- Fingers-up + alignment, 3.3 mm per step: apple seed 42 drift 1.2-1.6 mm,
  delivered at step 9,937; apple seed 7 drift up to 10 mm, delivered at
  10,763.
- + alignment, 6.7 mm per step actual, 50 N grip: apple and soup can crossed
  the maze with constant drift (6.3 and 9.3 mm), so there is no creep while
  driving; the drift came from the retreat and the arm swing.
- 13.3 mm per step with the old step-change start and no pre-retreat
  alignment: the apple was flung in the first steps of the retreat (state
  5). Hence the alignment during the hold and the speed ramp.
- 10 mm per step actual, arm 0.003: apple 5,738 and soup can 5,811 steps,
  drift 1-2.5 mm.
- Arm at 0.012 rad per tick with the 50 N grip: apple and soup can both lost
  during the swing within about 200 steps. With the tight grip the same
  speeds delivered apple 4,215, soup can 4,352, banana 4,079.
- Carry 24 mm per tick, arm 0.02, tight grip, fast pick: single runs apple
  seed 7 2,566, soup can seed 42 2,308, banana seed 42 2,311, foam brick seed
  7 2,553, but the batch at those rates was only 4/10. Traces of two losses:
  the banana slid from 3 to 17 mm toward the fingertips in about 15 steps
  during the arm swing with the base still; the apple had slid 19 mm across
  the pads during the swing and crept to 22.6 mm while driving at 19 mm per
  step with the closing axis aligned and 230 N per finger.
- Step breakdown of a delivered run at 897a4b2-like settings (banana seed
  42): lift at 262, state 67 from about 417, state 68 from about 2,368,
  delivered at 2,686.

### Tried and rejected in part 2

- An orientation-controlled swing: rotating the gripper about the axis across
  the pads, through an IK chain of 12 waypoints from the lift pose to the
  carry pose. Every variant failed on IK or head collisions for the apple and
  soup can.
- A gripper-down carry pose: it does not fit the footprint.

### The user's run on another machine

Code at beec9b4, 15 objects, maze seed 42, `./run.sh --max-steps 15000`: 5/15
delivered (foam brick, strawberry, soup can, potted meat can, apple, at steps
12,213-12,742). Failures:
- master chef can, bowl, plate: "No antipodal grasp candidates found";
- mustard bottle, large marker: no plan (87-129 s of planning);
- banana, hammer: "Timed out during close" (the close phase has since been
  rewritten in ec66857; not re-run on that machine);
- sugar box: lost just after the retreat;
- bleach cleanser: never left state 0;
- mug: on the floor (height range error).

`run.sh` passes `--max-steps 1000` by default, which ends every trial during
the grasp; pass `--max-steps` explicitly (3,500 is enough for delivered
trials at the current speeds).

**Machine note.** The Claude Code scratch directory `/tmp` is a 2.9 GB
RAM-backed tmpfs on the dev machine. Full-repo snapshots there filled it and
caused the earlier memory shortage; keep snapshots code-only with assets
linked (`tools/batch.py` now does this).

## Session 3 update (rubric scorer, lift height, planning budgets, goal reach)

Date: 2026-11-18. Part 2 above supersedes this section's "Mid-maze drops"
text and its entry in "Next steps" (the Next steps list at the end of the
file has been rewritten). The numbers in this section describe the code at
380d21a and earlier. The Session 2 section and the rest of the file are kept
for reference, and the Session 2 results table is superseded by the one
below.

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
- One collision check (`GraspPlanner._valid`) cost about 30 ms of CPU; since
  f5b81f3 (part 2) the self-collision part takes 0.9 ms instead of 13 ms.
- `/tmp` under Claude Code is a 2.9 GB RAM-backed tmpfs; keep snapshots there
  code-only (part 2).

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
are the number of trials (of 10) that earned that item. The last column
(added in part 2) is the step range of the delivered trials; the older rows
did not record it, except 380d21a, whose range comes from its per-trial
table below.

| Code state | Delivered | Mean points /40 | lift<=3000 | lift>=10cm | hold 50 | grasp pose | reach | no drop | delivery steps |
|---|---|---|---|---|---|---|---|---|---|
| Baseline, CPU clock only (a7173ac) | 5/10 | 25.4 | 9 | 0 | 9 | 9 | 5 | 8 | not recorded |
| + lift 15 cm only (not a commit state; 1 s RRT budget) | 4/10 | 29.2 | 9 | 9 | 9 | 9 | 4 | 7 | not recorded |
| + planning budgets + lift 15 cm (7b93fc6) | 6/10 | 34.0 | 10 | 10 | 10 | 10 | 6 | 7 | not recorded |
| + goal reach fix (380d21a) | 7/10 | 35.2 | 10 | 10 | 10 | 10 | 7 | 7 | 10,291-11,842 |
| + squeeze 2 mm past measured contact, ramped (REJECTED, not committed) | 2/10 | 12.2 | 3 | 4 | 4 | 4 | 2 | 2 | not recorded |
| + fingers-up carry, alignment, tight grip, fast pick, carry 24 mm/tick, arm 0.02 rad/tick (2db7f27) | 4/10 | 30.4 | 10 | 10 | 10 | 10 | 4 | 4 | 2,311-2,427 |
| carry 18 mm/tick, arm 0.012 rad/tick (897a4b2) | 7/10 | 35.2 | 10 | 10 | 10 | 10 | 7 | 7 | 2,646-2,948 |
| + least-rotation reach order (84202b1, current) | 8/10 | 36.8 | 10 | 10 | 10 | 10 | 8 | 8 | 2,712-3,028 |

Per trial at 380d21a (the per-trial table for 84202b1 is in part 2):

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

**Mid-maze drops: what was known on 2026-11-18 (superseded by part 2, which
gives the cause; the "likely mechanism" below was right).**
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
   (Session 3, part 2: most likely the same slide-off as the mid-maze drops,
   not an ejection by the squeeze; a tighter grip measured steady.)

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
- A full run with transport took about 9,000–10,000 steps, or 1–2 minutes (measured on a 2-core machine in session 1); at 84202b1 a delivered run takes about 2,700-3,000 steps.
- Use a small `--max-steps` when you only care about the pick; the lift now completes by about step 360.
- `run.sh` passes `--max-steps 1000` by default, which ends every trial long before delivery (during the grasp on the code the user ran, beec9b4); pass `--max-steps` explicitly (3,500 is enough for delivered trials at the current speeds).
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
| 999 | Tuck the arm into the travel pose while the object falls; leave once it has stayed within 3 mm and 0.05 rad over 15 steps (200-step cap) | → 0 |
| 0 | `NavigateToGrasp` decides NAVIGATE / PREPARE / GRASP | → 1 (GRASP), → 4 (no usable grasp) |
| 1 | Hold the latched pose, wait to settle, `GraspPlanner.plan()` | → 2 or 4 |
| 2 | `GraspFollower`: approach, close, lift (moving target point; see part 2) | → 3 or 4 |
| 3 | Hold the lifted object for at least `hold_before_transport` ticks (60) while the finger command ramps `carry_grip_extra` (30 mm) further in and the wrist roll turns the closing axis to the retreat direction (leaves when aligned, cap 200 ticks) | → 5 |
| 5 | Retreat `retreat_distance` (1.0 m) straight back, arm frozen, speed ramped (`_ramped_speed`) | → 67 |
| 67 | Move the arm into `carry_joint_targets` (gripper up) along a collision-checked path with the object attached (`_tuck_with_object`, at `tuck_rate`), rebuild the map with the table as an obstacle, then A* + `CurvedNavigator` with base speed capped at `carry_speed`, ramped, and slowed or stopped by `_align_grip` | → 68 when `curved_nav.status == "reached"` |
| 68 | `reach_goal()`: back off `reach_backoff_cells` (7) along the path, solve IK so the object (not the gripper) lands on the goal (least gripper rotation first), move the arm at `reach_rate` | holds |
| 4 | Failed; holds the last pose | terminal |

Tunables live in `Solver.__init__`:
- `hold_before_transport`
- `retreat_distance`
- `carry_speed` (0.018 m/tick commanded; the base covers 83 % of it)
- `carry_accel` (0.0005 m/tick^2), `carry_grip_extra` (0.030 m)
- `tuck_rate` and `reach_rate` (0.012 rad/tick each)
- `align_grip_to_travel`, `grip_align_rate` (0.1 rad/tick)
- `reach_backoff_cells`
- `travel_joint_targets` (driving to the table)
- `carry_joint_targets` (carrying through the maze: gripper up, 1.6 m)

The tuck rate while carrying is `_tuck_rate`, set from `tuck_rate` when
entering 67. The old values were 4 mm/tick and 0.003 rad/tick.

## Things that are easy to get wrong

- **Action format.** The action is 15 absolute joint targets.
  - Base world x = `qpos[0] − 2.0`.
  - Index 2 is base yaw.
  - Index 3 is the torso.
  - Indices 6–12 are the arm.
  - Indices 13 and 14 are the fingers.
- **`env.step` and `time.sleep`.** In `pick_nav_reach_env_multi.py` it only sleeps (1/240 s per substep) when `gui=True`; the original env always sleeps. The tools still monkeypatch `time.sleep`; do the same in any new script that might use the original env.
- **No integral action in the controller.** The env's position controller has gains of 0.3 and no integral term. Commanding the *measured* pose as a hold target makes the arm sag; this was bug 2. Always latch targets.
- **Navigation runs at a fixed heading.** The wall inflation uses the robot's *current* projected footprint. Any arm pose wider than about 0.3 m from the base centre seals the maze bottleneck. Always measure a new pose with `custom_libraries/wall_projection.robot_xy_polygons(env)`. The carry pose is gripper-up because no gripper-down pose fits (best 0.308 m against 0.289 m).
- **The held object isn't in the footprint.** The footprint ignores the object, so a long object can still brush the walls.
- **The base kicks forward at the start of every step.** The position controller closes 30 % of the remaining error per substep (5 per step), so the velocity runs 1, 0.7, 0.49, 0.34, 0.24 and then jumps back. The kick grows with speed and must reach the held object along the closing axis, not across the pads; hence `_align_grip`.
- **Do not hold a joint exactly at a limit.** It is measured a hair past it and the reach planner rejects the start state (`shoulder_lift` is held at -1.20 for this reason).
- **Grip force follows the finger command.** 6.6 N per mm inside the object, steady; the carry commands 30 mm further in than the planner's squeeze.
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
  Current score (84202b1): 8 / 10 delivered, 36.8 / 40 mean rubric points,
  deliveries at steps 2,712-3,028.
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
  `env.step` calls (the only counter in the env). It took ~1,100 calls for
  the pick and ~10,500 for a full run in session 1; now the lift is at step
  260-364 and a delivery at 2,712-3,028. The user now requires the whole run
  within 3,000 `env.step` calls. If the rubric means physics substeps (5 per
  call), the budgets are five times tighter; ask.

## Next steps, in priority order

Rewritten in session 3, part 2. Done: rubric scorer, lift height, goal reach
for the soup can, the mid-maze drops (cause found, carry rebuilt), the slow
grasp execution (lift now at step 253-364), the 3,000-step episode on 8 of
10 reference trials.

1. **Remaining losses are run-to-run, not object-specific.** The foam brick
   was lost in 2 of 2 batch trials at 84202b1 (seed 42 during the arm swing,
   step 563; seed 7 mid-maze, step 1,342), but four traced re-runs of the
   same code (seeds 42 and 7, twice each) all delivered, at 2,708-2,924
   steps. In one of them the brick crept steadily toward the fingertips
   (upward, out of the fingers-up grip) from about 2 mm to 37 mm over the
   second half of the maze and was still held at delivery; the other three
   drifted 1-11 mm. That upward creep while driving is a new pattern and is
   not explained yet. Repeat the batch (`--repeat 3`) to get a rate.
2. **Margin on 3,000 steps.** (2026-11-19: the target is now 1,500 steps; see
   Session 3, part 3 at the top.) Apple seed 7 was delivered at 3,028. The maze
   still takes about 1,950 steps; speed dips at corners and while the wrist
   realigns. The arm swing is the fragile part at higher rates (see the
   24 mm per tick batch, 4/10).
3. **Ideas not yet tried.**
   - Choose the grasp for the carry: a closing axis that can face the travel
     direction and needs little rotation into the carry pose.
   - Align the closing axis with the acceleration direction in corners and
     braking, not only the travel direction.
   - A mass-based speed limit, and the team's dynamic wrist tilt. Reading the
     object's mass from the simulator is a read-only query and has not been
     used so far; confirm with the team that it is allowed.
4. **Object coverage** (grading uses 5 objects from the provided 15). See
   the failures in the user's run in part 2. Also "Object moved; stop and
   obtain a fresh plan" was seen twice before the grasp (apple, foam brick);
   not investigated.
5. **Rubric interpretations.** `score.py`'s grasp-pose tolerance (10 cm) and
   "lift height measured from the resting height" are our readings of the
   rubric.
6. **Ask the teaching team.**
   - Whether "sim steps" means `env.step` calls or physics substeps; the user
     now requires the whole run within 3,000 `env.step` calls.
   - The carry-stability points: the rubric text gives "4 pts" and also "2
     pts each (max 8)".
7. **Paper review.** In progress in `paper_review/` (`FInal_extraction.md`
   written; `FInal_synthesis.md` being written).
8. **Widen the set of pickable objects.**
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
9. **Awkward starts.**
   - Bleach (8): still sliding at step 200 in the old state 999, and never
     left state 0 in the user's run. State 999 now waits for the object to
     rest (with a cap), so re-test it first.
   - Large marker (12): sunk in the tabletop (z = 0.595 < 0.626). Fail fast.
   - Master chef can (0): 102 mm across, gripper opens ~95 mm. Fail fast.
   - Bowl (9) and plate (11): "No antipodal grasp candidates found" in the
     user's run.
10. **Floor pick for the mug (object 10)**, which `env.reset()` bounces off
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
11. **Robustness.**
   - Navigator deadlock: on `blocked_or_off_path` the navigator returns zeros
     forever; add a replan or timeout (review).
   - `_check_holding` does not cover state 3; a loss during the hold goes
     unnoticed (review).
   - Declare the `hasattr`/`getattr` solver attributes in `__init__`.
12. **Housekeeping**, if the team agrees: the committed `pnr/` virtualenv,
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