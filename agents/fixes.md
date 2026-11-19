# Fixes on `last`: what was wrong and how it was fixed

The task (from the project PDF) has three parts. The Fetch robot must:

1. generate antipodal grasps, solve IK and pick the object up from the table;
2. carry it through the maze;
3. bring it within 10 cm of the green goal sphere.

The solver is a state machine:

| State | What it does |
|---|---|
| 999 | Wait for the object to land |
| 0 | Drive to the table |
| 1 | Settle, then plan a grasp |
| 2 | Execute the grasp |
| 3 | Holding the object |
| 4 | Failed |
| 5 | Retreat from the table (new) |
| 67 | Navigate the maze |
| 68 | Place the object on the goal (new) |

## Results

Headless runs on seeds 42 and 7. A success is the planner's own state 3, holding the lifted object for 100 steps.

| Stage | Before | After |
|---|---|---|
| Valid grasp start state | 0 / 30 | 30 / 30 |
| Pick (all 15 objects × 2 seeds) | 0 / 30 | 9 / 30 in the full sweep. The 5 small objects picked 10 / 10 in the later end-to-end runs. |
| Route through the maze exists | never (A* always failed) | yes |
| Object delivered within 10 cm of goal (5 pickable objects × 2 seeds) | 0 | 2 / 10 (banana and strawberry, seed 42) |

Current state (session 3, part 2, 84202b1), scored with `tools/score.py` on
the reference batch (objects 5, 6, 7, 14, 2 × seeds 42 and 7): **8 / 10
delivered, 36.8 / 40 mean rubric points** per trial, with deliveries at
steps 2,712–3,028 (one, apple seed 7 at 3,028, is over the 3,000-step limit
the user now requires for the whole episode). Before part 2 it was 7 / 10 and
35.2 points (380d21a), with deliveries at about 10,300–11,800 steps. See
handoff.md, Session 3 and Session 3, part 2, for the full table.

Update 2026-11-19 (part 3): the user's target is now 1,500 steps for the whole
episode. After fix 30 (818e59b, local) the objects 5 and 2 batch delivered 4 /
4 in 2,709-2,929 steps, and the 1, 13, 6, 7, 14 batch 7 / 10. The 8 / 10 above
is at 84202b1 and was not re-run on the full reference batch with fix 30.

The objects that pick reliably are banana, strawberry, apple, foam brick and tomato soup can. Runs are not deterministic, so expect some run-to-run variation.

## How it was debugged

1. **Make it observable.** A headless runner played one episode (one object, one seed) and printed one summary line:
   - the states visited;
   - the final state;
   - the planner's counters: candidates tried, and why each was rejected (IK, approach, closure, transit…).

   That turns "it doesn't work" into "it is stuck in state X for reason Y".
2. **Fix the earliest failure first.** Each bug hid the next one. For each, I logged the exact condition that failed, found why, fixed it, then re-ran.
3. **Sweep everything.** I ran all 15 objects × 2 seeds after each round, so a fix for one object couldn't silently break another.

## The bugs, in the order they were hit

### 1. Base alignment never finished (`navigate_to_grasp.py`, `_prepare`)

**Symptom.** Every run timed out in state 0: "Base alignment/settling timed out" after 600 ticks.

**Cause.** The alignment step snapshots all 15 joints on its first tick, while they are still moving. It then waits for *every* joint, fingers included, to return within 0.01 of that snapshot. One finger settled 2 cm away, so the check could never pass.

**Fix.** This step only moves the base, so only base x/y are compared against the snapshot. The other joints just have to be still.

### 2. The arm sagged onto the table before planning (`solver.py`, state 1)

**Symptom.** The planner rejected the start pose as already colliding. This is the "gripper still within table frame" problem from the last commit message.

**Cause.** While waiting to settle, the solver re-set the hold target to the *measured* joint positions every tick. The position controller has no integral term, so each tick gravity pulled the arm a little lower, and the target followed it down. The gripper fell from z = 0.91 m to 0.63 m; the tabletop is at 0.626 m. "Settled" only became true once the gripper was lying on the table.

**Fix.** Latch the hold pose once, when entering state 1, and keep commanding that same pose. The gripper now stays at about z = 1.0 m.

### 3. The torso parked against the table edge (`solver.py`, `navigate_to_grasp.py`)

**Symptom.** The start state was invalid again, this time with the torso in contact with the table.

**Cause.** The parking map leaves the tabletop out on purpose, so the low base can tuck under it. But the torso is tall, and its front reaches 8.8 cm ahead of the base centre.

**Fix.**
- `get_extra_obstacle_bounds` records the overhead boxes it skips.
- The solver measures how far the robot links that overlap the tabletop's height reach forward (0.088 m).
- `NavigateToGrasp` only accepts parking cells at least that reach + 3 cm + the arrival tolerance from the table.

The torso now parks 19 cm clear.

### 4. The failure state crashed the episode (`solver.py`, state 4)

State 4 returned `None`, and `env.step(None)` raises. It now holds the last commanded pose.

### 5. The fingers hit the object on the way in (`grasp.py`)

**Symptom.** With a valid start, every plan still failed. The main reason was `approach_fail`: fingers within about 1 mm of the object during the straight-line approach.

**Cause.** The gripper opened only 4 mm wider than the object on each side. The 2 mm collision margin used up half of that. The object's collision shape (a convex decomposition) is also a few mm fatter than the point cloud the grasps are sampled from.

**Fix.**
- `opening_clearance` 0.008 → 0.024. The gripper now opens 12 mm per side; it has 10.3 cm of travel.
- `contact_penetration` 0.0015 → 0.006. Fingers stop at the fatter collision shape, so the closure check must tolerate that.
- `squeeze` 0.001 → 0.004, for a firmer hold.
- An integral term in `GraspFollower`. The arm stalled 0.0085 rad short of its target (the threshold is 0.008) under gravity, so the "open" step timed out.

### 6. Wide objects lost all their grasps (`grasp.py`)

**Cause.** With the bigger opening, an object close to the gripper's maximum width (such as the master chef can) no longer fitted, so it got no candidates.

**Fix.**
- Candidates are filtered with a minimum clearance (`min_opening_clearance` = 0.008).
- The gripper opens by the full clearance when its travel allows, and by whatever travel is left otherwise.

### 7. Bowl and plate rim grasps were all rejected (`grasp.py`)

**Cause.** The "something is in the way of the opening" test treated *every* object point outside the contact span as an obstruction. That includes the far wall of a bowl, which is nowhere near the fingers.

**Fix.** Only points within opening clearance + finger thickness of the contacts count as obstructions.

### 8. A grasp-selection error crashed the episode (`solver.py`, state 0)

`NavigateToGrasp` raises `RuntimeError` when there is no candidate, or when the grasp height is out of range. The solver now catches it and goes to state 4 instead of crashing.

The mug's height error is the environment, not the code. With `env.reset()`, the mug bounces off the table onto the floor at about (−2.0, −2.07).

### 9. Navigation could never find a route through the maze (`solver.py`)

**Symptom.** A* returned an empty path even at time 0, with no object in hand.

**Cause.**
- Navigation keeps the base at a fixed heading and inflates the maze walls by the robot's whole projected footprint.
- The travel pose held the arm 0.84 m in front of the base.
- The maze's narrowest passage leaves about 0.5 m of clearance from its centre line.

So the inflated map sealed the passage completely.

**Fix.** A separate carry pose (`carry_joint_targets`), found by searching joint space, is used only after the pick. Its requirements:
- every link stays inside the base footprint (≤ 0.29 m);
- the gripper is held up at about 1.5 m;
- the gripper stays at least 0.2 m clear of the body.

The pose used to drive to the table is unchanged, because the grasp planner plans its paths from it.

### 10. Transport after the pick was not implemented (`solver.py`, new states 5 and 68)

State 3 used to be a terminal hold. The new sequence is:

1. **Hold** the lifted object for 60 ticks.
2. **Retreat** (state 5) 1.0 m straight back, with the arm frozen, so the object clears the table.
3. **Rebuild the map** with the table as a full obstacle, so the base no longer routes under it.
4. **Re-tuck slowly** (0.003 rad/tick, torso 2 mm/tick), with the fingers kept squeezing. Faster tucks fling the object. (Changed in part 2: see fixes 21–24.)
5. **Navigate** with the base capped at 4 mm/tick while carrying. The navigator otherwise commands up to about 0.4 m per tick, which jerks a friction-held object loose. (Now 18 mm/tick with a speed ramp and an aligned grip; see fixes 21–24.)
6. **Reach** (state 68):
   - back off 7 cells along the arrival path;
   - solve IK (the planner's private torso + arm model) for a gripper pose that puts the *object*, not the gripper, on the goal;
   - move the arm there along a collision-checked path (see §14, and §19 and
     §20 for later fixes). The first version was a slow joint-space
     interpolation, which swung the object into the walls.

## Session 2: holding the object through transport

Measured with `tools/trace_hold.py`. End-to-end deliveries on the reference
batch (objects 5, 6, 7, 14, 2 × seeds 42 and 7) went from 2 / 10 to 5 / 10.

### 11. Fingers gripped at the pad tip (`grasp.py`)
The contact was planned 6.5 mm from the fingertip (`tcp_offset` 0.025 on a
63 mm pad), so objects slid onto the tip edge during the lift. `plan()` now
tries `tcp_offsets = (0.010, 0.025)`: 21 mm from the tip first, the old depth
only as a fallback.

### 12. Off-centre grasps pivoted out (`grasp.py`, `navigate_to_grasp.py`)
Finger contacts have no spinning friction. Candidates are now ranked down by
the distance from the centre of mass to the grasp line (`com_weight` 4.0).

### 13. Weak squeeze (`grasp.py`)
`squeeze` 0.004 → 0.012. This slows creep but is the suspected cause of
occasional ejections; see handoff.md, next step 3.

### 14. The reach swung the object into the maze walls (`solver.py`, state 68)
The final arm move was a joint-space interpolation. It is now an RRT-Connect
path with the object attached and the walls as obstacles, aiming at the goal
or points 5 cm from it, with 3 planning attempts.

### 15. Driving on with an empty gripper (`solver.py`)
States 5, 67 and 68 go to state 4 after 30 ticks without bilateral finger
contact; `grasp_error` records where the object was lost.

Not used, by team decision: PyBullet's `frictionAnchor` on the finger links.
It removes the creep completely but changes the robot's contact model.

## Session 3: scoring, lift height, planning budgets and the goal reach

Measured with `tools/score.py` and `tools/batch.py` on the reference batch.
Delivered 5 / 10 (25.4 points) with only fix 16, 6 / 10 (34.0) after 17 and
18, 7 / 10 (35.2) after 19 and 20.

### 16. Planning deadlines used wall time (`grasp.py`, `solver.py`)
**Symptom.** Six parallel banana trials all returned "No plan found" with
reason `time_budget` after about 20 candidates; a first baseline batch run
6-wide scored 2 / 10, with 3 trials failing at planning.
**Cause.** The deadlines of the grasp planner and the goal reach counted wall
time, so a busy machine used up the budget on other processes' work.
**Fix.** `planning_clock = time.process_time` (CPU time) for every planning
deadline. Planning time costs no simulation steps.

### 17. Transit and search budgets too small (`grasp.py`)
**Symptom.** The planner failed, or fell through to a late candidate (the
banana picked candidate 12 or 23 instead of 1 and was then dropped).
**Cause.** With a generous budget, 28 of 29 banana transit plans were plain
straight-line edges, but each took 1–2.5 CPU-s just to collision-check
(50–70 samples at about 30 ms each), so a 1 s budget rejected them on a busy
machine.
**Fix.** `rrt_seconds` 1 → 8 and `planning_seconds` 30 → 60 (per grasp
depth). No candidate in the reference batch was rejected for transit
afterwards.

### 18. Lift of 8 cm, accepted at 4 cm (`grasp.py`)
**Symptom.** The rubric wants the object at least 10 cm up; the lift was
planned at 8 cm and accepted at 4 cm.
**Fix.** `lift_distance` 0.08 → 0.15 and the lift is accepted at
`min_lift_fraction` (0.8) of it, 12 cm. Lifts now measure 14.8–15.5 cm and
sag at most 3 mm over the 50-step hold.

### 19. The held object blocked its own reach (`solver.py`, `_solve_reach`)
**Symptom.** Soup can, seed 7: 0 of 6 reach plan calls succeeded.
**Cause.** Not IK. Held top-down, the can creeps up during the carry until it
is 0.3 mm from the palm (`gripper_link`), inside the planner's 2 mm collision
margin. Fingers were exempt from the held-object check but the palm was not,
and since the object moves rigidly with the gripper in the planner, every
configuration including the start was invalid.
**Fix.** The held object is not checked against the gripper links. It is
still checked against the walls and the rest of the robot. Offline, 0 / 6 →
8 / 8 plans for this case.

### 20. Reach search spent its budget on one RRT (`solver.py`, `_solve_reach`)
**Symptom.** Soup can, seed 42: the old search failed 2 of 5 calls.
**Cause.** One RRT used 6 s of the 20 s budget, so candidates with a clear
straight-line move were never tried. Standoff distance, base heading and goal
height were not the cause (the base stops 0.62 m from the goal).
**Fix.** Solve IK for all candidates first, return the first with a
collision-free straight move, then run RRT nearest-first. Budget 20 → 60
CPU-s. Offline, 3 / 5 → 8 / 8 plans, 2.5–7.2 CPU-s each.

### Tried, not kept in session 3: squeeze relative to the measured contact
The hold command was the finger positions measured at the end of the close
phase minus 2 mm per finger, ramped at 0.2 mm per tick during the lift.
Delivered 2 / 10 (12.2 points): banana, apple seed 7 and foam brick seed 7
lost contact during the lift; soup can (both seeds) never rose; apple and
foam brick seed 42 slid 26–29 mm and were dropped. The finger position gain
is 0.03 (arm joints 0.3), so a 2 mm offset gives far too little force; the
committed command (7–9 mm inside the collision hull) gives 45–60 N. Not
committed; an intermediate value and the ramp alone are untested.


## Session 3, part 2: a 3,000-step episode

The user requires the whole episode to fit in 3,000 `env.step` calls; it took
about 10,500. Work of 2026-11-19, measured with `tools/trace_carry.py` and
the reference batch. Delivered 4 / 10 (30.4 points) after fix 21–23 and 25
with fast speeds (2db7f27), 7 / 10 (35.2) at the slower speeds of fix 24
(897a4b2), 8 / 10 (36.8) after fix 29 (84202b1).

### 21. Mid-maze drops: the base kicked the object across the pads (`solver.py`)
**Symptom.** Objects were lost in about 3 of 10 trials while the base drove.
Two caught apple drops: smooth base motion at 3.3 mm per step, drift of the
object in the gripper growing from about 3 mm to 9–14 mm over 2,000–3,000
steps, then the grip force falling from 50–60 N to zero in about 3 steps.
**Cause.** The env's position controller moves each joint 30 % of its
remaining error per physics substep (5 per step), so the base's velocity in
a step goes 1, 0.7, 0.49, 0.34, 0.24 of its first-substep value and then
jumps back: a forward kick at the start of every step (about 66 × v m/s for
v metres per step). In the old carry pose the closing axis pointed straight
down (world z component −0.96), so the kick ran across the 24 mm-tall pads
and the object crept off them. This was most likely also the "ejection at
high squeeze" of session 2.
**Fix.**
- A new carry pose (`carry_joint_targets`): gripper straight up at 1.6 m
  inside the base footprint (0.289 m), found by an IK search (a random
  joint-space search found none in 180,000 samples; no gripper-down pose
  fits, best 0.308 m), so creep ends at the palm.
- `_plan_tuck` / `_tuck_with_object` move the arm into that pose along a
  collision-checked path with the object attached (a straight joint-space
  edge if valid, else RRT-Connect), sharing `_held_object_validity` with the
  goal reach.
- `_align_grip` turns the wrist roll so the closing axis lies along the
  direction of travel (modulo pi), during the hold before the retreat and
  through the maze, at 0.1 rad per tick. The base waits while the error is
  above 0.3 rad; below that its speed scales with cos² of the error.
Evidence is from single traced runs, not rates: with the new pose and no
alignment the banana slid 38 mm into the palm and was delivered; with
alignment the apple drifted 1.2–1.6 mm (seed 42) and up to 10 mm (seed 7)
and was delivered; at 6.7 mm per step the drift stayed constant through the
maze (6.3 and 9.3 mm), so the drift came from the retreat and the arm swing.

### 22. Grip force only 50 N (`solver.py`, state 3)
**Cause.** Grip force is linear in how far the finger command sits inside
the object: 6.6 N per mm, steady (min = max over 20 steps) at every level
measured on the held apple and soup can. 50 N at the planner's squeeze, 76 N
at +4 mm, 103 N at +8, 129 N at +12, 182 N at +20, 218–248 N with the fingers
commanded shut (narrow objects such as the banana reach 125–135 N, because
the command cannot go below 0).
**Fix.** `carry_grip_extra` = 0.030: during the state 3 hold the finger
command is ramped a further 30 mm in, at 0.5 mm per tick. With the 50 N grip
the apple and soup can were lost during the arm swing at 0.012 rad per tick;
with the tight grip the same speeds delivered the apple (4,215 steps), soup
can (4,352) and banana (4,079).

### 23. Reach planner rejected the new carry pose (`solver.py`)
**Symptom.** Every reach from the fingers-up pose failed with "joint_limit"
on the start state.
**Cause.** A joint held exactly at a limit is measured a hair past it.
**Fix.** `shoulder_lift` is kept at −1.20, off its −1.221 limit, and
`_held_object_validity` clips the start configuration to the limits.

### 24. Carry was slow and unramped (`solver.py`)
**Fix.** `_ramped_speed` with `carry_accel` = 0.0005 m per tick² for the
retreat, the maze and the reach backoff; a step-change start flung the apple
in the first steps of the retreat at 13.3 mm per step. `carry_speed`,
`tuck_rate` and `reach_rate` became solver attributes. Rates: `carry_speed`
0.004 → 0.024 → 0.018 m per tick commanded (the base covers 83 % of it),
`tuck_rate` and `reach_rate` 0.003 → 0.02 → 0.012 rad per tick (2db7f27,
then 897a4b2). At 24 mm per tick and 0.02 rad per tick single runs delivered
at 2,308–2,566 steps, but the batch was 4 / 10: the banana slid 3 → 17 mm
toward the fingertips in about 15 steps during the arm swing with the base
still, and the apple slid 19 mm across the pads in the swing and then crept
to 22.6 mm while driving at 19 mm per step with the closing axis aligned and
230 N per finger. The slower rates give 7 / 10.

### 25. Slow pick (`custom_libraries/grasp.py`, `solver.py`; ec66857, fc56bf5)
**Symptom.** The lift completed at step 1,035–1,267.
**Fix.**
- `GraspFollower` follows the planned path with a moving target point (0.04
  rad / 6 mm per step in transit, 0.01 rad / 3 mm in approach and lift,
  ramped over 8 steps), waiting only at phase ends and path corners. The
  integral term acts only on the error perpendicular to the path. The
  fingers open during the transit. The close phase ramps the finger command
  at 0.5 mm per step to the full squeeze and ends after 12 steps of bilateral
  contact or when the fingers stall with both touching; it fails early with
  "Fingers stopped closing without bilateral contact". A one-step contact
  flicker at the top of the lift is tolerated.
- State 999 tucks the arm while the object falls and ends once the object
  has stayed within 3 mm and 0.05 rad over 15 steps (200-step cap), so state
  0 starts at step 62–160 instead of 200.
Lift step on the 10 reference trials: 253–364, median 297. The hammer (13)
now picks for seeds 42 and 7 (lift 13.3–13.8 cm).
Tried, not kept: lift at 3 mm per step without the ramp (the banana lifted
13.5 cm, 11.6 mm grasp error); a per-finger opening check (it never
converged on some grasps, because gravity shifts both fingers about 4.5 mm
sideways); a 0.2 mm per step for 10 steps stillness test for state 999 (the
strawberry never met it).

### 26. Self-collision check dominated planning time (`grasp.py`; f5b81f3)
**Cause.** `GraspPlanner._valid` made one robot-vs-robot closest-point query,
about 20 ms and 95 % of a check.
**Fix.** Self-collision is checked pair by pair (304 link pairs), skipping
pairs whose bounding boxes are apart. Same answer on 400 of 400 random poses
(52 in collision); 0.9 ms instead of 13 ms per check.

### 27. Per-step printing (`solver.py`; a70a4b7)
The solver no longer prints its state and every robot contact on each step.

### 28. Reach flipped the gripper over (`solver.py`; 84202b1)
**Symptom.** Objects were lost in the final arm move (foam brick seed 42 and
strawberry seed 7 in state 68 at 897a4b2).
**Cause.** The candidate order tried a centred aim in every orientation
before an off-centre aim in the current one, so it often chose a pose that
turned the gripper over again with the object in it.
**Fix.** Candidates are tried least-gripper-rotation first (orientation outer
loop, aim inner). Delivered 7 / 10 → 8 / 10.

### 29. `tools/batch.py` snapshots (a71f790, 82c09fd, beec9b4)
It no longer copies `OUT_DIR` into its own snapshot (an `OUT_DIR` inside the
repo used to nest without end), leaves `.claude` out, and links `assets/`
into the snapshot instead of copying 150 MB. Full-repo snapshots filled the
2.9 GB RAM-backed `/tmp` of the dev machine and caused the earlier memory
shortage.

### New tool
`tools/trace_carry.py` (a139817, extended in 61e2151) logs per step the base
displacement, finger forces, the drift of the object in the gripper frame
(x toward the fingertips, y along the closing axis, z across the pads) and
the angle between the closing axis and the base motion, with EVENT lines on
grip-force jumps. `SOLVER_SET="name=value,..."` overrides solver attributes
for experiments.

### Tried, not kept in part 2
- An orientation-controlled swing (rotating the gripper about the axis across
  the pads, through an IK chain of 12 waypoints from the lift pose to the
  carry pose): every variant failed on IK or head collisions for the apple
  and soup can.
- A gripper-down carry pose: it does not fit the footprint.

## Session 3, part 3: toward 1,500 steps

New target from the user (2026-11-19): the whole episode must finish within
1,500 `env.step` calls for full credit (it was 3,000). The code at 84202b1
finishes in about 2,700-3,000. Open questions for the course staff and the
user: how credit scales with steps, whether the lift limit scales too, which
objects count, and whether reading the object mass from the simulator is
allowed.

### 30. Objects crept across the pads while the grip was still light (`custom_libraries/grasp.py`; 818e59b, local, not pushed)
**Symptom.** Objects were lost in state 67 (carry), mostly near the start. In
the user's run of all 15 objects (maze seed 42, their `run.sh`, 84202b1) only
3 of 15 were delivered: foam brick (3,036 steps), banana (2,981) and tomato
soup can (3,043). Strawberry, potted meat can, apple, hammer and sugar box
were lost in state 67; mustard bottle and large marker found no grasp plan;
master chef can, bowl and plate had no grasp candidates; bleach cleanser never
starts; the mug ended on the floor. The earlier commit beec9b4 on the same
machine delivered 5 of 15, at about 12,200-12,700 steps. A wider batch here at
84202b1 (objects 1, 4, 13, 6, 7 x seeds 42 and 7) delivered 2 of 10.
**Cause.** The object crept across the pads during the retreat and the arm
swing, while the grip was still light; the full force came only later, during
the hold.
**Fix.** `GraspConfig.squeeze` raised to 0.060: the fingers are commanded 30
mm per finger past contact from the close phase onward. The object is held at
full force (about 250 N, 110-135 N for narrow objects; grip force is linear at
6.6 N per mm of finger command offset) before the lift and the carry.
**Result.**
- Sugar box, hammer, strawberry and apple (2 seeds each): delivered 2 of 8
  before, 6 of 8 after.
- Batch objects 1, 13, 6, 7, 14 x seeds 42 and 7: 7 of 10 delivered. Failures:
  sugar box seed 7 lost in state 67 at step 779; strawberry seed 42 lost in
  state 68 at step 2,593; foam brick seed 42 lost in state 67 at step 2,033.
- Objects 5 and 2 x both seeds: 4 of 4 delivered, at 2,709-2,929 steps.
- Still failing: potted meat can (4) and master chef can (0) fail at grasp
  planning. All 60 candidates are rejected with IK failures and approach
  failures; master chef can seed 42 goes to state 4 at step 68.

### Tried, not kept in part 3
- **Slower arm swing** (`tuck_rate` 0.005 instead of 0.012). Sugar box and
  apple were delivered, but in more than 3,000 steps; hammer and strawberry
  were still lost.
- **Gripper-straight-down carry pose.** The maze is passable with a robot
  footprint radius up to 0.38 m (plus the 5 cm margin) on maze seeds 42, 7, 1,
  2 and 3; the route is about 20-21 m (197-212 cells), and the current
  fingers-up pose has radius 0.289 m. A pose search allowing radius up to 0.38
  m found no pose with radius <= 0.38 m and at least 0.10 m clearance between
  the held object and the robot. The closest had radius 0.372 m and clearance
  0.098 m (gripper height 0.95 m, gripper xy (0.15, 0.25) in the base frame).
  The held object ends up 7-10 cm from the torso, or the arm hits the head.
  The fingers-up carry pose stays. (This is a different result from the older
  "does not fit the footprint" note, which assumed the 0.289 m footprint.)

### Experiment: twice the carry speed (2026-11-19, not committed)
Traced with `tools/trace_carry.py`, `SOLVER_SET="carry_speed=0.036,carry_accel=0.002"`
(twice the driving speed and four times the ramp), on top of 818e59b.
- Apple seed 7: delivered at step 1,989, no drop, drift in the grip 9.2 mm at
  the end of the carry.
- Tomato soup can seed 42: delivered at step 1,791, drift 3.5 mm.
- Banana seed 42: delivered at step 1,799, drift 4.9 mm.
- Foam brick seed 7: lost in state 67 at step 831. It had already slid 15.5
  mm in the grip (about 10 mm downward) by step 700, before the base started
  driving, i.e. during the retreat and arm swing. It crept to 21.5 mm over the
  first 80 steps of driving and then slipped out during an acceleration from a
  stop. This loss starts in the swing, not at top speed.
- In the three deliveries the maze took 811-856 moving steps at a mean 26-27
  mm per step (peak 30 mm per step, the commanded 36 mm times the 83 % the
  base covers per step); before, about 1,700-1,900 steps.
- Step breakdown: lift done at about 350-410, state 67 starts at about
  395-460, navigation ends (state 68) at about 1,490-1,680, the reach takes
  about 280-310.
- Conclusion: the full-force grip holds at twice the speed on straights and
  corners for three of four objects. The remaining 300-500 steps to reach
  1,500 must come from the hold (60 steps), the arm swing before driving
  (state 67 starts about 290 steps before the base moves fast) and the reach
  (drive onto the goal cell, back off 7 cells, then reach).
- **Result (kept):** ten-trial reference batch at `carry_speed` 0.036,
  `carry_accel` 0.002: 8 of 10 delivered, mean 36.8/40, the same score as
  at 18 mm per tick, in 1,712-1,970 steps instead of 2,712-3,028. Lost:
  banana seed 7 (state 67, step 1,271) and tomato soup can seed 7 (state
  67, step 1,016), both mid-maze. Both foam brick trials were delivered.

### Paper reviews
`paper_review/` (`FInal_extraction.md`, `FInal_synthesis.md`). Takeaways used:
losses come from torsion about the closing axis and from the object acting as
an inverted pendulum in the fingers-up pose, so the offset between the grasp
centre and the centre of mass matters.

## Known limitations / next steps

- **Foam brick is lost in 2 of 2 trials** at 84202b1 (seed 42 in state 67 at
  step 563, during the arm swing; seed 7 in state 67 at step 1,342,
  mid-maze). Not traced yet.
- **Little margin on 3,000 steps.** Apple seed 7 was delivered at 3,028. The
  maze still takes about 1,950 steps. The arm swing is the fragile part at
  higher rates (24 mm per tick gave 4 / 10).
- **Object coverage.** In the user's run on another machine (beec9b4, 15
  objects, maze seed 42) 5 / 15 were delivered. Master chef can, bowl and
  plate: "No antipodal grasp candidates found"; mustard bottle and large
  marker: no plan; banana and hammer: "Timed out during close" (the close
  phase has been rewritten since and not re-run there); sugar box: lost just
  after the retreat; bleach cleanser never left state 0; the mug is on the
  floor. "Object moved; stop and obtain a fresh plan" was seen twice before
  the grasp (apple, foam brick) and not investigated.
- **Bleach cleanser** never leaves state 0, with no error. State 999 has
  changed since; not re-tested.
- **Rubric questions** for the teaching team: whether "sim steps" means
  `env.step` calls or physics substeps; the carry-stability points wording.
- `run.sh` passes `--max-steps 1000` by default; pass `--max-steps`
  explicitly (3,500 is enough for delivered trials).
- See handoff.md, Next steps, for the full list. A paper review is in
  progress in `paper_review/`.

## Files changed

- `solver.py`: fixes 2, 3, 4, 8, 9 and 10 (session 1), 14 and 15 (session 2),
  19 and 20 (session 3; also the CPU-time deadline of fix 16), 21–25, 27 and
  28 (part 2).
- `custom_libraries/navigate_to_grasp.py`: fixes 1 and 3.
- `custom_libraries/grasp.py`: fixes 5, 6 and 7 (session 1), 11–13 (session
  2), 16, 17 and 18 (session 3), 25 (`GraspFollower`) and 26 (part 2), 30
  (`squeeze` 0.060, part 3).
- `tools/score.py` (new): scores one trial against the rubric.
- `tools/batch.py` (new): runs `score.py` over objects × seeds in parallel
  from a frozen copy of the repo; fix 29.
- `tools/trace_carry.py` (new): per-step carry trace (session 3 and part 2).
