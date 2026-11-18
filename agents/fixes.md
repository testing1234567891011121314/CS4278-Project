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

Current state (session 3, 380d21a), scored with `tools/score.py` on the
reference batch (objects 5, 6, 7, 14, 2 × seeds 42 and 7): **7 / 10
delivered, 35.2 / 40 mean rubric points** per trial. See handoff.md, Session
3, for the full table.

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
4. **Re-tuck slowly** (0.003 rad/tick, torso 2 mm/tick), with the fingers kept squeezing. Faster tucks fling the object.
5. **Navigate** with the base capped at 4 mm/tick while carrying. The navigator otherwise commands up to about 0.4 m per tick, which jerks a friction-held object loose.
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

### Tried, not kept: squeeze relative to the measured contact
The hold command was the finger positions measured at the end of the close
phase minus 2 mm per finger, ramped at 0.2 mm per tick during the lift.
Delivered 2 / 10 (12.2 points): banana, apple seed 7 and foam brick seed 7
lost contact during the lift; soup can (both seeds) never rose; apple and
foam brick seed 42 slid 26–29 mm and were dropped. The finger position gain
is 0.03 (arm joints 0.3), so a 2 mm offset gives far too little force; the
committed command (7–9 mm inside the collision hull) gives 45–60 N. Not
committed; an intermediate value and the ramp alone are untested.

## Known limitations / next steps

- **Objects are still lost during transport in about 3 of 10 trials.** The
  drops are intermittent. Two traced apple drops show the object creeping
  in the gripper while the base drives (3 mm to 9–14 mm of drift), then
  sliding off the 24 mm-tall finger pads; in the carry pose the closing axis
  points straight down. Runs that delivered drifted 1–6 mm. A fingers-up
  carry pose is the next thing to try; see handoff.md, Session 3, and
  `tools/trace_carry.py`.
- **Planning still fails for some objects.** Sugar box, large marker and potted meat can mostly end with `ik_fail` / `transit_fail`. Candidate ranking and approach direction for tall boxes need work.
- **Bleach cleanser never leaves state 0**, with no error. Not yet investigated.
- **Long runs.** A full episode with transport is about 10,300–11,800 steps.
- **Slow grasp execution** in some trials (lift at step 2,425 and 2,494
  against the 3,000 limit); see handoff.md, next step 5.

## Files changed

- `solver.py`: fixes 2, 3, 4, 8, 9 and 10 (session 1), 14 and 15 (session 2),
  19 and 20 (session 3; also the CPU-time deadline of fix 16).
- `custom_libraries/navigate_to_grasp.py`: fixes 1 and 3.
- `custom_libraries/grasp.py`: fixes 5, 6 and 7 (session 1), 11–13 (session
  2), 16, 17 and 18 (session 3).
- `tools/score.py` (new): scores one trial against the rubric.
- `tools/batch.py` (new): runs `score.py` over objects × seeds in parallel
  from a frozen copy of the repo.
