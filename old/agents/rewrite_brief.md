# Rewrite brief: what we learned, what failed, what hurts

Written 2026-11-20 for the session that rewrites the solver from the pristine
course code. It collects what four sessions of debugging, one day of team work,
a library baseline and five code reviews established. Read this before
`agents/handoff.md` and `agents/fixes.md`, which hold the older history in
detail (state machine, each fix and its evidence).

Everything here is marked by how we know it: **measured** (a trace, a paired
batch, or a number read from the simulator), **confirmed** (traced through the
code by a reviewer), or **inferred** (reasoned, not shown).

## 1. The task and the limits we work under

- CS4278 Pick-Nav-Reach: a Fetch robot in PyBullet picks one YCB object from a
  table (or the floor, if it fell), carries it through a maze and brings it
  within 10 cm of a goal. `get_action` is called once per `env.step`.
- **Step budget: 3,000 `env.step` calls per episode** (the user's cap, set
  2026-11-20; it was 4,000 earlier that day). A delivery after 3,000 counts as
  a failure. Always pass `--max-steps 3000`; the tools default to 15,000.
- **Open question that can change the budget:** one `env.step` runs 5 physics
  steps (`pick_nav_reach_env.py:39`), and the rubric says "sim steps" (lift
  within 3,000, reach within 15,000). If the grader counts physics steps, the
  lift budget is 600 `env.step` calls and the reach budget is exactly 3,000.
  Not answered by the teaching team. Design for the lift to finish well under
  600 `env.step` calls so either reading passes (lifts today happen at step
  250–590).
- **Self-implemented code only** in our solver (assignment spirit, the user's
  rule). Planning libraries are allowed only in the separate baseline.
- **Graded scene:** likely a variant of `pick_nav_reach_env.py` with the 15
  provided YCB objects (the user's "likely"; unconfirmed). Maze layout, route
  length and object pose may differ. No per-object rules keyed on the object
  id; no tuning to one map.
- Do not change the environment files or any object, contact or dynamics
  parameter.

## 2. Where the code is

| Branch | Head | State |
|---|---|---|
| `fix/pick-nav-reach` | `4fafaea` | Pushed. The reference solver. |
| `worktree-agent-a084b126561bf781f` | `6c3d70f` | Local only. Team grasp and carry work; failed review, no gain. Mine it for ideas and tools, do not merge. |
| `worktree-agent-ab08aa9bcf33246a6` | `a7bbac6` | Local only. Library baseline solver and comparison tools (based on `d8208e0`). |

The worktrees under `.claude/worktrees/` hold uncommitted `scratch/HANDBACK.md`
notes and traces for each branch. They can disappear; nothing below depends on
them.

**Reference result** (`4fafaea` plus the cloud-seed setting; 14 objects × seeds
42 and 1 × cloud draws 300 and 301; run at 4,000 steps):

| Object | Delivered | Delivery step | Failure |
|---|---|---|---|
| tomato soup can (2) | 4/4 | 1,235–1,251 | |
| mustard bottle (3) | 4/4 | 2,102–2,161 | |
| strawberry (6) | 4/4 | 1,345–1,379 | |
| apple (7) | 4/4 | 1,282–1,291 | |
| bleach cleanser (8) | 4/4 | 2,194–2,261 | |
| bowl (9) | 4/4 | 1,225–1,720 | |
| mug (10) | 4/4 | 1,757–1,775 | floor pick; it bounces off the table |
| foam brick (14) | 4/4 | 1,240–1,248 | |
| banana (5) | 3/4 | 1,215–1,228 | lost in the maze drive |
| sugar box (1) | 1/4 | 1,971 | lost in the maze drive |
| potted meat can (4) | 0/4 | | lost in the maze drive |
| hammer (13) | 0/4 | | lost in the maze drive |
| plate (11) | 0/4 | | never lifted: base jams on a table corner, 4,000 steps in state 0 |
| large marker (12) | 0/4 | | never lifted |
| master chef can (0) | not run | | cannot be gripped |

Total 36/56. All deliveries fit in 3,000 steps. The team branch scored 35/56
on the same trials.

## 2a. How the current solver works

`Solver.get_action(env)` in `solver.py` is a state machine; the action is 15
absolute joint targets (base x, y, yaw; torso; head; 7 arm joints; 2 fingers).

| State | What happens | Exit |
|---|---|---|
| 999 | Arm goes to the travel pose while the object falls; wait until it has been still for 15 ticks (capped). Choose table or floor mode from where it rests. | → 0 |
| 0 | `NavigateToGrasp` samples grasps, picks a parking cell from which one is reachable, A* on a parking map (tabletop left out so the base can tuck under it), `CurvedNavigator` drives there, base turns to face the object. | → 1, or 4 |
| 1 | Hold a latched pose, wait for the object to settle, call `GraspPlanner.plan()`. | → 2, or 4 |
| 2 | `GraspFollower` executes the plan: open, transit, approach, close, lift (15 cm). Watches the object and fails with "Object moved" if it shifts before the close. | → 3 (31 for a floor pick), or 4 |
| 31 | Floor pick only: raise the object to carry height. | → 3 |
| 3 | Hold at least 60 ticks while the finger command ramps 30 mm further in (grip force) and the wrist turns the closing axis toward the retreat direction. Weigh the object from contact forces; heavier objects get slower carry rates. | → 5 |
| 5 | Retreat 1 m straight back from the table, arm frozen, speed ramped. | → 67 |
| 67 | Carry through the maze. Default is the yaw carry (`yaw_carry.py`): the arm stays in the grasp pose, gripper down, and the route, base yaw and wrist roll are planned together so the closing axis faces the direction of travel (the base's per-tick kick then pushes along the pads' normal, not across them). Fallback is the fixed carry: tuck the arm gripper-up along a checked path, A* with the table as an obstacle. | → 68 |
| 68 | Back off along the path, solve IK so the object (not the gripper) lands on the goal, move the arm. | holds |
| 4 | Failed; holds the last pose. | terminal |

**Grasp planning** (`custom_libraries/grasp.py`, `GraspPlanner.plan`):
1. Sample antipodal point pairs on the cloud (pairs whose normals oppose along
   the line between them, inside the friction cone, narrower than the gripper).
   Score by antipodal quality, top-down approach and distance from the centre
   of mass to the grasp line; keep a spread of locations and several wrist
   rolls per pair. If none: rim candidates (bowl), then flange candidates
   (plate: lower finger under the rim, pads tilted, the plate pivots on its
   foot ring as the fingers close).
2. For each candidate, in order: finger-opening check, pregrasp IK (numerical,
   with restarts), RRT-Connect transit from the current arm pose, Cartesian
   straight-line approach, closure check, straight lift with the object
   attached. All collision checks use PyBullet closest-point queries with a
   2 mm margin, and the simulator state is saved and restored around planning.
3. Tried at two grasp depths (10 mm, then 25 mm); if nothing plans, a second
   pass re-orders candidates by a cheap pre-check (fingers tight, palm in the
   object, support contact) and resamples with a larger cap.
4. Budgets are CPU time (60 s per depth), not wall time.

## 2b. Fixes already in the current code

`agents/fixes.md` lists 35 numbered fixes with their evidence. The ones that
shaped the design:

- **Getting to the grasp:** base alignment that finishes; latched hold targets
  (the env's controller has no integral term, so holding the measured pose
  sags); torso kept clear of the table edge; a failure state that does not
  crash; fingers that do not hit the object on the way in; grasps kept for
  wide objects; rim grasps for the bowl; flange grasp for the plate; floor
  pick for the mug.
- **Planning budgets:** CPU-time deadlines instead of wall time; larger transit
  and search budgets; self-collision check made cheap; pre-check ordering only
  as a fallback (as the first order it broke the bleach cleanser).
- **Holding:** fingers grip at the pad, not the tip; grasps near the centre of
  mass; lift of 15 cm; grip force raised from 50 N by commanding the fingers
  30 mm past the squeeze; grip ramped before the object can creep.
- **Carrying:** the cause of mid-maze drops was the base's per-tick position
  step kicking the object across the pads; fixed by ramped speeds, the closing
  axis aligned with travel, slower wrist rolls, mass-based speed limits and
  then the yaw carry, which removed the arm swing and cut about 500 steps.
  Driving on with an empty gripper is detected (bilateral contact gone for 30
  ticks).
- **Reaching:** IK targets the object rather than the gripper; the held object
  no longer blocks its own reach; the reach does not flip the gripper over.
- **Review fixes (`4fafaea`):** see section 7 for the three of them that
  introduced new defects.

## 2c. Why we have not reached every object

1. **Three objects are limited by the environment, not our code:** the master
   chef can is wider than the open gripper; the marker tunnels into the table
   and is thrown out; thin rims (bowl, plate) can only be gripped with 10–19 N.
2. **Three are lost in the drive for a reason we can measure but not predict:**
   sugar box, hammer and potted meat can sit on pad edges or corners because
   their collision shapes are not flat and parallel, there is no spinning
   friction, and so they pivot or creep out. Which cloud draw produces a grasp
   that survives is not predictable from anything observed before the drive.
3. **We tuned against noise.** Until the unseeded cloud was found (the last
   day), single trials were read as improvements or regressions. A day of
   grasp and carry changes moved the paired total from 36/56 to 35/56.
4. **The solver's structure resists the fixes it needs.** Retry, timeouts and
   watchdogs bolted onto a state machine with lazily created state produced
   code that failed review (stale state on a second pick, unchecked motions).
5. **Planning effort is misdirected.** Each attempt sees a different cloud,
   tall objects burn the budget on palm-in-object candidates, and rim grasps
   switch off when a stray antipodal pair appears.
6. **Process:** parallel agents on interacting code, a machine that can run
   one trial at a time, and no tests on the solver.

## 3. Facts about the environment (measured unless marked)

These are limits, not bugs of ours. Design around them.

- **Master chef can cannot be pinched.** Fully open, the pads have a clear gap
  of 102.82 mm. The can's collision shape is 103.0 mm at its narrowest (median
  103.7). The mesh is a closed cylinder, 142 mm tall, no lip. The only route
  left is pushing it without lifting; the user has not decided on that.
- **The large marker tunnels into the table.** It is dropped end-on from z = 5
  at about 7.5 m/s, goes into the 52 mm tabletop (0.574–0.626 m) and stays
  wedged upright with 34 of its 121 mm showing, at about (0.027, 0.004, 0.595)
  on every seed. When the arm comes near, before any finger contact, it is
  thrown out at over 100 rad/s and lands on the floor, sometimes under the
  table where the base cannot park. Cause not proven (suspected: the solver
  island it shares with the arm). `p.restoreState` alone kicks it 17 mm.
- **Thin walls can only be held weakly.** The env clips finger targets to
  [0, 0.05] m with position gain 0.03: about 6.6 N per finger per mm the target
  is commanded past contact, and the target cannot go below fully closed. Bowl
  rim (8.6 mm): 18–19 N per finger. Plate flange (7 mm): 10–16 N. Other grasps:
  110–300 N. The full 30 mm squeeze needs a section at least 63 mm wide.
- **No spinning friction; lateral friction is 0.25** (finger 0.5 × object 0.5,
  multiplied by Bullet). One contact point per pad resists no rotation about
  the line between the pads.
- **Collision shapes differ from the point cloud at the mm level.** The sugar
  box's collision faces are 1.5° from parallel, so flat pads touch at an edge.
  The hammer's contact lands on the pad's tip corner although the cloud shows a
  flush line. Collision meshes are about 3 mm fatter than the cloud (reviewer).
- **The point cloud is sampled unseeded** (`pick_nav_reach_env_multi.py:158`,
  `trimesh.sample.sample_surface`). Every run plans on a different cloud and
  picks a different grasp, and the same object and seed flips between delivered
  and lost. With the cloud seeded, trials are bit-identical.
- **Seeds 42 and 7 are near-duplicates for the pick.** The drop does not depend
  on the maze; the seed changes only the maze. Coverage comes from more cloud
  draws and different mazes, not more seeds.
- **The mug bounces off the table onto the floor** on every seed; it needs the
  floor pick.
- **`saveState`/`restoreState` is not neutral.** Restoring perturbs the contact
  state of resting objects. Marginal grasps (bowl rim, apple) flipped outcome
  between `restoreState` and a hand-written write-back, in both directions.
- **The torso can jam on a tabletop corner** while driving round the table to a
  parking spot. The parking map allows for the torso's forward reach (0.088 m),
  not its radial reach (0.325 m).

## 4. Why objects are lost (measured from traces)

- Whether an object is delivered is decided by **the grasp chosen for that
  cloud draw**, not by the drive. Paired slow and fast carries lost the same
  draws; the slow carry only delayed the loss.
- **Sugar box:** rotates 25–115° about the pad-to-pad line under a steady
  ~300 N grip, although gripped within 1 mm of its centre of mass. Single-point
  contacts at pad corners. The squeeze itself drives part of it: base still,
  ~290 N lost 2 of 4 holds; 120 N and 50 N lost none.
- **Hammer:** both contacts at the pad tip corner (1.5 mm from the tip); slides
  toward the tips while driving. Grip only ~115 N. Lost on almost every draw.
- **Potted meat can:** slides out toward the fingertips. Behaves as if lost to
  per-tick kicks: a slow carry delivered it 8/8 against 5/8.
- **Plate:** rim at the very tip of the pads, 6–16 N; droops 0.25° per tick in
  the hold with the base still.
- **Low, flat objects:** a top-down pinch can only use the outer half of the
  pad, because the fingertips must stay above the table.
- **Apple:** one contact point per pad on a sphere; turns 7–18° either way.
- Earlier sessions (see `agents/fixes.md` 21–34): the base's per-tick position
  steps kick the held object across the pads; fast wrist rolls slide objects
  out; heavy objects need slower carries.

## 5. What was tried and did not work

Do not repeat these without a new idea. All on the six problem objects unless
noted, paired on seeded draws.

| Idea | Result |
|---|---|
| Carry as slowly as the step budget allows | 7/24 against 6/24; costs ~1,300 steps per delivery (2,700–3,100 total, which no longer fits in 3,000) |
| Slow carry when grip force per kg is low | potted meat 8/8 (from 5/8) but hammer 2/8 (from 5/8); no measure separates them beforehand |
| Slow carry once the object moves in the drive | fired on every trial: a blanket slow carry |
| Grip test in the hold, re-test at 120 N | changed no outcome; movement in the hold does not predict loss (a hammer that moved 15° was delivered, grasps that moved 1.3° were lost) |
| Lower grip force in the hold (120 N) | no delivery gained |
| Rank grasps by horizontal distance to the centre of mass | within noise; choices were already within 1–8 mm |
| Rank by predicted pad coverage | high-coverage candidates did not plan; banana lost in the lift |
| Close along the mean surface normal + per-candidate depth (cap 25 mm) | 6/24 → 11/24 in one paired run, no gain in the full regression; 28 mm depth lost the mustard in the lift |
| Deeper flange grasp for the plate | lost one draw, no plan on two |
| "Hang": turn a tilted grasp to point down after the lift | helped potted meat in traces; plate dropped during the turn; failed review (too broad, runs in the scored hold window) |
| Gentler retreat (18 mm/tick) | one draw each way (bowl lost, apple kept); not tested alone |
| Re-pick after "Object moved" | one marker delivery by floor pick at step 3,180 (over the 3,000 cap); failed review |

Published work (web research, 2026-11-20): nobody documents a method for our
hard cases. ManiSkill excludes the plate as "non-graspable/hard to grasp"; the
real-Fetch SceneReplica benchmark leaves out the master chef can, plate and
hammer. Recurring advice: rotation about the pad line is the main transport
failure, squeezing harder does little, grip nearer the centre of mass.

## 6. The library baseline (for the report, and as a check on us)

`solver_pbplan.py` on the baseline branch: pybullet_planning (IK, collision,
straight-line motion), OMPL via pb_ompl (transit, base), toppra (timing), each
stage switchable against ours. See `README_pbplan.md` there.

- Same candidates: ours found 187 plans, the library 162 (of 480).
- Whole trials (10 objects × 3 seeds, base motion ours): ours 19/30 delivered,
  library 14/30; median planning 8.5 s against 63 s.
- All-library including base motion (seed 42 only): 2/10.
- No bug found in our movement code. The 17 cases the library planned and ours
  rejected are our 2 mm collision clearance.
- Its failures are library weaknesses (IK without joint limits or restarts),
  not proof our code is right. Built before the seeded-cloud setting, so its
  whole-trial numbers carry noise.

## 7. Defects in the current solver (from the reviews)

The rewrite must not carry these over.

**Structure (`solver.py`)**
- `get_action` is a 350-line `if` chain on magic state numbers (999, 0–5, 31,
  67, 68); about 40 attributes are created lazily with `getattr`/`hasattr`;
  the reset list in `begin_pick` misses about ten of them, so a second pick
  inherits stale state. Five carry speeds are scaled in place.
- Exceptions escape `get_action`: `navigate()` raises when A* finds no route,
  and only one call site is wrapped. The batch runner hides this.
- No timeouts: the settle wait before planning, the 1 m blind retreat from the
  table (no map check), the maze drive when the base lags its reference, and
  the final reach can each wait until the step limit.
- Every failure goes to one terminal state. No retry.
- Crash if the object is at rest before the arm reaches the travel pose
  (`_travel_pose_complete` read before it exists).
- Scene constants hard-coded: base spawn offset 2.0 m (about ten places), goal
  height 0.90 m, 1 m walls on a 0.1 m grid, spawn height 4.0 m, floor test
  0.30 m.
- Planning runs inside a tick with CPU budgets up to 60 s × 3; wall time is
  unbounded.

**Grasp planner (`custom_libraries/grasp.py`)**
- Each depth, each pass and the retry draws a new random 800 of 1,024 points,
  so the "re-ordering" pass is not a re-ordering, and the base parks for
  candidates the planner never sees again. One shared rng is also consumed by
  IK restarts and the transit search.
- Top-ranked candidates on tall objects put the palm ~60 mm inside the object;
  the cheap grasp-pose check runs after the transit search (up to 8 s each).
- Rim grasps are generated only when the antipodal sampler returns nothing, so
  one stray antipodal pair switches them off for the bowl.
- The lift is collision-checked with the object at its pre-close pose; the
  follower stops watching the object at the close.
- The follower leaves the checked path by 1.5–2 cm at corners (0.02 rad
  tolerance) against a 2 mm margin; its integral term is never reset.
- Worst-case planning is 4 × 60 CPU-s; sampling is not charged to the deadline.
- Introduced by `4fafaea` (my review fixes): a failed flange attempt breaks out
  of the whole depth loop; the second pass is gated on the last depth's
  candidate kind; clearing the object history shortens the "object moved"
  allowance window from 40 samples to as few as 13.
- Flange: the far-edge watch is not latched; candidates depend on where the
  base stood when sampling; the lower-finger gap (2.5 mm) is calibrated on a
  different IK solution from the one executed (3 mm tolerance).

**Navigation and carry**
- The base turns in place at the parking spot with no collision check.
- A* plans on the parking map, but the path smoother checks a different map
  without the under-table zone.
- Two deadlocks: the navigator returns zero motion forever once the base is in
  an inflated cell; the robot can arrive within tolerance but outside the reach
  band and wait.
- Map inflation is a square, not a disc.

## 8. Measurement pitfalls (`tools/`)

- Default budget is 15,000 steps. Pass `--max-steps 3000`.
- The batch tool tabulates every log in the output folder: use a fresh folder
  per run.
- Success is latched at the first step within 10 cm; whether the object stays
  is never observed.
- Grasp-pose points are nearly free (10 cm tolerance, orientation unused). For
  flange grasps the pose error has a fixed ~35 mm offset (pivot not modelled).
- Lift height on a floor pick is measured from the floor.
- A crashed trial leaves no score line and is dropped from per-object
  denominators; an out-of-memory kill looks like a solver crash.
- `SOLVER_SET` is read from the shell and not recorded in results.
- The seeded-cloud setting (`PNR_CLOUD_SEED=N`, commit `4523897`) and
  `PLANNER_SET` exist only on the team branch. Port the first one early.
- Four trials per object cannot show a one-in-four improvement. Totals moved
  34 → 35 → 36 of 56 across a day while objects traded places.

## 9. Process pain points

- **Memory.** The WSL2 VM has 6 GB. Running two simulations beside background
  agents killed the session twice. One trial peaks at about 1.07 GB. Run one
  at a time, in a capped cgroup, behind a shared lock:
  `flock /home/shared/pnr_sim.lock systemd-run --user --scope --quiet -p MemoryMax=2000M -p MemorySwapMax=0 [-E VAR=value] pnr/bin/python tools/batch.py <fresh dir> ... -j 1 --max-steps 3000`
  About 50 minutes for 56 trials. Large paired runs belong on the user's Xeon
  machine (results come back under `xeon-results/`).
- **Parallel agents on the same code.** Two agents with split file ownership
  still produced interacting changes (a planner snapshot change that lost the
  bowl; a retreat change that traded the bowl for the apple). The result was
  900 lines with no gain. One writer at a time; regress the combined code.
- **Single unseeded trials misled us repeatedly.** A "plate regression" was a
  failure the old code also had on those draws; several "improvements" were one
  trial. Always run the old code on the same seeded draws.
- **Tuning on single draws.** Retreat values were flipped twice across commits.
  Magic numbers justified by one trial accumulated.
- **Reviews after the fact found what tests did not.** No solver code is unit
  tested; the planner test's fake simulator hides collision geometry.
- The user prefers short agent briefs.
- `tools/batch.py` copies the working tree to a snapshot and runs that; delete
  each `snap/` afterwards.

## 10. Requirements for the rewrite

Agreed direction (2026-11-20, user asked for a fresh rewrite from pristine;
the port-versus-retype question was put to the user and not yet answered):
new structure from the pristine course code, proven algorithms ported module
by module with the review fixes applied, each port gated by a paired
regression against `4fafaea`.

- One object per state with an entry, a step, a step budget and a declared
  failure exit. An episode-level budget of 3,000 steps that every state knows.
- A per-pick context created fresh for each attempt, so a retry cannot inherit
  state. One config object, never mutated in place.
- No exception leaves `get_action`.
- World and robot frames read from the simulator, not assumed. No scene
  constants (spawn offset, goal height, wall size, table position).
- One point-cloud sample per pick, shared by parking and planning; a seeded
  mode for tests.
- Cheap rejections (palm in object, grasp pose) before the transit search.
  Rim grasps available when antipodal candidates fail to plan.
- Verify the object pose after the close before lifting.
- A bounded retry: on a failed grasp, another candidate or parking spot, within
  the step budget.
- Scorer fixed first (first-crossing success, floor-pick lift reference,
  crashes counted, settings recorded), so the reference numbers are right.
- Keep what works and was hard to find: the yaw carry and its speed ramps, the
  mass-based carry speed, the grip force in the hold, the flange grasp for the
  plate, the floor pick for the mug, CPU-time planning deadlines.

## 11. Open questions

1. Step unit: `env.step` calls or physics steps? (Ask the teaching team.)
2. Master chef can: push to the goal, or record as ungraspable? (User.)
3. Rewrite scope: port existing algorithms into a new structure, or retype
   everything? (User.)
4. What separates a hammer or sugar-box grasp that survives the drive from one
   that does not? Nothing measured before the drive predicts it.
5. Why the wedged marker is thrown out when the arm approaches.
