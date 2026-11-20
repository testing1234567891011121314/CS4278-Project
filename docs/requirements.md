# Requirements for the solver rewrite

Each requirement has an id that tests and commits cite. The source is in
brackets: **spec** is `readme.md`, **user** is an instruction given on
2026-11-20, **brief** is `old/agents/rewrite_brief.md` (section number),
**measured** is a number read from the simulator in this rewrite.

## Terms

Proposed in the design session, not yet confirmed by the user.

- **Tick**: one `env.step` call. One tick runs 5 physics steps. The rubric's
  "sim step" is a physics step (confirmed by the user), so its 3,000 and
  15,000 sim steps are 600 and 3,000 ticks. Every budget below is in ticks.
- **Trial**: one episode, identified by object, maze seed and cloud draw.
- **Attempt**: one try at picking the object within a trial. A trial may hold
  several.
- **Lifted**: the object is at least 10 cm above where it rested.
- **Delivered**: the object's centre is within 10 cm of the goal.
- **Lost**: lifted, then no longer held, before delivery.

## A. Interface and limits

| Id | Requirement | Source |
|---|---|---|
| R1 | `Solver(env)` replaces the course's `KeyBoardController`: `get_action()` returns one `numpy.ndarray` of shape (15,) of absolute joint targets per tick. | spec |
| R2 | The course files are not edited: `maze_utils.py`, `utils.py`, `keyboard_control.py`, `assets/`. In `pick_nav_reach_env.py` only the controller is swapped, as the readme instructs: the import and the two lines of its main block that used `KeyBoardController` now use `Solver`. No object, contact or dynamics parameter is changed. | brief 1, user |
| R3 | Grasp generation, inverse kinematics, motion planning and navigation are our own code. No planning library. | spec, brief 1 |
| R4 | The solver works on a variant of the scene. Frames, sizes and positions (robot spawn, table, wall size, goal height) are read from the observation or the simulator, never written as constants. No rule is keyed on the object's id. | user, brief 1 and 10 |
| R5 | The layout is: `solver.py` with an enum of states; `custom_libraries/` for grasping and navigation; `tools/` for benchmarking; `tests/` for regressions. | user |
| R30 | The solver never writes to the state of the env's simulator: no `restoreState`, no resetting of joints or bodies. It acts only through the action it returns. Read-only queries are allowed. | user |
| R31 | Planning runs in a second simulator that the solver owns: a copy of the scene in its own physics client, set to the observed state before each planning call. | user |
| R6 | The rewrite is typed fresh. The code under `old/` is a reference for approaches and measured values; nothing is copied from it. | user |

## B. Pick (64% of the grade)

| Id | Requirement | Source |
|---|---|---|
| R7 | Grasp candidates are generated from the observed object point cloud and surface normals, as antipodal pairs. | spec |
| R8 | Inverse kinematics and a collision-free motion plan execute the chosen grasp. | spec |
| R9 | The object is lifted by tick 600, which is the rubric's 3,000 sim steps. A first attempt is budgeted to fit inside that. | spec, user |
| R10 | The lift is at least 10 cm and stays at least 10 cm through the hold. For an object picked from the floor, it also ends at least 10 cm above the tabletop. | spec, brief 8 |
| R11 | The object is held for at least 50 ticks after the lift. | spec |
| R12 | After the hold, the gripper's pose relative to the object is close to the planned grasp. The error is reported in millimetres and degrees; the rubric's tolerance is 10 cm. | spec |
| R13 | Every one of the 15 provided objects is picked and delivered. None is recorded as ungraspable. Where no antipodal pair fits the gripper, another kind of hold is chosen from the object's geometry. | user |
| R32 | R13 is met when each object is delivered in at least 4 of its 5 trials. | user |

## C. Navigation and reach (36% of the grade)

| Id | Requirement | Source |
|---|---|---|
| R14 | The route through the maze is planned from `cube_positions`. | spec |
| R15 | The object is delivered: its centre comes within 10 cm of the goal. | spec |
| R16 | Delivery happens by tick 3,000, which is the rubric's 15,000 sim steps. | spec, user |
| R17 | The object is not dropped between the lift and the delivery. | spec |

## D. Failure handling

| Id | Requirement | Source |
|---|---|---|
| R18 | No exception leaves `get_action`. | brief 7 and 10 |
| R19 | Every state has a tick budget and a declared failure exit. Nothing waits without a limit. | brief 7 and 10 |
| R20 | A failure leads to a retry while ticks remain: another candidate, another parking pose, or a new pick of a dropped object. | user, brief 10 |
| R21 | Each attempt starts from a fresh context, so it cannot inherit state from the one before. | brief 7 and 10 |
| R22 | The hold is verified after the fingers close and before the lift. A lost object is detected during the carry. | brief 10 |
| R23 | The terminal states hold a latched pose. The env's controller has no integral term, so holding the measured pose sags. | brief 2b |

## E. Determinism and measurement

| Id | Requirement | Source |
|---|---|---|
| R24 | All randomness comes from one generator seeded from the config. With the same cloud and seed, the solver makes the same decisions. The cloud is used as observed, without re-drawing a subset per pass. | brief 7 |
| R25 | The config is one immutable object. | brief 10 |
| R26 | Every planning call has a CPU-time deadline. | brief 2a |
| R27 | The scorer measures outcomes from the simulator, reads only `solver.state` and `solver.report`, records the settings, counts a crash as a failed trial and caps a trial at 3,000 ticks. | brief 8 and 10 |
| R28 | Versions are compared on the same seeded cloud draws, against the reference `4fafaea`. | brief 8, user |
| R29 | Library code has unit tests that need no simulator. Each plan segment has a simulator test. Simulator runs go one at a time under the memory cap. | user, brief 9 |

## Facts the requirements rest on

- **Budgets** (brief 2): the reference solver lifts at tick 250 to 590 and
  delivers at tick 1,215 to 2,261.
- **Goal height** (measured): the goal is 0.9 m above the floor and success is
  a 3D distance, so an object pushed along the floor cannot be delivered. R13
  therefore means every object has to be lifted.
- **Master chef can** (measured): rests upright at the table centre, top at
  0.770 m, 1.054 kg. Its collision shape is 103.0 mm at its narrowest (brief
  3); the pads open to a 102.85 mm gap. Each finger's collision box is
  63 × 24 × 12 mm.
- **Plate** (measured): rests flat on the table, 260 mm across, 27 mm tall,
  0.058 kg.
- **Large marker** (measured): ends wedged in the tabletop (its box spans
  0.529 to 0.660 m; the tabletop's upper face is at 0.623 m), 0.027 kg, and
  was still moving at tick 599.
- **Second simulator** (measured): a planning client with the robot, table,
  maze and object adds 286 MB to the live env's 986 MB, loads in 1.2 s, checks
  one robot pose against table, object and maze in 0.26 ms, and steps at 657
  ticks per second. The live object's pose is untouched by it.
- **Cloud seeding** (commit `4523897`): the point cloud can be seeded from
  outside the env by wrapping `trimesh.sample.sample_surface`, so R2 and R28
  do not conflict.

## Open decisions

1. **Primary measure.** Proposed: mean rubric points per trial over all 15
   objects, deliveries second. Not confirmed.
2. **How the can is held.** Settled by experiment: a tight fit, the fully
   open hand pushed over it (see `GRASP.md`).
3. **The object in the planning simulator.** A copy of the live object's
   collision shape, or a shape built from the cloud.
4. **Rehearsal.** Whether the planning simulator is only queried for
   clearances, or also stepped to try a grasp before it is executed.
5. **Which 5 trials** per object R32 counts.
6. **Submission date.**
