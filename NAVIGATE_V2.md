# NAVIGATE_V2.md: the carry rebuilt on top of the untouched pick

Started 2026-11-21. `NAVIGATE.md` is the record of the first carry (commit
`f09377d`); its numbers were measured on a grasp that had regressed and in a
shorter maze, and are not carried over. This file is the record from the
point where the pick was restored.

Contents: 1 the task; 2 test conditions; 3 the code; 4 findings; 5 problems
faced; 6 hypotheses tested; 7 measurements; 8 fixes (change log); 9 open.

**Evidence level throughout:** cloud seed 2, the maze `run.sh` builds, one
trial per object and setting, unless a line says otherwise. One trial is not
a rate, and section 4 (finding 9) shows how little it takes to change one.

## 1. The task, as the user set it (2026-11-21)

Three tasks and no others:

1. Get all 15 grasps to a carry pose that does not drop the object. The plate
   and the large marker count: they are grasped, only later than the others.
2. Then give the solver its retry.
3. The back-off is not hard-coded.

Constraints:

- **The grasp library is frozen.** Nothing under `custom_libraries/grasping/`
  is edited. The pick's functions in `solver.py` (`_settle` to `_drag`) are
  not edited either.
- The solver navigates again, because the whole trial has to be run.
- Results are quoted with their cloud seed, maze and number of trials.

Where each stands:

| Task | State |
|---|---|
| 1. Non-dropping carry pose for all 15 | **Not reached: 11 of 15 delivered on cloud seed 2 and 12 of 15 on seed 3** (5 and 5 in the first two runs on seed 2). Never delivered on either seed: the bleach and the plate. A level pose on a route the planner had checked was delivered on the first carry 13 times of 14 over runs 2 and 3 on seed 2 |
| 2. Retry | In the solver (V2-5, V2-13) and seen to work: four objects were dropped, picked up again and delivered (can and hammer on seed 2, sugar box and apple on seed 3), three of them from the floor and raised to the goal's height |
| 3. Back-off from the scene | In the solver (V2-1, V2-8). Chosen per trial: no retreat, or 0.15 to 0.6 m in one of eight directions. First carries on seed 2 take none or 0.15 m straight back; one re-pick in a corridor took 0.15 m at 45 degrees to one side |

## 2. Test conditions

| | |
|---|---|
| Maze | the one `run.sh` builds (`run_batch.py`, layout seed 42): 12 rows by 16 columns of 1 m cells, a room 6 cells wide on the left, walls 2 m high |
| Goal | x = 17.5 m, 0.9 m above the floor; success is the object's centre within 10 cm of it |
| Route | **27.8 m** from the table to the goal (the env's own maze, used for all of `NAVIGATE.md`, gave 21 m) |
| Ticks | 3,000. The pick ends between tick 240 and 875, so 2,100 to 2,750 are left |
| Speed that follows | the object has to be led at **14 to 16 mm per tick** on a first carry (9 to 12 in `NAVIGATE.md`), and 20 to 30 after a re-pick |
| Cloud | seeded per run. Seed 2 is used so far because the pick lifts all 15 on it (seed 1 does not lift the plate or the marker) |

The base's speed jumps at every tick (`NAVIGATE.md`, section 1): the env's
position control covers 30 % of the gap to the target in each of the 5
physics steps of a tick. At 15 mm per tick the base goes from about 0.3 m/s
at the end of one tick to 1.3 m/s at the start of the next, a change of
about 24 g over one physics step. The env's force limit (2,000,000 N) does
not soften this. Every joint but the fingers has the same gain, so the arm
cannot be moved against the base to cancel it. The jump is in proportion to
the speed, so the longer maze is harder on every hold.

## 3. The code

| File | What it holds |
|---|---|
| `custom_libraries/navigation/transport.py` (new) | everything planned after the pick: `choose_back_off`, `travel_poses` (the rule), `plan_travel_pose`, `plan_carry`, `reach_step`, `plan_reach`, and `in_hand` (the object presses on a finger or the palm and is within 25 cm of the hand) |
| `custom_libraries/navigation/travel_pose.py` | the poses and `fold_path`, the move into a pose; new: `height`, the object's height in the pose |
| `custom_libraries/navigation/carry.py` | the route and `CarryFollower`; new: `CarrySettings.turn_step`, `CarryPlan.trail`, the limits on the base's step and heading, `CarryFollower.led` |
| `solver.py` | `_back_off`, `_navigate`, `_reach`, `_retry` call the above; new settings `back_off_*`, `travel_*`, `carry_*`, `reach_*`, `lost_ticks`; `navigate_ticks` 2,000 to 2,900 |
| `tools/travel_poses.py` | the survey; new: `--turn-mm`, `--poses`, `--trace`, columns `pads_lost_m` and `touched`, loss judged by `in_hand`, routes chosen as the solver chooses them |

Nothing under `custom_libraries/grasping/` is changed. Check that the pick
is unchanged: in `runs/deliver_12` every one of the 15 lift ticks equals
that of `runs/pick_cycles/seed2` (the pick alone, commit `0b1b621`, same
seed and maze). The 105 fast tests pass.

## 4. Findings

1. **The route under `run.sh` is 27.8 m, not 21 m.** All earlier carry work
   assumed 21 m. The object must be led a third faster than anything
   `NAVIGATE.md` measured, and every hold is nearer its limit.
2. **Most of the first full run's losses were the carry's own motion, not
   the hold.** Four faults in `navigation/carry.py`, all in code `f09377d`
   also ran: the heading jumped 9 degrees in one tick as the drive began;
   the base's target jumped 230 mm; the route began with a hook; the base
   moved up to 40 mm in a tick while the object was led 15. With the first
   three fixed the bleach went from lost at 0.0 m to 8.9 m and the sugar box
   from 0.0 to 4.4 m, hand down.
3. **What then loses the heavy objects is creep, not a lack of grip.** The
   pads press with about 200 N each from the lift until the tick the object
   leaves. Meanwhile it slides: the sugar box a steady 5 mm per metre on a
   straight at 15 mm per tick (2.4 at 11), the bleach 3.5 (1.1 at 11). The
   pads are 58 mm long.
4. **Which way the object creeps is set by the pose.** Hand down: along the
   fingers toward their tips, which is the way out. Hand level: along the
   fingers into the palm, where it stops (bleach 15 mm to 2.6 mm in 2 m),
   and then down across the pads. One finger under: into the palm, and
   sideways along the lower finger after a bend.
5. **Each pose blocks two directions and leaves the third to friction.**
   Only the hand pointing down can keep the pads' line along the travel
   through a bend, because only then does the wrist's roll turn about the
   vertical. After a bend a level hand points along the arm while the object
   travels along the route, so the jolts are sideways to it until the base
   has come round.
6. **No pose carries every object at 15 mm per tick.** Whole route, survey
   3: folded with the hand kept 7 of 15, hand down 7 of 14, hand level 6 of
   11, finger under 6 of 12.
7. **Thirteen objects have at least one pose that carried them the whole
   route** in at least one trial. The bleach (best 18.4 m) has none; the
   plate was carried the whole route twice in survey 2 and not in survey 3
   (best 21.1 m).
8. **Folded at 0.40 m from the base is out of reach** for nearly every
   object. The object rides 0.70 m out, so the base and object together are
   about 1 m long: the turn on the spot needs open floor.
9. **Outcomes are reproducible and very sensitive.** The marker's hand
   points straight down, so "folded" and "hand down" differ only by
   straightening a tilt of a degree or two. It was lost at 5.3 m folded and
   carried 27.9 m hand down, the same in survey 2 and survey 3. A table of
   single trials does not separate poses that are close.
10. **The master chef can is lost by the turn on the spot at 11 mm per
    tick in every pose, and kept at 6.** Folded with its level hand kept,
    it was delivered at tick 2,456: its first delivery in any run.
11. **The final reach by a straight move of the arm fails often.** In full
    run 1 three carries arrived and were not delivered (banana 10.2 cm
    short, marker 65.7 cm, the apple's second carry). Moving the base and
    torso instead delivered the banana.
12. **A re-pick after a drop has not yet delivered.** Full run 1: 12
    carries started after a re-pick, none delivered. The object must then be
    led at 20 to 30 mm per tick, and the pick itself fails on the floor
    ("no grasp candidates" 7 times for the bowl, "no parking pose" 4 times
    for the plate).
13. **An object picked from the floor cannot always be raised to the goal's
    height in the travel pose.** For the marker the straight way up from 15
    cm to 90 cm leaves the arm's reach.
14. **A level pose on a route the planner had checked was delivered every
    time it was used: 5 of 5 in full run 2** (sugar box, potted meat can,
    banana, bowl, hammer; one trial each). In the same run the four objects
    carried hand down or folded were all dropped. The sugar box and the
    hammer had never been delivered before.
15. **The level poses are wide.** With the object 0.70 m out, the narrowest
    arm the planner finds for "finger under" has its elbow 8 cm off the
    centre line, against 1 cm hand down. The route enters the maze's
    doorway at an angle, and there the elbow is 9 mm inside the door jamb.
    Six of fifteen carries were refused for it in full run 2. In survey 3,
    which does not check collisions, the arm touched a wall in 10 of 23
    level carries. With the object 0.60 m out the same pose passes the
    whole route (soup can); at 0.50 m the arm cannot take it.
16. **The retry carries a dropped object to the goal.** The strawberry and
    the apple were each dropped, picked up from the floor and led to the
    goal on the second carry (9.9 m at 30 mm per tick, 22.7 m at 22). Both
    then stopped 55 cm under it.
17. **Many "out of reach" reports from the travel-pose planner were not the
    arm's reach.** Two causes: one allowance of work was shared by all the
    candidate end poses, so one that failed late starved the rest ("out of
    reach at step 1"); and the pick's arm solver stops when a joint is on
    its limit, so from an arm stretched down to the floor it could not turn
    the hand at all. Entries marked "out of reach" in surveys 1 to 3
    include both.
18. **The master chef can is on the edge.** Delivered in full run 1
    (folded, 0.3 m back-off); carried the whole route twice in the surveys;
    dropped 0.13 m into the drive in full run 2 (folded, 0.15 m back-off,
    at the goal's height); dropped while taking the folded pose in full
    run 3, then delivered after a re-pick. Its turn was at 6 mm per tick
    each time.
19. **With the pose chosen together with a clear route and the object 0.60
    m out, 11 of 15 were delivered** (full run 3), and nothing delivered
    in runs 1 or 2 was lost. Nine first carries were in a level pose; eight
    of them delivered.
20. **The retry delivers.** Full run 3: the can (dropped, re-picked from
    the table) and the hammer (dropped twice, re-picked from the floor)
    were both delivered, led at 20.8 and 24.2 mm per tick with a finger
    under the object.
21. **A floor-picked object can be taken to the goal's height in the travel
    pose** once the planner's own arm iteration is used (V2-17): the
    hammer, finger under. The raise at the goal (V2-15) has still not been
    exercised.
22. **With the hand down, the wrist's roll before the drive drops things.**
    Strawberry and hammer in full run 3, hammer twice in surveys 2 and 3:
    all lost in "roll along" or "roll across" at 0.0 m.
23. **The result holds on a second cloud seed: 12 of 15 on seed 3.** The
    bleach and the plate were delivered on neither seed. The retry
    delivered two more objects there (sugar box, apple).
24. **Finger under holds at speeds the other poses do not.** It carried the
    whole route at 20.8, 24.2, 30 and 30 mm per tick (can, hammer, sugar
    box, apple), all after a re-pick. So the first carry could be led
    faster in that pose, which would leave time for a second.

## 5. Problems faced

| # | Problem | How it showed | What was done | State |
|---|---|---|---|---|
| P1 | The route is a third longer than assumed | `plan_carry` reported 27.84 m and 14 to 16 mm per tick in the first full run | Surveys from then on drive the whole route at 15 mm per tick | Stands: it is the test condition |
| P2 | Survey 1 was too short and too slow to predict a full carry | It held 13 or 14 of 15 in three poses over 2.5 m at 11 mm per tick; the full run then delivered 5 | Survey 2 and 3: the whole 27 m at 15 | Fixed |
| P3 | Survey 2 was confounded | It started from the new, shorter back-off, so the sugar box and bleach were lost at 0.0 m in every pose where survey 1 had held them | Traced tick by tick; led to P6 to P9 | Fixed; survey 3 repeats it |
| P4 | "Lost" meant two different things | Survey 1 used the pick's test (off a pad); the can, jammed on one pad and the palm with 470 N on the fingers, counted as lost | Surveys and solver now use `in_hand`; `pads_lost_m` keeps the stricter record | Fixed |
| P5 | The survey's turn on the spot is not checked for collisions | Banana, hand level, touched the table in the turn (38 N) and was lost | None in the survey; the solver checks its own turn | Open in the tool |
| P6 | The heading jumped as the drive began | Trace: 13.9 to 23.2 degrees in one tick; object thrown 11 cm; bleach gone three ticks later | V2-6 | Fixed |
| P7 | The base's target jumped | Trace: 230 mm in one tick at 0.4 m; the carry then failed with "base fell behind its route" | V2-7 | Fixed |
| P8 | The route began with a hook | Route printed: 0.25 m south, round, then north-east. The base stood inside the margin the route keeps from the table | V2-8 | Fixed for the back-off; a re-pick near a wall can still start inside the margin |
| P9 | The base outran the object | Trace: base steps of 21 to 40 mm where the object was led 15 | V2-9 | In; not measured on its own |
| P10 | Heavy objects creep out of the hand | Traces of the bleach and sugar box (finding 3) | V2-10 lays a heavy object's hand level | Partly: sugar box carried the whole route level (one trial); bleach best 18.4 m |
| P11 | No pose holds everything | Surveys 2 and 3 | V2-10, a rule by hand direction and mass | Open (section 9) |
| P12 | The final reach failed | Banana, marker, apple in full run 1: "goal out of reach" | V2-11, V2-12 | Banana fixed; marker open |
| P13 | A failed reach was followed by "no clear route" | Banana: the re-plan found the 0.6 m left shorter than the arm and gave up three times | V2-12: within 25 cm of the goal goes straight to the reach | Fixed |
| P14 | The marker is picked late and from the floor | Lift at tick 811, carry from 874: it must be led at 20 mm per tick, 75 cm below the goal | None yet | Open |
| P15 | Re-picks fail in the pick | "no grasp candidates", "no parking pose", "object moved during the drive", "no grasp plan" | None: the pick is frozen | Recorded only |
| P16 | Changing the back-off changes everything after it | The pick's end tick, the route's start and the turn all differ between surveys 1, 2 and 3 | Each survey is read with the back-off it started from | Stands |
| P17 | The pose rule took poses the route could not pass | Full run 2: six objects "no clear route to the goal", three of them delivered in run 1 | V2-14: the pose is the first whose whole route is clear | Fixed (soup can, mustard, foam brick delivered again) |
| P18 | The level poses are too wide for the maze's doorway | Refusals name `elbow_flex_link`, 9 mm inside the jamb | V2-18: more arm seeds (elbow above or below the line); V2-19: object 0.60 m out | Passes for the soup can; all 15 in full run 3 |
| P19 | A floor-picked object arrives 55 cm under the goal | Strawberry and apple, second carries; marker | V2-15: a raise at the goal; V2-16: ticks kept back for it | Not yet seen to work |
| P20 | The travel-pose planner reported "out of reach" for want of work | "step 1 of 42: out of reach" for every pose after the first that failed | V2-17: an allowance per end pose; "out of work" reported as such | Fixed |
| P21 | The pick's arm solver stops at a joint limit, and is frozen | The raise could not turn the hand by one step from the floor-pick pose | V2-17: the planner's own iteration (`travel_pose.solve_arm`) leaves a stuck joint out | In; the pick's solver untouched |
| P22 | A carry that fails with the object in the hand is planned again three times the same way, then the solver stops | Full run 2: trials ended at tick 400 to 750 with 2,300 ticks unused | V2-14 removes the common cause | The retry itself still repeats the same plan |

## 6. Hypotheses tested

| # | Hypothesis | Test | Result |
|---|---|---|---|
| H1 | One travel pose will carry all 15 | Surveys 2 and 3, whole route | **Refuted.** Best pose 7 of 15 |
| H2 | The hand stood straight down, wrist steering, is the best pose | Survey 1 said so (13 of 14 over 2.5 m, least slip). Surveys 2 and 3 over the whole route | **Holds only for short, light objects.** 7 of 14 over the whole route; the heavy ones creep out along the fingers |
| H3 | The can is lost because of its pose | Survey 1, then the can alone at 4, 7, 11 mm per tick and with the turn at 6 | **Refuted.** Lost in the turn in every pose at 11; held at 4 and 7; with the turn at 6 it reaches the drive |
| H4 | Standing the can's level hand down will help it | `runs/travel_can_*`, three speeds | **Refuted.** It comes off the pads within 0.7 m and jams against the palm, turned 45 to 55 degrees |
| H5 | The sugar box and bleach are lost at the start because they touch the table after the shorter back-off | Contact record added to the survey (`touched`) | **Refuted.** Neither touched anything but the hand |
| H6 | They are lost because the drive starts too fast | Trace | **Refuted.** Gone within two ticks of the drive's start, before any speed. The heading had jumped (P6) |
| H7 | The heading jump is the whole fault | Fixed it and traced again | **Refuted.** The base's target then jumped 230 mm (P7), on a route with a hook (P8) |
| H8 | "No retreat" is right whenever the planner finds the carry can start where the hand is (V2-1 as first written) | Full run 1 and survey 2 | **Refuted.** The planner's check passed, but the route started with a hook. The back-off must end clear of the route's margin (V2-8) |
| H9 | Heavy objects are lost for lack of grip | Trace of the pads' forces | **Refuted.** About 200 N per pad until the object leaves. `NAVIGATE.md` found the same: a fuller squeeze crept faster |
| H10 | A level hand turns the creep inward, toward the palm | Survey 1 slips, then traces | **Confirmed for the direction of the fingers** (bleach 15 mm to 2.6 mm and stays; sugar box 35 mm inward). **But** the object then sags across the pads (bleach out at 17.8 m) |
| H11 | One finger under the object stops the loss, since the weight no longer hangs on friction | Surveys 2 and 3, trace of the bleach | **Partly.** Hammer, apple, bowl, mustard, banana, potted meat can carried in both surveys. The bleach slid 43 mm sideways in the first bend and was out at 18.4 m; soup can, strawberry and foam brick were lost near 5 m |
| H12 | The same pose gives the same outcome | Marker and strawberry, "folded" against "hand down" with a hand already straight down | **Refuted** (finding 9) |
| H13 | The final reach can be a straight move of the arm | Full run 1 | **Refuted** for 3 of the 8 carries that arrived. Base and torso first (V2-12) |
| H14 | Carrying at the goal's height makes the reach a matter of centimetres | Banana, full trial | **Confirmed where the pose is within reach** (banana delivered 2,469). Refused for the marker from the floor |
| H15 | A dropped object can be picked up and still delivered | Full run 1 | **Not once in 12 carries.** To be measured again on the current code |
| H16 | Mass, grasp width or the lever from the pads to the centre of mass will tell which pose an object needs | Table of survey 2 and 3 outcomes against those values | **Not found.** Only "0.6 kg or more: not hand down" is supported (sugar box, bleach; the mustard bottle at 0.61 kg is carried either way) |
| H17 | The rule of section 7.7 (level poses first) will deliver more than "hand down first" | Full run 2 against full run 1 | **Not shown: 5 of 15 both times.** But it delivered every object it actually carried in a level pose (5 of 5), and lost four of run 1's five by refusing them a route or carrying them otherwise |
| H18 | The level poses fail on this route because of the elbow, not the hand or the object | The refused pose's nearest link printed | **Confirmed**: `elbow_flex_link`, 9 mm inside the door jamb |
| H19 | A narrower arm for the level pose exists if the planner looks for it | More seeds, elbow above and below the line | **Partly.** Narrowest found 8 cm at 0.70 m, still refused. Passes with the object 0.60 m out (13 cm, but a shorter chord across the corner) |
| H20 | "Out of reach" from the planner means the arm cannot get there | Read the solver and the allowance | **Refuted** for many cases (finding 17) |
| H21 | Laying the hand level at the goal will raise a floor-picked object to it | `runs/deliver_try05`, `_try06` | **Not yet.** Refused both times, first for want of work, then by the pick's solver at a joint limit. With V2-17 in, untested |

## 7. Measurements

### 7.1 Survey 1: thirteen poses, 2.5 m of route at 11 mm per tick

`runs/travel_01`. Each object is picked once; the simulator's state after
the back-off is saved; every pose starts from that state, is taken slowly,
then gets the turn on the spot and 2.5 m of the real route. Back-off: the
old fixed 0.6 m. "Lost" is the pick's test (off one of the two pads four
ticks running).

| Pose | Within reach | Held | Lost |
|---|---|---|---|
| as lifted (arm not folded) | 15 | 14 | master chef can |
| folded, hand kept and turned forward, 0.70 m | 15 | 13 | can, strawberry |
| folded, 0.40 m | 2 | 2 | |
| pads' line along the base's heading, 0.70 m, no wrist steering | 15 | 11 | can, strawberry, bleach, hammer |
| the same, 0.40 m | 3 | 2 | can |
| pads' line across the heading, 0.40 m | 7 | 6 | bleach |
| hand stood straight down, wrist keeps the pads' line along the travel | 14 (not the plate) | 13 | can |
| hand straight down, pads' line across the travel | 14 | 11 | can, bleach, hammer |
| hand level | 11 | 9 | can, strawberry |
| hand 45 degrees up | 9 | 7 | can, apple |
| hand level, one finger under the object | 12 | 9 | strawberry, apple, hammer |
| the same, 0.40 m | 0 | | |
| hand 30 degrees up, one finger under | 13 | 11 | can, strawberry |

The mug slipped 56 mm and turned 100 degrees in the hand as lifted, and 7 mm
and 7 degrees stood down. In the level poses the slip along the fingers was
toward the palm: sugar box 24 to 32 mm, bleach 3 to 9 mm.

### 7.2 The master chef can alone

Gripped from the side by a tight fit, hand level, 1.05 kg; its back-off is
driven at 3 mm per tick. `runs/travel_can_*`.

| Turn / drive, mm per tick | folded, hand kept | hand stood down |
|---|---|---|
| 4 / 4 | held 2.5 m | held 2.5 m |
| 7 / 7 | held 2.5 m | held 2.5 m |
| 11 / 11 | lost in the turn | lost in the turn |
| 6 / 9 | lost after 0.19 m | off the pads after 0.29 m |
| 6 / 11 | held 4 m | off the pads after 0.67 m |
| 6 / 14 | held 4 m | off the pads after 0.28 m |

The outcomes at 9, 11 and 14 are not in order of speed: the can is on the
edge of the hold and one trial does not place the edge.

### 7.3 Full run 1: `runs/deliver_12`

All 15, pick to delivery. Code: V2-1 to V2-5 (no retreat if the carry can
start where the hand is; hand stood down if it points down, else folded;
turn at 6 mm per tick; drive at 14.3 to 15.8). The four motion faults were
still in.

**5 of 15 delivered.**

| Object | Result | First carry |
|---|---|---|
| master chef can | **delivered 2,456** | folded, hand kept |
| tomato soup can | **delivered 2,644** | stood down |
| mustard bottle | **delivered 2,664** | stood down, no retreat |
| potted meat can | **delivered 2,587** | stood down |
| foam brick | **delivered 2,652** | stood down |
| banana | 10.2 cm from the goal | carried the whole route; the reach failed; the re-plan found no route for the 0.6 m left |
| large marker | 65.7 cm from the goal | carried the whole route from its lift at tick 811; the reach failed three times |
| sugar box | lost four times | first loss about 285 ticks in: no retreat, then a turn of 171 degrees |
| strawberry | lost twice, third carry running at the end | about 430 ticks in, at the start of the drive |
| bleach cleanser | lost twice | about 380 ticks in, at the start of the drive |
| mug | lost once, second carry 62 cm short at the end | about 665 ticks in, at the start of the drive |
| plate | lost twice | about 600 ticks in, folded with the hand kept |
| apple | lost once; second carry reached the goal at tick 2,932, then "goal out of reach" | about 780 ticks in |
| hammer | lost three times | about 990 ticks in |
| bowl | lost once | about 1,500 ticks in; on the floor afterwards "no grasp candidates" seven times |

The losses "at the start of the drive" are where the heading jump (P6)
strikes. Of 12 carries after a re-pick none delivered; one arrived and
failed its reach, two were running at tick 3,000. What a re-pick ran into:
"object moved during the drive" 3, "no grasp plan" 3, "no parking pose" 4
(all the plate), "no grasp candidates" 7 (all the bowl).

### 7.4 Survey 2: the whole route at 15 mm per tick, faults still in

`runs/travel_02`. Turn at 6 mm per tick, then the whole route at 15. Starts
from the back-off of V2-1. A number is the metres carried before the object
left the hand.

| Object | folded, hand kept | hand down, pads along | hand level | level, finger under | 30 up, finger under |
|---|---|---|---|---|---|
| master chef can | **whole route** | 0.4 | **whole route** | out of reach | out of reach |
| sugar box | 0.0 | 0.0 | 0.0 | 0.0 | out of reach |
| tomato soup can | **whole route** | **whole route** | out of reach | **whole route** | **whole route** |
| mustard bottle | **whole route** | **whole route** | out of reach | **whole route** | out of reach |
| potted meat can | **whole route** | **whole route** | **whole route** | **whole route** | **whole route** |
| banana | **whole route** | **whole route** | 6.5 | **whole route** | out of reach |
| strawberry | 9.1 | lost in the pose | 21.4 | lost in the pose | lost in the pose |
| apple | 12.0 | **whole route** | 17.5 | **whole route** | **whole route** |
| bleach cleanser | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 |
| bowl | **whole route** | 11.8 | **whole route** | **whole route** | 16.5 |
| mug | 4.1 | **whole route** | out of reach | out of reach | out of reach |
| plate | **whole route** | out of reach | 5.0 | **whole route** | 21.8 |
| large marker | 5.3 | **whole route** | out of reach | out of reach | out of reach |
| hammer | lost at the start | 0.2 | 18.5 | **whole route** | out of reach |
| foam brick | **whole route** | **whole route** | **whole route** | **whole route** | **whole route** |
| **Whole route** | **8 of 15** | **8 of 14** | **4 of 11** | **9 of 12** | **4 of 8** |

### 7.5 Traces of the bleach and the sugar box

`tools/travel_poses.py --trace`; `runs/travel_trace01` to `_trace05`.

| Run | Code | What the trace showed |
|---|---|---|
| `trace01`, bleach, hand down | as survey 2 | pose, wrist rolls and turn move it under 2 mm, 200 N per pad. First tick of the drive: heading 13.9 to 23.2 degrees. Gone three ticks later |
| `trace02`, bleach and sugar box | V2-6 | bleach: base target jumps 230 mm at 0.4 m, the carry fails. Sugar box: base step 61 mm at the start, lost at 8.0 m |
| `trace03`, bleach | V2-6 | the route printed: it starts with a hook |
| `trace04`, both, hand down | V2-6, V2-7, V2-8 | bleach lost at 8.9 m, sugar box at 4.4 m, both by sliding out along the fingers |
| `trace05`, bleach and plate, three poses | V2-6 to V2-9 | the table below |

The bleach in three poses (`trace05`):

| Pose | Which way it moves in the hand | Outcome |
|---|---|---|
| folded (hand down) | along the fingers toward their tips: 14 mm to 42 mm | out at 2.8 m |
| hand level | along the fingers into the palm (15 mm to 2.6 mm in the first 2 m) and stays; then down across the pads: 2, 4, 8, 13, 27, 70 mm at 2, 6, 10, 12, 14, 16 m | out at 17.8 m |
| level, finger under | into the palm; sideways along the lower finger, 43 mm in the first bend, then steady near 62 mm for 12 m | out at 18.4 m |

The plate (rim gripped with 12 N per pad): folded with the hand kept it sat
still for 20 m and went at a bend at 21.1 m; level it slid 93 mm across the
pads in the first 2 m and was out at 9.5 m; finger under, out at 12.9 m.

### 7.6 Survey 3: the whole route again, with the faults fixed

`runs/travel_03`. As survey 2, with V2-6 to V2-9 in and the back-offs V2-8
chooses.

| Object | kg | folded, hand kept | hand down, pads along | hand level | level, finger under |
|---|---|---|---|---|---|
| master chef can | 1.05 | **whole route** | 0.5 | **whole route** | out of reach |
| sugar box | 0.64 | 6.5 | 6.5 | **whole route** | lost in the turn |
| tomato soup can | 0.32 | **whole route** | **whole route** | out of reach | 5.3 |
| mustard bottle | 0.61 | **whole route** | **whole route** | out of reach | **whole route** |
| potted meat can | 0.35 | **whole route** | **whole route** | **whole route** | **whole route** |
| banana | 0.17 | **whole route** | **whole route** | lost in the turn (touched the table) | **whole route** |
| strawberry | 0.05 | **whole route** (turned 114 degrees in the hand) | lost in the turn | lost in the pose | 4.8 |
| apple | 0.25 | 9.6 | 11.6 | **whole route** | **whole route** |
| bleach cleanser | 0.97 | 2.8 | 8.4 | 17.8 | 18.4 |
| bowl | 0.03 | 6.7 | 6.6 | **whole route** | **whole route** |
| mug | 0.14 | 4.0 | **whole route** | out of reach | out of reach |
| plate | 0.06 | 21.1 | out of reach | 9.5 | 12.9 |
| large marker | 0.03 | 5.3 | **whole route** | out of reach | out of reach |
| hammer | 0.26 | lost in the first wrist roll | lost in the first wrist roll | 10.9 | **whole route** |
| foam brick | 0.19 | **whole route** | **whole route** | **whole route** | 5.0 |
| **Whole route** | | **7 of 15** | **7 of 14** | **6 of 11** | **6 of 12** |

"30 degrees up, finger under" was within reach for one object from these
back-offs and is dropped.

Carried the whole route in both whole-route surveys:

| Pose | Objects |
|---|---|
| folded, hand kept | can, soup can, mustard, potted meat can, banana, foam brick |
| hand down, pads along | soup can, mustard, potted meat can, banana, mug, marker, foam brick |
| hand level | can, potted meat can, bowl, foam brick |
| level, finger under | mustard, potted meat can, banana, apple, bowl, hammer |

### 7.7 The pose rule taken from surveys 2 and 3 (V2-10)

It uses only what the solver can measure after the lift, not which object
it is:

1. A hand that does not point mostly down (the can, the plate): keep it,
   turn it forward, fold the arm.
2. A hand pointing down holding 0.6 kg or more: level first, then finger
   under, then hand down.
3. Any other hand pointing down: finger under first, then hand down.

Counting each object's two whole-route trials under this rule gives about 11
of 15, against about 9 for "hand down first" and 12.5 for the best pose per
object. Those counts differ by less than the scatter between the two
surveys, and the surveys tested each pose at the height the object was
lifted to, not at the goal's height.

### 7.8 Single full trials after the fixes

| Object | Run | Result |
|---|---|---|
| foam brick | `runs/deliver_try01` (V2-1 to V2-5) | delivered 2,652 |
| banana | `runs/deliver_try02` (V2-1 to V2-13) | **delivered 2,469**, finger under at the goal's height, 0.15 m back-off. Was 10.2 cm short in full run 1 |
| large marker | `runs/deliver_try02` | lost in the drive after 22.95 m, at tick 2,485, 4.9 m from the goal. Hand down, not at the goal's height, led at 20.3 mm per tick |

### 7.9 Full run 2: `runs/deliver_13`

V2-1 to V2-13 on all 15, same seed and maze as full run 1. All 15 lift
ticks equal those of full run 1.

**5 of 15 delivered, a different five.**

| Object | Run 1 | Run 2 | Back-off | Pose in run 2 | What happened in run 2 |
|---|---|---|---|---|---|
| master chef can | 2,456 | lost | 0.15 m | folded, at the goal's height | dropped 0.13 m into the drive (tick 744); six re-picks failed in the pick |
| sugar box | lost | **2,465** | none | level, at the goal's height | first carry. Its first delivery in any run |
| tomato soup can | 2,644 | not carried | 0.15 m | finger under | every route refused: the arm 0 to 11 mm from a wall near the maze's entrance |
| mustard bottle | 2,664 | not carried | none | level | every route refused |
| potted meat can | 2,587 | **2,451** | 0.15 m | finger under | first carry |
| banana | 10 cm short | **2,469** | 0.15 m | finger under | first carry |
| strawberry | lost | not carried | 0.15 m | finger under | every route refused |
| apple | lost | not carried | 0.15 m | finger under | every route refused |
| bleach cleanser | lost | not carried | 0.15 m | level | every route refused |
| bowl | lost | **2,471** | 0.15 m | finger under | first carry |
| mug | lost | lost | none | hand down, at the goal's height | dropped after 19.9 m (tick 2,231) |
| plate | lost | lost | none | folded, hand kept | dropped after 11.7 m (tick 1,449) |
| large marker | 66 cm short | lost | none | hand down, low | dropped after 22.95 m (tick 2,485), led at 20.3 mm per tick |
| hammer | lost | **2,472** | 0.15 m | finger under | first carry. Run 1 lost it three times |
| foam brick | 2,652 | not carried | 0.15 m | finger under | every route refused |

Sorted by what was done with the object:

| | Objects | Delivered |
|---|---|---|
| carried in a level pose (level, or finger under) on a route the planner had checked | 5 | **5** |
| carried hand down or folded | 4 | 0 |
| refused a route: the level pose's elbow stands 17 to 24 cm to the side | 6 | not carried |

- **A level pose on a checked route was delivered every time it was used.**
  Five trials, one each; survey 3's level carries were not checked for
  collisions and the arm touched a wall in 10 of 23 of them.
- **Four objects that run 1 delivered were lost** (can, soup can, mustard,
  foam brick). Three were never carried. By the rule that nothing delivered
  may be lost, run 2's code is not kept as it stands (V2-14 follows).
- "No clear route" ended the trial after three identical plans. The pose is
  now chosen together with the route (V2-14).

### 7.10 Single full trials after full run 2

`runs/deliver_try04` (V2-14 in): the pose is the first whose whole route is
clear.

| Object | Result | Pose | Note |
|---|---|---|---|
| tomato soup can | **delivered 2,642** | hand down, at the goal's height | finger under refused: arm 18 mm into a wall at the maze's entrance |
| mustard bottle | **delivered 2,622** | hand down, at the goal's height | level and finger under both refused |
| foam brick | **delivered 2,620** | hand down, at the goal's height | finger under refused |
| strawberry | 55 cm under the goal | hand down | dropped after 17.9 m (tick 1,953); picked up from the floor; second carry 9.9 m at 30 mm per tick reached the goal at tick 2,937; could not be raised |
| apple | 55 cm under the goal | hand down, low | dropped after 5.0 m (tick 1,002); picked up; second carry 22.7 m at 22 mm per tick reached the goal at tick 2,861; could not be raised |

- **The retry carried both dropped objects to the goal.** These are the
  first second carries to arrive.
- The strawberry's re-pick was the first back-off not straight away: "no
  retreat", four straight retreats and 0.15 m at 45 degrees one way were
  refused, and 0.15 m at 45 degrees the other way was taken.
- Both then stopped 55 cm under the goal: section 4, finding 13.

### 7.11 Full run 3: `runs/deliver_14`

V2-1 to V2-19 on all 15, same seed and maze as full runs 1 and 2. All 15
lift ticks equal those of full run 1.

**11 of 15 delivered.** Nothing that run 1 or run 2 delivered was lost.

| Object | Run 1 | Run 2 | Run 3 | First carry in run 3 | Then |
|---|---|---|---|---|---|
| master chef can | 2,456 | lost | **2,367** | folded, at the goal's height: dropped while the pose was taken | picked up again from the table (tick 582 to 715); finger under, led at 20.8 mm per tick; delivered |
| sugar box | lost | 2,465 | **2,428** | level, at the goal's height, no retreat | |
| tomato soup can | 2,644 | not carried | **2,458** | finger under, at the goal's height | |
| mustard bottle | 2,664 | not carried | **2,439** | finger under, at the goal's height, no retreat | |
| potted meat can | 2,587 | 2,451 | **2,417** | finger under, at the goal's height | |
| banana | 10 cm short | 2,469 | **2,445** | finger under, at the goal's height | |
| strawberry | lost | not carried | lost | hand down, at the goal's height: dropped in the wrist's roll before the drive (tick 677) | picked up; hand down, dropped after 20.8 m at 21.4 mm per tick (tick 2,347); picked up; dropped taking the finger-under pose |
| apple | lost | not carried | **2,442** | finger under, at the goal's height | first delivery in any run |
| bleach cleanser | lost | not carried | lost | finger under, at the goal's height, led at 18.5 mm per tick: dropped after 5.9 m (tick 1,261) | picked up; dropped taking the hand-down pose; then the pick failed four times |
| bowl | lost | 2,471 | **2,456** | finger under, at the goal's height | |
| mug | lost | lost | **2,552** | finger under, at the goal's height, no retreat | first delivery in any run |
| plate | lost | lost | lost | folded, hand kept, led at 17 mm per tick: dropped after 5.5 m (tick 990) | the pick failed six times |
| large marker | 66 cm short | lost | lost | hand down, at the goal's height, 0.6 m back-off, led at 22.3 mm per tick: dropped after 10.1 m (tick 1,761) | the pick failed three times |
| hammer | lost | 2,472 | **2,482** | hand down, at the goal's height: dropped in the wrist's roll before the drive (tick 503) | picked up from the floor; dropped taking the finger-under pose; picked up again; finger under at the goal's height, led at 24.2 mm per tick; delivered |
| foam brick | 2,652 | not carried | **2,610** | hand down, at the goal's height | |

Sorted by the pose of the first carry:

| First carry | Objects | Delivered on it |
|---|---|---|
| level or finger under, on a checked route | 9 | **8** (not the bleach) |
| hand down | 4 | 1 (foam brick; not the strawberry, marker, hammer) |
| folded, hand kept | 2 | 0 (can, plate) |

- **Level poses on a checked route: 8 of 9 on the first carry here, 13 of
  14 over runs 2 and 3.** The one loss is the bleach, 0.97 kg, led at 18.5
  mm per tick because its pick ended at tick 625. The two later carries
  that delivered (can, hammer) were finger under as well; two other
  objects were dropped while that pose was being taken after a re-pick
  (hammer once, strawberry once).
- **The retry delivered two objects** (can, hammer). The hammer is the
  first object picked from the floor to be raised to the goal's height and
  delivered.
- **The wrist's roll with the hand down drops things.** Strawberry and
  hammer were both lost in "roll along", the roll that lines the pads up
  with the route before the drive, with the hand at the goal's height.
- Speeds of 20 to 24 mm per tick were carried the whole route twice (can,
  hammer), both with a finger under the object.
- Not delivered: strawberry, bleach, plate, marker.

### 7.12 The same code on cloud seed 3: `runs/deliver_15_seed3`

V2-1 to V2-19, all 15, `run.sh`'s maze, one trial each. All 15 lift ticks
equal those of the pick alone on seed 3 (`runs/pick_cycles/seed3`).

**12 of 15 delivered.**

| Object | Seed 2 | Seed 3 | Carries on seed 3 |
|---|---|---|---|
| master chef can | 2,367 | lost | hand down at the goal's height (on this seed its hand points down), 15.1 mm per tick: dropped after 0.5 m; the pick then failed |
| sugar box | 2,428 | **2,637** | hand down: dropped after 2.7 m. Picked up; dropped taking the level pose. Picked up; finger under, 30 mm per tick: delivered |
| tomato soup can | 2,458 | **2,613** | hand down |
| mustard bottle | 2,439 | **2,601** | hand down |
| potted meat can | 2,417 | **2,430** | finger under |
| banana | 2,445 | **2,615** | hand down |
| strawberry | lost | **2,438** | finger under. Its first delivery in any run |
| apple | 2,442 | **2,709** | hand down: dropped after 12.5 m. Picked up from the floor; finger under, 30 mm per tick: delivered |
| bleach cleanser | lost | lost | level, 17.9 mm per tick: dropped after 15.4 m. Picked up; finger under, 30 mm per tick: dropped after 1.7 m. Picked up; level, still being carried at tick 3,000, 8.6 m from the goal |
| bowl | 2,456 | **2,598** | hand down |
| mug | 2,552 | **2,593** | picked from the floor, 0.6 m back-off, arm as lifted |
| plate | lost | lost | 0.6 m back-off (the fallback), no pose within reach, "no clear route to the goal": never carried |
| large marker | lost | **2,393** | picked from the floor, finger under at the goal's height, 19.3 mm per tick. Its first delivery under these conditions |
| hammer | 2,482 | **2,475** | finger under |
| foam brick | 2,610 | **2,430** | finger under |

- **Two seeds, 11 and 12 of 15.** Not delivered on either: the bleach and
  the plate. On one of the two: strawberry and marker (seed 2), can (seed 3).
- **The retry delivered two more objects** (sugar box on its third pick,
  apple on its second), both finger under at 30 mm per tick, the fastest
  the carry allows.
- **Finger under has now carried the whole route at 20 to 30 mm per tick
  four times** (can and hammer on seed 2, sugar box and apple on seed 3),
  and lost the bleach at 30 after 1.7 m.
- On seed 3 the level poses were refused or out of reach more often: seven
  first carries were hand down, and four of those delivered.
- **Hand down lost the heavy ones again**: can after 0.5 m, sugar box after
  2.7 m, apple after 12.5 m.

## 8. Fixes (change log)

| # | Change | Fixed | Broke or left failing |
|---|---|---|---|
| V2-1 | Back-off chosen from the scene (`transport.choose_back_off`): no retreat if the travel pose can be taken and the carry started where the hand is; else the shortest clear retreat; else the old fixed retreat. | The back-off is no longer hard-coded. | As first written it let the base stand inside the route's margin: a hook at the start of the route (P8). Tightened by V2-8. |
| V2-2 | Travel pose from survey 1: hand stood straight down where it already points mostly down, else folded with the hand kept. | Master chef can delivered for the first time (full run 1). | Heavy objects creep out, hand down. Replaced by V2-10. |
| V2-3 | The turn on the spot has its own speed limit, 6 mm per tick (`CarrySettings.turn_step`). | The can gets through the turn. | Costs about 80 ticks on a quarter turn. |
| V2-4 | A drop is judged by `in_hand`, not by both pads. | An object jammed on one pad and the palm is carried on. | Not shown to change a full run's outcome. |
| V2-5 | After a drop the hand opens where it is and the way to the ready pose is planned from there (it used to retrace the grasp's approach from wherever the base stood). | | Not measured on its own. |
| V2-6 | The turn on the spot ends on the heading the drive begins with (`carry.make_plan`); the heading is never allowed to jump (`CarryFollower`). | Bleach no longer thrown out on the first tick of the drive. | |
| V2-7 | The base's place on the route is the first point, going forward from where it was, within one arm of the object: it only moves forward and never jumps (`CarryPlan.trail`). | Bleach 0.4 m to 8.9 m, sugar box 0.0 to 4.4 m (hand down). | |
| V2-8 | The back-off must end where a route can begin: clear of the walls and the table by the margin the route keeps. Lengths 0.15, 0.3, 0.45, 0.6 m, straight away first, then the other directions. Floor maps on which the base stands clear are used first. | No hook at the start of the route. | Back-offs seen so far: 0.15 m (banana). Not counted over all 15 yet. |
| V2-9 | The base may move at most 1.2 times the object's speed per tick; the object is slowed where it would. | | Not measured on its own. |
| V2-10 | Pose rule from surveys 2 and 3 (section 7.7). | To be read off full run 2. | |
| V2-11 | The travel pose brings the object to the goal's height where that is within reach (`fold_path(height=...)`); otherwise it keeps its height. | Banana carried at the goal's height. | Refused for the marker picked from the floor. |
| V2-12 | The final approach moves the base and the torso with the arm still (`transport.reach_step`); the arm's straight move is only for what they cannot do. An object within 25 cm of the goal on the floor plan goes straight to the reach. | Banana delivered at tick 2,469 on its first carry. | |
| V2-13 | A drop in the carry records its tick, phase and the distance led (`plan_stats["lost"]`). | | |
| V2-14 | The pose is the first in the rule's order that is within reach, has a clear way in, and in which the whole carry is clear (`transport.plan_travel`). | Soup can, mustard and foam brick delivered again (2,642, 2,622, 2,620), hand down after the level poses were refused. | Strawberry and apple, given hand down the same way, were dropped at 17.9 m and 5.0 m. |
| V2-15 | At the goal, an object more than 5 cm under it is raised by laying the hand level, standing still (`transport.plan_raise`); the base and torso then close what is left. | | Refused in both trials so far (H21). |
| V2-16 | The carry of an object that will arrive under the goal keeps 200 more ticks back; REACH may take 600 ticks. | | The marker is then led at 23 mm per tick instead of 20, and was dropped at 20.1 m instead of 22.95 m. |
| V2-17 | The travel-pose planner: an allowance of work per end pose; its own arm iteration that leaves out a joint on its limit (`travel_pose.solve_arm`); "out of work" reported apart from "out of reach". | | Changes which poses are found for every object. |
| V2-18 | More seeds for the folded arm: shoulder at four heights, elbow above and below the line, wrist at four angles (80 seeds, was 18). The end poses' spreads are recorded. | Narrowest "finger under" arm 8 cm at 0.70 m. | Still refused at the doorway at 0.70 m. |
| V2-19 | The object rides 0.60 m from the base's centre, not 0.70 (`travel_reach`). | "Finger under" at the goal's height passes the whole route for the soup can. | None of the surveys was run at 0.60 m. |

## 9. Open

- **No pose is drop-free for all 15 at the speed this maze forces** (task
  1). What could change that, none of it tried:
  - Re-seating during the carry. The solver can measure where the object
    sits in the hand at every tick. A level hand rolled half a turn reverses
    which way the object sags across the pads.
  - Rehearsing the candidate poses in a physics copy before choosing one.
  - A pose per object. The grader uses these 15, so it is open to us, but
    it is a table, not a mechanism, and finding 9 says single trials would
    fill it badly.
- **The four not delivered in full run 3:**
  - **Bleach** (0.97 kg): its pick ends at tick 625 after two failed
    attempts, so it is led at 18.5 mm per tick; finger under, dropped after
    5.9 m. Best in any trial 18.4 m of 27.
  - **Plate**: 12 N per pad on the rim, hand nearly level, so the rule
    keeps the hand and folds; dropped after 5.5 m at 17 mm per tick. It was
    carried the whole route in survey 2 (folded, and finger under).
  - **Marker**: lifted at tick 811 from the floor, led at 22 mm per tick
    hand down; dropped after 10.1 m. Finger under is out of the arm's reach
    for it.
  - **Strawberry** (48 g): hand down, dropped in the wrist's roll; hand
    down again after a re-pick, dropped after 20.8 m.
  The first three are led faster than 17 mm per tick because their picks
  end late, and the pick is frozen.
- **The wrist's roll with the hand down** (finding 22): slower, or left out
  where the pads' line is already near the route.
- **Dropped while a pose is taken after a re-pick** (hammer, strawberry,
  bleach in full run 3).
- **Speed.** The first carry arrives 340 to 540 ticks before the end, so it
  could be led about 10 % slower. At 11 mm per tick the sugar box still
  creeps 2.4 mm per metre hand down, which is 65 mm over the route, so
  speed alone does not save it. Not tried for the level poses.
- **Retry** on the current code (task 2): not measured.
- **The back-off** in directions other than straight away, and after a
  re-pick in a corridor (task 3): not yet exercised.
- **Seeds other than 2 and 3**, and the unseeded cloud `run.sh` uses. The
  pass bar is per object over five draws; two draws have been run.
- **The plate on seed 3** stopped at tick 619 with the plate in the hand
  and 2,380 ticks unused: the folded pose was refused ("the arm would
  jump" at step 10 of 24 of turning the hand in place), the arm as lifted
  was refused a route (the wrist 5 and 11 mm from a wall where 12 are asked
  for), and the same plan was tried three times (P22).
- **Leading the first carry faster where a finger is under the object**
  (finding 24). Not tried.
- V2-4, V2-5 and V2-9 have not been measured on their own.
