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
| 1. Non-dropping carry pose for all 15 | **Not reached. Over 100 layouts (benchmark 2, section 7.26): 80.5 % of 1,500 trials delivered and 69.1 % with no drop (76.1 % and 62.6 % with V2-19 on the same trials); all 15 delivered on 4 seeds of 100, the middle seed 12.** On layout 42: 13 of 15 delivered on cloud seed 2 and 13 of 15 on seed 3 with V2-20 to V2-29 (section 7.22; 11 and 12 with V2-19), 11 of 15 on each with no drop. Not delivered: strawberry and bleach on seed 2, can and plate on seed 3; each is delivered on the other seed. On four other layouts with the unseeded cloud, V2-19 delivered 11, 12, 12 and 12 (section 7.13); V2-20 on has not been run there |
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
25. **The result holds on other layouts and under the unseeded cloud: 11,
    12, 12 and 12 of 15** on layouts 42, 17, 67 and 21 (section 7.13, the
    user's runs on the Xeon machine, one trial each). 47 of 60 delivered,
    41 of 60 with no drop after the lift.
26. **Over those four runs two objects account for 8 of the 13 failures:
    the master chef can (0 of 4, dropped on its first carry) and the large
    marker (0 of 4, never lifted).** The bleach and the plate were each
    delivered twice, the hammer three times, the other ten every time.
27. **The marker's failure there is in the pick**, on its first attempt
    ("object moved before the close"), under every one of the four clouds.

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

### 7.13 Four full runs under `run_batch.py` on the Xeon machine: four layouts, unseeded cloud

Run by the user on 2026-11-21 between 09:11 and 11:21 on commit `1b8c126`
(the code of `dd0ab31`; the user named the commit, the runs' files do not
record it). `run_batch.py`
with `--maze-seed` and `--seed` both set to 42, 17, 67 and 21, the cloud
unseeded (as in the graded runs), `--max-steps 5000`, all 15 objects, one
trial each. Read from each run's `summary.jsonl` only (kept with
`config.json`, `maze.json` and `note.md` in `runs/xeon/NAME/`); the videos
and logs were not opened, so poses, speeds and distances led are not known
for these runs.

| Run | Layout and env seed | Delivered | Delivered with no drop after the lift |
|---|---|---|---|
| `20261121_091134_95cf3843` | 42 | 11 of 15 | 8 |
| `20261121_093324_f67a097d` | 17 | 12 of 15 | 11 |
| `20261121_095259_55d0a0d5` | 67 | 12 of 15 | 11 |
| `20261121_112123_2d988cf7` | 21 | 12 of 15 | 11 |
| **All four** | | **47 of 60** | **41 of 60** |

Every delivery was before tick 3,000 (the latest at 2,911); the trials not
delivered ran on to 5,000. A number is the tick of delivery.

| Object | 42 | 17 | 67 | 21 | Delivered | No drop |
|---|---|---|---|---|---|---|
| master chef can | lost | lost | lost | lost | **0 of 4** | 0 |
| sugar box | 2,439 | 2,415 | 2,441 | 2,298 | 4 | 4 |
| tomato soup can | 2,582 | 2,420 | 2,476 | 2,515 | 4 | 4 |
| mustard bottle | 2,444 | 2,526 | 2,450 | 2,519 | 4 | 4 |
| potted meat can | 2,549 | 2,441 | 2,393 | 2,422 | 4 | 4 |
| banana | 2,606 | 2,420 | 2,460 | 2,435 | 4 | 4 |
| strawberry | 2,562 | 2,435 | 2,493 | 2,807 (two drops) | 4 | 3 |
| apple | 2,448 | 2,496 | 2,493 | 2,528 | 4 | 4 |
| bleach cleanser | lost | 2,532 | 2,488 | lost | **2 of 4** | 2 |
| bowl | 2,892 (one drop) | 2,475 | 2,419 | 2,227 | 4 | 3 |
| mug | 2,747 (one drop) | 2,552 (dropped taking the pose) | 2,422 | 2,596 | 4 | 2 |
| plate | lost | 2,286 | lost | 2,267 | **2 of 4** | 2 |
| large marker | never lifted | never lifted | never lifted | never lifted | **0 of 4** | 0 |
| hammer | 2,399 (one drop) | lost | 2,911 (three drops) | 2,451 | 3 of 4 | 1 |
| foam brick | 2,597 | 2,441 | 2,425 | 2,432 | 4 | 4 |

What the 13 trials not delivered record:

| Object | Layout | Lifted at | Failures in order |
|---|---|---|---|
| master chef can | 42 | 220 | no parking pose; lost in the carry; "object moved during the drive" four times; "no grasp plan" twice |
| master chef can | 17 | 266 | lost taking the travel pose; "no grasp plan" six times; a second carry from tick 1,434, lost; ended 6.3 m from the goal |
| master chef can | 67 | 221 | lost in the carry; "no grasp plan" five times; lost taking the travel pose; "object moved during the drive" |
| master chef can | 21 | 208 | lost in the carry; "no grasp plan" seven times |
| bleach cleanser | 42 | 520 | lost in the carry; lost taking the travel pose; the pick failed four times around them |
| bleach cleanser | 21 | 491 | lost in the carry; "no grasp candidates" five times |
| plate | 42 | 311 | lost taking the travel pose; "object lost during the lift" seven times in all |
| plate | 67 | 372 | "dragged to the edge" before the lift; lost in the carry; then "arm did not reach the end of the approach" or "of the transit" six times |
| large marker | 42, 17, 67, 21 | never | "object moved before the close" on the first attempt every time; then "no parking pose" (42 once, 67 and 21 seven times each), "no grasp candidates" (42 and 17, six times each) or "object moved during the drive" (17 once) until the solver stopped. Where the marker lay is not in the summary |
| hammer | 17 | 172 | lost in the carry three times, then in the hold; "episode budget spent"; ended 0.42 m from the goal |

- **The carry does not rest on layout 42.** Three other layouts gave 12 of
  15 each, and no trial records a route refused.
- **Ten objects were delivered in all four runs.** Seven of them were never
  dropped: sugar box, soup can, mustard, potted meat can, banana, apple,
  foam brick.
- **The can was delivered in none.** It is lifted by tick 266 every time and
  lost on its first carry or while the pose is taken; the pick then finds
  no plan for it where it lies.
- **The marker was lifted in none.** This is the pick, not the carry: the
  first attempt ends "object moved before the close" under all four clouds,
  and the marker is not picked up from where it ends. It was lifted on
  cloud seeds 2 and 3 and is not on seed 1.
- **The bleach and the plate were delivered for the first time**, twice
  each, on the first carry with no drop.
- **The hammer was dropped on its first carry in three runs of four.**
- **The first attempts repeat across the four runs**: the mustard bottle
  ("object moved during the drive", lifted 348 to 377), the bleach (the
  same, then "no grasp plan", lifted 491 to 520) and the mug (the same,
  lifted 307 to 315) fail and recover the same way every time. So the scene
  on the table barely differs between these runs; they are four clouds and
  four layouts, not four scenes.
- **Late for the pick's 600 ticks**: the strawberry on layout 42 (lifted at
  648, after two losses in the lift), and the marker.

### 7.14 The master chef can on cloud seeds 1, 4 and 5

`runs/can_first_s1` (900 ticks), `runs/can_full_s4`, `runs/can_full_s5`:
the can alone, `run.sh`'s maze (layout 42), commit `1b8c126`, one trial
each. With sections 7.11 and 7.12 this gives the can's first carry on five
cloud seeds.

| Cloud seed | Grasp | First carry | Result |
|---|---|---|---|
| 1 | from the side | lost in the 0.15 m back-off, at 3 mm per tick, 13 ticks in | every later pick "object moved during the drive" |
| 2 | from the side | folded, at the goal's height: lost while the pose was taken | re-picked from above, finger under: delivered 2,367 |
| 3 | from above, tilted | hand down, 15.1 mm per tick: lost after 0.5 m | not delivered |
| 4 | from the side | folded, at the goal's height, 15.6 mm per tick | **delivered 2,465 on the first carry** |
| 5 | from the side | folded, at the goal's height, 14.9 mm per tick: lost after 21.0 m (tick 2,012) | re-picked from the floor by tick 2,357, then "no clear route to the goal" three times; stopped at tick 2,364 with the can in the hand 4.8 m from the goal |

- **First carry delivered on 1 seed of 5**; with the Xeon runs, 1 of 9.
- **On seed 5 the retry gave up with the can in the hand and 636 ticks
  left** (P22 again). Every plan was refused at its first pose: the held
  can is 0 mm from a fixed body (`gaps_mm: load: 0`), and the folded pose
  was refused for the same reason at step 1 ("load -1 mm"). What the can
  touches there has not been traced.

### 7.15 How the master chef can leaves a level hand: traces on cloud seed 5

`tools/trace_trial.py` (new: a line per tick of the whole trial, with where
the object sits in the hand, the forces on it, and what else it touches).
The can alone, layout 42, cloud seed 5, side grasp, folded with the hand
kept (level, pointing forward, the pads either side of the can), 14.9 mm
per tick. One trial per row.

| Run | Setting | What the trace shows |
|---|---|---|
| `runs/trace_can_s5.log` | as committed | The pads press 175 to 190 N each from the lift to the loss. In the first 4.5 m the can slides 9 mm into the palm and stays. It then sinks across the pads only while the base's heading is changing: 17 to 50 mm through the two bends at the maze's doorway (ticks 912 to 1,070, heading 0.80 to -0.98 rad), nothing on the straights, 50 to 52 mm through the next four bends with the palm pressing about 48 N, then 53 to 75 mm from 16.5 m on, after the can had moved 20 mm out of the palm. Lost at 21.0 m (tick 2,012). In the turn on the spot before the drive (1.7 rad, the can going round at 6 mm per tick) it sank 2 mm |
| `runs/trace_can_s5_yaw6.log` | `carry_max_yaw=0.006` (the object is slowed in bends until the base turns at most 0.006 rad per tick; 0.05 as committed) | The can sinks more slowly through the doorway (14 to 26 mm) but goes on sinking, also on a straight (40 to 47 mm between 9.6 and 10.7 m), and settles at 50 to 55 mm from 11 m. Still in the hand at tick 2,969 after 19.0 m: far too slow to arrive |
| `runs/trace_can_s5_repick.log` | as committed, from tick 2,300 | After the re-pick from the floor the can presses 11 N on a wall at a height of 0.20 m (at 13.00, -3.62). That is the "load 0 mm" for which every pose and route was refused |

- **The can is not lost for want of grip, nor on the straights.** It sinks
  across the pads in bends, and holds for hundreds of ticks once it has
  sunk about 50 mm and turned about 40 degrees in the hand.
- **Slowing the bends did not stop it** (one trial), and costs more ticks
  than the route leaves: 47 % of this route's drive ticks are spent with
  the heading changing.

### 7.16 Finger under for the can's side grasp

`runs/can_under_s2`, `_s4`, `_s5`: the can alone, layout 42, the pose order
set to finger under, then folded (`travel_order`), one trial each.

| Cloud seed | Finger under | Result |
|---|---|---|
| 2 | taken (not at the goal's height) | **lost in the turn on the spot** at tick 553, before the drive; not delivered |
| 4 | refused: carry blocked at the doorway on all three routes (base 3 mm, elbow 8 and 9 mm) | folded, delivered 2,465, the same as without the setting |
| 5 | refused: elbow -3, -1 and 11 mm at the doorway | folded, lost at 21.0 m, the same as without the setting |

Turning a side grasp so that one finger is under the can lays the can's
axis across the hand, and in the turn on the spot the hand moves along that
axis. Not pursued.

### 7.17 Where the level poses are refused a route

`runs/blocked_s3`, `runs/blocked_s2`: the pick and the choice of pose
only, with the planner's record extended to every route it tried. Layout
42. The base stands near (-0.7, 0) for the pick; the door from the room into
the maze is at (2.5, 4.5), a metre wide, and the route bends left into it
and right out of it.

| Object, cloud seed | Pose refused | Route 1 | Route 2 | Route 3 |
|---|---|---|---|---|
| sugar box, 3 | level | elbow 8 mm at (1.50, 4.02) | elbow -3 mm at (1.52, 4.15) | not recorded |
| sugar box, 3 | finger under | base 9 mm at (3.04, 4.29) | 12 mm at (4.55, 0.25) | not recorded |
| mustard bottle, 3 | level | upper arm 9 mm at (1.49, 4.12) | 2 mm at (1.52, 4.17) | not recorded |
| mustard bottle, 3 | finger under | upper arm 9 mm at (1.64, 4.25) | 8 mm at (1.68, 4.31) | not recorded |
| banana, 3 | finger under | base 3 mm at (3.07, 4.28) | 3 mm at (4.55, 0.22) | not recorded |
| hammer, 2 | finger under | base 3 mm at (3.07, 4.28) | 2 mm at (6.73, -2.29) | not recorded |
| can, 4 | finger under | base 3 mm at (3.07, 4.28) | elbow 9 mm at (4.55, 0.31) | elbow 8 mm at (4.55, 0.40) |
| can, 5 | finger under | elbow -3 mm at (1.49, 4.01) | elbow -1 mm at (1.53, 4.16) | elbow 11 mm at (1.52, 4.18) |

Every refusal is at the doorway or at one of the next two corners, by a
gap of -3 to 12 mm where 12 are asked for. Only the narrowest arm found for
the pose had been tried.

### 7.18 Full runs 5 and 6, and the re-run after them: V2-20 to V2-26

All 15, layout 42, one trial each: `runs/deliver_16_s2` (cloud seed 2) and
`runs/deliver_17_s3` (seed 3) with V2-20 to V2-25; then the objects whose
pose had changed, again, with V2-26 on top (`runs/deliver_18_s2`,
`runs/deliver_19_s3`). Compared with full run 3 (`runs/deliver_14`) and the
seed 3 run (`runs/deliver_15_seed3`). **All 30 lift ticks are those of the
pick alone.** Objects not listed have the same pose and delivery tick in
all three.

Cloud seed 2: **11, then 13, then 12 of 15.**

| Object | Before (V2-19) | V2-20 to V2-25 | With V2-26 |
|---|---|---|---|
| master chef can | folded, lost taking the pose; re-pick, finger under: 2,367 | the same first carry; re-pick, level by a way made joint by joint: 2,519 | the same: 2,541 |
| mustard bottle | finger under: 2,439 | level, second arm, joint by joint: 2,433 | the same: 2,466 |
| strawberry | hand down, lost three times | finger under, joint by joint: lost at 21.1 m; re-pick, finger under: **2,728** | the same first carry lost at 16.8 m; the second carry still running at tick 3,000 |
| bleach cleanser | finger under, lost at 5.9 m | level, joint by joint: lost taking the pose; four more carries lost | level, second arm, joint by joint: lost at 5.9 m; two more lost taking the pose |
| mug | finger under: 2,552 | finger under by a narrower arm, joint by joint: lost at 0.0 m; hand down: 2,754 | finger under as before: 2,552 |
| plate | folded, lost at 5.5 m | the same | the same |
| large marker | hand down, lost at 10.1 m | finger under, second arm: **2,479** | the same: 2,479 |

Cloud seed 3: **12, 12 and 12 of 15**, the same twelve.

| Object | Before (V2-19) | V2-20 to V2-25 | With V2-26 |
|---|---|---|---|
| master chef can | hand down, lost at 0.5 m | the same (no other way into a level pose was found either) | not run again |
| sugar box | hand down, lost at 2.7 m; third pick: 2,637 | level, second arm, joint by joint: lost taking the pose; third pick: 2,910 | level, third arm, joint by joint: lost at 0.0 m; second pick: 2,583 |
| mustard bottle | hand down: 2,601 | finger under, second arm, joint by joint: 2,435 | the same: 2,465 |
| bleach cleanser | level, lost at 15.4 m; two more carries | the same first carry; two more lost taking the pose | the same first carry; second lost at 1.5 m; third running at tick 3,000 |
| mug | arm as lifted: 2,593 | folded, joint by joint: 2,538 | the same: 2,555 |
| plate | stopped at tick 619 with the plate in the hand | folded, joint by joint: lost taking the pose within 6 ticks | the same |
| large marker | finger under: 2,393 | finger under, the move before the turn: 2,418 | finger under as before: 2,393 |

- **Nothing delivered before was lost**, on either seed, in either version.
- **A second arm for the same hand** (V2-23) gave the marker its first
  delivery on seed 2, and put the mustard bottle and the sugar box in a
  level pose where they had been carried hand down.
- **A pose reached joint by joint is a poor start for a carry.** Over both
  versions and seeds: carried to the goal 8 times, lost 13 times (7 while
  the pose was taken, 2 at 0.0 m, 4 later). Slowing the hand to 6 mm per
  tick (V2-26) did not change that: 4 carried, 6 lost. The first carries
  it lost (strawberry, bleach, sugar box, plate) were lost or never
  started before the change too, in the pose they had then; the one it
  took from a delivered first carry, the mug on seed 2, V2-26 puts back.
- **Still never delivered here: the bleach and the plate.**

### 7.19 The weakest holds led no faster than the ticks require

`runs/haste10_s2`, `_s3`, `_s5`: `carry_haste=1.0` (1.25 as committed),
layout 42, V2-20 to V2-26, one trial each.

| Object, cloud seed | At 1.25 | At 1.0 |
|---|---|---|
| plate, 2 | folded, 17.0 mm per tick: lost at 5.5 m | 13.6 mm per tick: **delivered 2,677**, first carry |
| bleach cleanser, 3 | level, 17.9 mm per tick: lost at 15.4 m | 14.3 mm per tick: **delivered 2,830**, first carry |
| bleach cleanser, 2 | level, lost at 5.9 m | 15.5 mm per tick: lost at 16.9 m |
| master chef can, 3 | hand down, 15.1 mm per tick: lost at 0.5 m | 12.1 mm per tick: lost at 0.5 m |
| master chef can, 5 | folded, 14.9 mm per tick: lost at 21.0 m | 11.9 mm per tick: lost at 22.6 m |

The plate and the bleach had never been delivered here. The speed limit the
solver itself works out from the pads' force, the friction and the mass
(`_carry_step`, used for the back-off) ranks the 15 objects, and the three
never delivered here are its three lowest: can 3.0 mm per tick, bleach 3.2,
plate 3.3 to 3.9; then sugar box 4.8, mustard bottle 5.0, hammer 7.0, the
rest 7.3 to 20 (`runs/deliver_16_s2`, `_17_s3`).

### 7.20 Re-seating: half a turn of the wrist for an object that has sunk

`runs/reseat_can_s5.log` (`runs/tmp/reseat_probe.py`): the can on cloud seed
5, the solver stopped at tick 1,150 with the can 51 mm below the hand's
axis; base brought to rest, the wrist rolled half a turn at 0.04 rad per
tick. **The can did not move in the hand** (19, 2, 51 mm throughout) and
ended 51 mm above the axis, the pads pressing 250 and 210 N.

`runs/trace_can_s5_reseat.log`: the whole trial with V2-27. **Delivered at
tick 2,611 on the first carry, no drop**, where it had been lost at 21.0 m.
Two re-seats, at 6.2 m (tick 1,015, 36 mm down) and at 20.8 m (tick 2,085,
35 mm down); between them the can sank 73 mm through the hand in 14.5 m.

### 7.21 Re-seating and the slower pace on the three weakest holds

Layout 42, one trial per cell. "Pace 1.25" is `carry_haste` as committed;
"pace 1.0" is the weak-hold rule (V2-28). A number after "lost" is the
metres led.

| Object, cloud seed | Before (V2-26), pace 1.25 | Re-seat, pace 1.25 (`runs/reseat_A_*`) | Re-seat, pace 1.0 (`runs/reseat_B_*`) | As it now stands (`runs/fragile_*`, `runs/pace_s5`, `runs/trace_bleach_s3.log`) |
|---|---|---|---|---|
| can, 2 | lost taking the folded pose; re-pick: 2,541 | the same: 2,541 | | the same first carry; re-pick, finger under: 2,694 |
| can, 3 | hand down, lost 0.5 | the same | | not run (folded before hand down, `travel_order`: lost 11.8) |
| can, 4 | folded: 2,465 | 2,465, no re-seat | 2,904, no re-seat | 2,755, no re-seat (with a tick reserve since dropped) |
| can, 5 | folded, lost 21.0 | **2,611**, re-seats at 6.2 and 20.8 m | **2,977**, one re-seat at 12.1 m | **2,856**, one re-seat at 12.1 m, the pace set again after it |
| bleach, 2 | level, joint by joint: lost 5.9 | re-seat at 4.8 m, lost 5.7 | re-seat at 14.5 m, lost 17.3 | finger under (no way joint by joint), 14.8 mm per tick: lost 19.6 |
| bleach, 3 | level, lost 15.4 | re-seat at 11.7 m, lost 12.8 | **2,830**, no re-seat | **2,830**, no re-seat (15.5 mm per tick with a tick reserve: re-seat at 21.6 m, lost 24.0) |
| plate, 2 | folded, lost 5.5 | the same | **2,677** | **2,677** |
| plate, 3 | folded, joint by joint: lost taking the pose | the same | the same | folded by the move then the turn, 16.5 mm per tick: lost 2.5 |
| sugar box, 2; mustard, 2; mug, 3 | 2,428; 2,466; 2,555 | the same ticks, no re-seat | | |

- **Re-seating delivers the can where it is held from the side and
  folded**: seed 5 at every pace tried (14.9, 12.7 and 11.9 mm per tick),
  and it is not called on where the can does not sink (seed 4). That is
  one cloud seed.
- **Re-seating does not save the bleach**: lost 1 to 3 m after each of four
  half turns, about where it was lost without them. What delivers the
  bleach on seed 3 is the pace: 14.3 mm per tick delivered, 15.5 and 17.9
  did not.
- **The slower pace delivers the plate on seed 2** (13.6 mm per tick; 17.0
  lost it).
- A trial repeated on the same seed and settings gave the same ticks every
  time here, so the repeats in the table are not further evidence.
- **Still lost on the first carry:** the can on seeds 1 (in the back-off),
  2 (taking the folded pose) and 3 (grasped from above, hand down); the
  bleach on seed 2; the plate on seed 3.

### 7.22 Full runs 7 and 8: V2-20 to V2-29 on all 15

`runs/deliver_20_s2` (cloud seed 2) and `runs/deliver_21_s3` (seed 3),
layout 42, one trial each, against full run 3 (`runs/deliver_14`) and the
seed 3 run (`runs/deliver_15_seed3`), which were V2-19. **All 30 lift ticks
are those of the pick alone, and nothing delivered before is lost.**

| | Seed 2, before | Seed 2, now | Seed 3, before | Seed 3, now |
|---|---|---|---|---|
| Delivered | 11 | **13** | 12 | **13** |
| Delivered with no drop after the lift | 9 | 11 | 10 | 11 |

| Object | Seed 2 | Seed 3 |
|---|---|---|
| master chef can | 2,694 (lost taking the folded pose; re-pick, finger under) | **lost**: hand down, 0.5 m, then the pick fails |
| sugar box | 2,428 | 2,674 (level by a third arm, joint by joint: lost in the turn; re-pick, level, 22 mm per tick, one re-seat at 18.4 m) |
| tomato soup can | 2,458 | 2,613 |
| mustard bottle | 2,466 | 2,465 |
| potted meat can | 2,417 | 2,430 |
| banana | 2,445 | 2,615 |
| strawberry | **lost**: finger under, joint by joint, 16.8 m; two more carries | 2,438 |
| apple | 2,442 | 2,709 (hand down, lost at 12.5 m; re-pick, finger under) |
| bleach cleanser | **lost**: finger under, 14.8 mm per tick, 19.6 m | **2,830** (new) |
| bowl | 2,456 | 2,598 |
| mug | 2,552 | 2,555 |
| plate | **2,677** (new) | **lost**: folded, 16.5 mm per tick, 2.5 m |
| large marker | **2,479** (new) | 2,393 |
| hammer | 2,482 (hand down, lost in the roll; third pick, finger under) | 2,475 |
| foam brick | 2,610 | 2,430 |

- **13 and 13 of 15, from 11 and 12.** Every one of the 15 objects is
  delivered on at least one of the two seeds; eleven are delivered on both.
- **Not delivered: strawberry and bleach on seed 2, can and plate on
  seed 3.** Each of the four is delivered on the other seed.
- First carries that still drop and are then delivered: can and hammer on
  seed 2, sugar box and apple on seed 3.

### 7.23 Benchmark 1 on the Xeon: 1,500 trials of the carry before V2-20

`runs/bench_01` (`tools/bench.py`, commit `376e759`, which is V2-19 for the
solver): 15 objects on seeds 1000 to 1099. The seed is the layout's, the
env's and the cloud's, so every trial has its own maze. 3,000 ticks, one
trial each. Logs are kept only for trials not delivered or with a drop.

| Object | Lifted | Lifted by tick 600 | Delivered | Delivered, no drop | Mean points of 40 |
|---|---|---|---|---|---|
| master chef can | 78 | 74 | 30 | 29 | 23.2 |
| sugar box | 100 | 100 | 84 | 53 | 36.2 |
| tomato soup can | 100 | 100 | 94 | 91 | 38.9 |
| mustard bottle | 100 | 99 | 98 | 94 | 39.5 |
| potted meat can | 100 | 100 | 100 | 100 | 40.0 |
| banana | 100 | 100 | 96 | 89 | 39.1 |
| strawberry | 100 | 100 | 77 | 36 | 33.8 |
| apple | 100 | 100 | 84 | 75 | 37.1 |
| bleach cleanser | 100 | 96 | 53 | 49 | 32.1 |
| bowl | 100 | 100 | 87 | 78 | 37.8 |
| mug | 100 | 99 | 84 | 45 | 35.3 |
| plate | 82 | 78 | 24 | 21 | 23.4 |
| large marker | 66 | 17 | 47 | 40 | 20.3 |
| hammer | 100 | 100 | 84 | 44 | 35.8 |
| foam brick | 100 | 100 | 100 | 95 | 39.8 |
| **all 1,500** | 1,426 | 1,363 | **1,142 (76 %)** | **939 (63 %)** | **34.2** |

One seed of the 100 has all 15 delivered; the middle seed has 12.

Points lost, of 60,000 (my reading of the rubric, section in `tools/bench.py`):

| Item | Points lost | Where |
|---|---|---|
| Within 10 cm of the goal, and in time (6 + 6) | 4,296 | plate 912, can 840, marker 636, bleach 564, strawberry 276, 192 each for sugar box, apple, mug and hammer, bowl 156 |
| No drop after the lift (4) | 2,020 | strawberry 256, can 240, hammer 224, marker 216, mug 212, bleach 200, plate 200, sugar box 184 |
| Lifted by tick 600 (6) | 822 | marker 498, can 156, plate 132 |
| Lifted, held 50 ticks, grasp pose (6 + 6 + 6) | 1,632 | marker 624, can 444, plate 420 |
| **All** | **8,770 (14.6 %)** | marker 1,974, can 1,680, plate 1,664, bleach 788, strawberry 616 |

- **72 % of the points lost are lost after the lift** (6,316 of 8,770).
- **Objects that looked settled on layout 42 are not.** A carry is lost in
  64 trials of 100 for the strawberry, 56 for the hammer, 53 for the mug, 46
  for the sugar box and 25 for the apple.
- **How the 358 undelivered trials ended:** never lifted 74 (marker 34, can
  22, plate 18); dropped and then no pick succeeded before the attempts ran
  out 160; dropped and out of ticks 68, the object in the hand again in 65;
  stopped with no drop 56 (plate 29, can 11, bowl 6, marker 6; mostly "no
  clear route to the goal", the stop that V2-21 and V2-22 are for).
- **The first carry that is lost is mostly a hand-down carry.** Of the first
  carries lost in the drive: sugar box 40 of 43 hand down, hammer 38 of 48,
  bleach 33 of 49, apple 21 of 25, strawberry 27 of 50, bowl 12 of 16,
  banana 10 of 10. The clean trials have no log, so this is not a rate for
  the pose; `tools/bench.py` now records every carry's pose for that.
- **Hand down is used because the level poses were refused, and mostly
  because no way into them was found**, not because the carry was blocked.
  Of the 388 hand-down carries in the logs, 260 have only ways in refused
  for "finger under" and "level", and 128 have a carry blocked for at least
  one of them. The reasons recorded: turning in place out of reach 245, the
  carry blocked 141, no folded pose within reach and clear 136, turning in
  place stopped by the robot's own links 131, folding after the turn out of
  reach 95, the arm would jump 73, the scene 52, the held object 22. V2-23
  and V2-24 (more arms, more ways in) are aimed at the ways in.
- **Where the object is lost in the drive, hand down:** spread over the
  whole route (sugar box: 13 within 0.5 m, 14 after 10 m), at 10.5 to 14 mm
  per tick, which is what a 20 to 25 m route in the ticks left requires.
- **The pick, which is frozen:** the marker is lifted by tick 600 in 17
  trials of 100; "no parking pose" 180 times and "object moved before the
  close" 133 times. In 12 trials it rests at 0.37 m, not on the floor, and
  is never lifted. The can: "no grasp plan" 228 times, and in 9 trials it
  rests off the table (0.05 m) when the pick begins and is never lifted.

### 7.24 The pick and the carry as one task, and what in 7.23 is confounded

All from `runs/bench_01` (1,500 trials, the carry before V2-20).

**What holds, with no confounder I can find:**

- **The ticks leave room for one carry.** The pick ends at tick 292 (median
  start of the carry); the route is 22.0 m (median of the 100 layouts, 17.6
  to 30.1); so the first carry is planned at 12.8 mm per tick (median; 10.8
  to 16.6 for the middle 80 %; trials with a log only) and a clean delivery is at tick 2,459 (2,414
  to 2,569). A carry after a drop has to go at 20 to 30 mm per tick.
- **A drop is recovered in 47 % of trials (203 of 431), and it depends on
  the object:** mug 74 %, hammer 71 %, sugar box 67 %, strawberry 64 %;
  plate 9 %, bleach 8 %, can 3 %. For the last three the pick from the maze
  floor fails ("no grasp plan", "no parking pose").
- **Five objects never lift on their first attempt** (mustard, bleach, mug,
  plate, marker: 0 of 100 each), always after "object moved during the
  drive" or "before the close". It costs them 180 to 520 ticks against the
  objects that do (median lift 163 to 229).
- **The layout is not a shared cause.** The number delivered per seed has
  variance 1.90, against 1.83 if the 15 objects failed independently of each
  other; trials with a drop, 2.51 against 2.44. No pair of the six objects
  that drop most drops on the same seeds more than chance (for example sugar
  box and hammer: both on 30 seeds, 25.8 expected).
- **Route length matters a little:** a drop in 27 %, 29 % and 35 % of the
  lifted trials on the shortest, middle and longest third of the layouts;
  delivered 83 %, 82 %, 76 %. Each third holds all 15 objects.
- **The solver's own friction limit does not predict a drop.** The mustard
  bottle (limit 3.7 mm per tick) drops in 6 trials, the strawberry and the
  mug (limit 20) in 64 and 53.
- **Two kinds of loss.** Heavy or weakly held objects go in the drive, at
  any distance (sugar box, hammer, bleach, apple). Small ones go while the
  hand is being moved: of the strawberry's 69 lost first carries, 11 while
  the pose is taken and at least 17 within 0.5 m; of the mug's 57, 20 while
  the pose is taken.

**What is confounded:**

| Claim | Confounder | What is left of it |
|---|---|---|
| A late pick makes a drop likelier (delivered 88 % if lifted before tick 200, 60 % at 450 to 600) | The object: the late picks are the marker, bleach, plate and mug | Nothing. Within one object, the earlier half and the later half drop equally often (sugar box 42 % and 50 %, hammer 58 % and 54 %, mug 62 % and 44 %, bleach 47 % and 53 %) |
| The hand-down pose loses the object (it is the pose of most lost first carries) | Selection: only trials with a failure or a drop have a log, so the hand-down carries that were kept are not counted. And the pose is not given at random: hand down is what is left when the level poses are refused | A lower bound per object, taking every clean trial as hand down: sugar box at least 43 % of hand-down first carries lost, hammer 46 %, strawberry 47 %, bleach 40 %, apple 22 %. The same bound for finger under: strawberry 49 %, mug 41 % |
| Faster carries are kept more often (under 12 mm per tick: 1 kept, 130 lost; at 30: 85 kept, 39 lost) | The same selection (a slow first carry that was kept has no log); and a fast carry is a second carry: another grasp, from the floor, a shorter route, often another pose | Nothing |
| The seed's effect | The seed is the layout, the object's placement and the cloud at once | They cannot be told apart here; together they add almost nothing beyond chance (above) |
| The gain of V2-20 to V2-30 on layout 42 (13 and 13 from 11 and 12) | None in the comparison (paired, repeatable), but ten changes at once on two draws | On layouts 67 and 21 the same code gives 11 and 12 against 11 and 13 (section 7.25) |

`tools/bench.py` now records every carry's pose, way in, speed and loss for
every trial (`carries` in `results.jsonl`, and a table by pose in
`summary.md`), which removes the selection.

### 7.25 Layouts 67 and 21, old against new, and joint by joint switched off

Full runs of all 15, one trial each, cloud seeded with the layout's number
(as `tools/bench.py` does): the new code (V2-20 to V2-30) in
`runs/deliver_new_m67` and `runs/deliver_new_m21`; commit `376e759` (V2-19)
from a copy of its code under `runs/tmp/base_376e759/`, in that folder's
`runs/deliver_old_m67` and `runs/deliver_old_m21`. All 30 lift ticks are
equal between the two versions.

| Draw (layout, cloud seed) | Delivered, old | new | No drop, old | new |
|---|---|---|---|---|
| 42, 2 (section 7.22) | 11 | 13 | 9 | 11 |
| 42, 3 (section 7.22) | 12 | 13 | 10 | 11 |
| 67, 67 | 11 | 11 | 9 | 10 |
| 21, 21 | 13 | **12** | 11 | **10** |
| All 60 trials | 47 | 49 | 39 | 42 |

- **28 of the 30 trials on these two layouts end the same way** in both
  versions (delivered or not, with or without a drop).
- **Layout 67:** the sugar box becomes a clean carry (finger under by a third
  arm; was hand down and lost at 5.7 m). Not delivered in either version:
  can (never lifted), banana (hand down, lost three times), bleach, plate.
- **Layout 21: one trial lost, the bowl.** Old: hand down, delivered at
  2,514. New: finger under reached joint by joint, lost as the drive began.
  The can and the bleach are delivered in both, about 400 ticks later in the
  new version (2,851 and 2,826), which leads weak holds without haste; the
  can with one re-seat at 8.0 m.
- **The banana on layout 67 is refused finger under by a real collision,
  not by the margin.** With `carry_margin` 5 mm in place of 12
  (`runs/probe_m67_banana_margin5`) the upper arm is 35 mm inside the wall
  further along the route. The narrowest finger-under arm for that grasp is
  21 cm wide.

**Joint by joint switched off** (`travel_more_ways=1`), on the ten trials of
the four draws whose pose was reached joint by joint (`runs/ways1_*`), same
code otherwise:

| Trial | With joint by joint | Without |
|---|---|---|
| mustard, 42 seed 2 | level, delivered 2,466 | finger under, delivered 2,439 |
| strawberry, 42 seed 2 | not delivered (lost at 16.8 m) | not delivered (hand down, lost) |
| sugar box, 42 seed 3 | lost in the turn, then delivered 2,674 | hand down, lost at 2.7 m, then delivered 2,637 |
| mustard, 42 seed 3 | delivered 2,465 | hand down, delivered 2,601 |
| mug, 42 seed 3 | folded, delivered 2,555 | as lifted, delivered 2,593 |
| potted meat, 67 | delivered 2,416 | hand down, delivered 2,451 |
| marker, 67 | delivered 2,538 | hand down, delivered 2,593 |
| sugar box, 21 | level, delivered 2,435 | level, low, delivered 2,496 |
| mustard, 21 | delivered 2,443 | hand down, delivered 2,517 |
| **bowl, 21** | **lost at 0.0 m, not delivered** | **hand down, delivered 2,514** |

- One trial better without it, nine with the same outcome, none worse. With
  it off the four draws give 13, 13, 11 and 13 delivered: no draw below the
  old code.
- Ten trials do not decide it. The default stays on, so that benchmark 2
  measures it: its ablation re-runs every trial in which joint by joint
  acted, which gives the exact outcome of the system without it.

### 7.26 Benchmark 2 on the Xeon: V2-20 to V2-31 against benchmark 1, and each part switched off

`runs/bench_02` (`tools/bench.py --ablate`, commit `4e155e7`, no uncommitted
changes, the same machine as 7.23): the same 1,500 trials as benchmark 1, 15
objects on seeds 1000 to 1099, 3,000 ticks, one trial each, the cloud seeded
with the trial's seed. Every trial records every carry. The tables are in
`runs/bench_02/report.md` (`tools/bench_report.py runs/bench_01
runs/bench_02`); its counts were recounted from the rows
(`runs/tmp/b2_check.py`), and the tables below that are not in the report
come from `runs/tmp/b2_attr.py` and `runs/tmp/b2_pose_pairs.py`. The 1,500
trials took 55 core-hours and the 1,666 trials of the ablation 72.

**The three controls pass**, so the ablation's shortcut (a trial in which a
part did not act is not run again) may be quoted:

| Control | Trials | Ran the same (lift tick, delivery tick, last tick, failures) |
|---|---|---|
| A part did not act in the trial; run without it | 40 at random for each of the six parts | 240 of 240 |
| Every part off, against benchmark 1's same trials | 100 at random | 100 of 100 |
| The same settings again | 40 at random | 40 of 40 |

The second says two things: the six switches cover every change since
`376e759`, and the machine repeats a trial a day later. The first is a
sample: none of 40 differing puts the share of such trials that would differ
under 7 % for each part (95 %).

**Benchmark 1 (V2-19) against benchmark 2 (V2-31), the same 1,500 trials:**

| Measure | V2-19 | V2-31 | Difference (95 % interval, seeds resampled) | Trials gained | lost | p (sign test) |
|---|---|---|---|---|---|---|
| Delivered | 1,142 (76.1 %) | 1,208 (80.5 %) | +4.4 points (+3.2 to +5.7) | 99 | 33 | 7.5e-09 |
| Delivered with no drop | 939 (62.6 %) | 1,036 (69.1 %) | +6.5 points (+5.0 to +8.0) | 123 | 26 | 2.8e-16 |
| Mean points of 40 | 34.15 | 34.88 | +0.72 (+0.55 to +0.90) | | | |
| Lifted | 1,426 | 1,426 | 0 | 0 | 0 | |

The first lift is at the same tick in 1,500 of 1,500 trials: the pick is
untouched. Delivered on a seed: 15 of 15 on four seeds (one before), the
middle seed 12 (12 before), the worst 9 (7 before), mean 12.1 (11.4).

| Object | Delivered: V2-19 | V2-31 | gained | lost | No drop: V2-19 | V2-31 | gained | lost |
|---|---|---|---|---|---|---|---|---|
| master chef can | 30 | 38 | 9 | 1 | 29 | 36 | 8 | 1 |
| sugar box | 84 | **98** | 16 | 2 | 53 | **88** | 38 | 3 |
| tomato soup can | 94 | **90** | 1 | 5 | 91 | **85** | 0 | 6 |
| mustard bottle | 98 | 100 | 2 | 0 | 94 | 95 | 1 | 0 |
| potted meat can | 100 | 100 | 0 | 0 | 100 | 100 | 0 | 0 |
| banana | 96 | 99 | 3 | 0 | 89 | 91 | 2 | 0 |
| strawberry | 77 | 81 | 9 | 5 | 36 | 37 | 3 | 2 |
| apple | 84 | 86 | 4 | 2 | 75 | 78 | 6 | 3 |
| bleach cleanser | 53 | **77** | 26 | 2 | 49 | **75** | 26 | 0 |
| bowl | 87 | 87 | 6 | 6 | 78 | 81 | 8 | 5 |
| mug | 84 | 86 | 5 | 3 | 45 | 48 | 6 | 3 |
| plate | 24 | 26 | 4 | 2 | 21 | 24 | 5 | 2 |
| large marker | 47 | 49 | 3 | 1 | 40 | 40 | 1 | 1 |
| hammer | 84 | 91 | 11 | 4 | 44 | **60** | 16 | 0 |
| foam brick | 100 | 100 | 0 | 0 | 95 | 98 | 3 | 0 |

- **Three objects carry the gain:** the bleach (+24 delivered), the sugar
  box (+14, and +35 with no drop) and the hammer (+7, +16 with no drop).
  These, both measures for the first two and no drop for the hammer, are the
  only per-object differences that stand after correcting for 30 tests
  (p under 0.0017). The can's +8 (p 0.021) does not.
- **One object is worse: the tomato soup can**, 94 to 90 delivered and 91 to
  85 with no drop (0 gained, 6 lost, p 0.031; not significant once
  corrected for 15 objects, but the cause is found, below).
- **The bowl is unchanged in sum and not in its parts:** 6 trials gained and
  6 lost.

**Each part switched off**, on the trials of the full run in which it acted
(the effect is over all 1,500 trials):

| Part | Switched off by | Acted in | Delivered: gained by the part | lost by it | p | No drop: gained | lost | p | Effect on delivered, points (95 %) | on no drop | on points of 40 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| More arms (V2-23) | `travel_arms=1` | 142 | 14 | 9 | 0.40 | 18 | 6 | 0.023 | +0.3 (-0.2 to +0.9) | +0.8 (+0.1 to +1.5) | +0.07 (-0.01 to +0.15) |
| Both other ways in (V2-24) | `travel_more_ways=0` | 489 | 60 | 24 | 0.00011 | 88 | 22 | 1.6e-10 | +2.4 (+1.3 to +3.6) | +4.4 (+3.1 to +5.7) | +0.46 (+0.29 to +0.63) |
| Joint by joint alone | `travel_more_ways=1` | 323 | 29 | 16 | 0.072 | 46 | 13 | 1.9e-05 | +0.9 (+0.0 to +1.7) | +2.2 (+1.2 to +3.3) | +0.19 (+0.07 to +0.32) |
| No haste for a weak hold (V2-28) | `carry_fragile=0` | 243 | 23 | 15 | 0.26 | 25 | 13 | 0.073 | +0.5 (-0.2 to +1.3) | +0.8 (+0.1 to +1.5) | +0.10 (-0.02 to +0.21) |
| Re-seating (V2-27) | `reseat_sag=0` | 27 | 1 | 1 | 1 | 1 | 1 | 1 | 0.0 (-0.2 to +0.2) | 0.0 (-0.2 to +0.2) | 0.00 (-0.03 to +0.03) |
| Stall (V2-21, V2-22) | `free_reach=0`, `carry_retry_margins=(1.0,)` | 62 | 11 | 1 | 0.0063 | 9 | 0 | 0.0039 | +0.7 (+0.2 to +1.1) | +0.6 (+0.2 to +1.1) | +0.06 (0.00 to +0.11) |

| System | Delivered, % | No drop, % | Points of 40 |
|---|---|---|---|
| V2-19 (benchmark 1) | 76.1 | 62.6 | 34.15 |
| V2-31 with no other way in | 78.1 | 64.7 | 34.42 |
| V2-31 with the move before the turn, no joint by joint | 79.7 | 66.9 | 34.69 |
| **V2-31, full** | **80.5** | **69.1** | **34.88** |
| full without more arms | 80.2 | 68.3 | 34.81 |
| full without the weak hold's pace | 80.0 | 68.3 | 34.78 |
| full without re-seating | 80.5 | 69.1 | 34.88 |
| full without the stall parts | 79.9 | 68.5 | 34.82 |

- **No part hurts, so no default changes.** Every row without a part is at
  or below the full system on all three measures.
- **Joint by joint stays on.** The ten local trials of 7.25 said one better
  without it and nine the same. On the 323 trials in which it acts it gains
  29 deliveries and loses 16, and gains 46 clean deliveries and loses 13.
  The rule set before the run (switch it off if the system without it is at
  least as good) is not met.
- **The ways in are over half of the gain:** +2.4 of the +4.4 points
  delivered, +4.4 of the +6.5 with no drop. The two rows are nested: the
  move before the turn gives +1.5 delivered and joint by joint +0.9 on top
  of it.
- **What stands after correcting for the twelve tests of the table**
  (Holm): both ways in on both measures, joint by joint on no drop, the
  stall parts on no drop. More arms and the weak hold's pace have intervals
  on no drop that exclude zero and sign tests that do not pass; on delivered
  neither is told from zero.
- **Re-seating does nothing measurable:** it acts in 27 trials (the can 19,
  the sugar box 6) and changes the outcome of two, one each way.
- **Not measured:** two parts off at once. The single effects on delivered
  add up to +3.9 (arms, both ways in, the weak hold's pace, the stall parts)
  against +4.4 for all together, and to +6.6 against +6.5 on no drop.

**Which part acts for which object** (delivered: gained by the part, lost
by it; of the trials in which it acted):

| Object | Both ways in | of which joint by joint | No haste for a weak hold | Stall | More arms |
|---|---|---|---|---|---|
| sugar box | 14, 2 of 65 | 5, 2 of 48 | | 1, 0 of 2 | 0, 2 of 17 |
| bleach | 12, 2 of 40 | never (a weak hold) | 17, 5 of 100 | | 1, 0 of 7 |
| hammer | 10, 3 of 39 | 10, 3 of 33 | 0, 1 of 6 | | 2, 0 of 10 |
| strawberry | 7, 3 of 45 | 1, 3 of 32 | | | 2, 2 of 12 |
| apple | 4, 2 of 44 | 3, 0 of 35 | | | 3, 0 of 13 |
| mug | 3, 0 of 25 | 4, 0 of 22 | | 1, 1 of 6 | 2, 2 of 11 |
| master chef can | 3, 1 of 5 | | **2, 6 of 74** | | 1, 0 of 5 |
| plate | | | 3, 3 of 57 | 3, 0 of 24 | 1, 0 of 3 |
| bowl | **2, 6 of 35** | **2, 5 of 26** | | 5, 0 of 6 | 1, 0 of 6 |
| tomato soup can | **0, 4 of 28** | **0, 2 of 18** | | 1, 0 of 1 | **0, 2 of 12** |

- **The weak hold's pace is the bleach's gain and the can's loss.** All 100
  bleach trials, 74 of the can's and 57 of the plate's have a weak hold. Led
  without haste the bleach gains 17 and loses 5; the can gains 2 and loses
  6. Their clean deliveries arrive at tick 2,863 and 2,862 (median; every
  delivery of the twelve objects with no weak hold, 2,446 to 2,554), which
  leaves no second carry.
- **Of the 33 trials delivered by V2-19 and not by V2-31, 32 are delivered
  again with one part off:** a way in for 21 (joint by joint for 16 of
  them), more arms alone for 4, the weak hold's pace for 6, the stall parts
  for 1. So the losses are these parts' doing and not noise between two
  runs.
- **The soup can's five lost trials** are all finger-under carries that a
  new part made possible (more arms, the move before the turn, or joint by
  joint), lost at 12 to 22 m; with the part off each is carried hand down
  and delivered.

**Which pose loses what** (the handoff's open question, and the claim 7.24
could not settle). The ablation answers it without the selection: with a
way in switched off the same trial takes another pose, so the two poses are
compared on the same trial. First carries, kept (not lost) with the part and
without, for `travel_more_ways=0`:

| Pose with the other ways in | Pose without | Trials | Kept with | Kept without | Kept only with | only without | p |
|---|---|---|---|---|---|---|---|
| level, finger under | hand down | 274 | 219 | 193 | 45 | 19 | 0.0016 |
| level | hand down | 77 | 73 | 40 | 33 | 0 | 2e-10 |
| level | level, finger under | 46 | 44 | 42 | 4 | 2 | 0.69 |
| folded | as lifted | 14 | 7 | 2 | 5 | 0 | 0.06 |

Finger under against hand down, by object (kept with finger under, kept
with hand down, of trials): hammer 20, 4 of 25; sugar box 9, 3 of 13; apple
34, 30 of 40; banana 18, 16 of 21; foam brick 21, 19 of 21; strawberry 10,
9 of 22; potted meat 40, 40 of 40; mustard 9, 9 of 9; mug 6, 7 of 11;
**bowl 19, 21 of 29; soup can 21, 26 of 26**.

- **Hand down is the worse pose for the heavy objects** (level against hand
  down: 33 trials kept only level, none only hand down) and for the hammer.
- **For the strawberry it makes no difference** (10 and 9 of 22).
- **For the soup can hand down is the better pose:** it is never lost hand
  down (26 of 26 here; 31 of 31 over all its hand-down first carries) and
  lost in 15 of its 68 finger-under first carries, at 17.8 m (median).
- These are trials in which the hand could not be led straight into the
  level pose, so the level pose here is one reached by another way.
- First carries of the full run by pose, all ways in: level 182 of 192 kept
  (95 %), finger under 603 of 788 (77 %), hand down 168 of 257 (65 %),
  folded 62 of 136 (46 %), as lifted 11 of 41 (27 %). These are not rates of
  the pose: the rule gives level to the heavy objects and folded to the can
  and the plate.

**What is still lost** (V2-31, of 60,000 points): 7,686 (12.8 %; 8,770
before). After the lift 5,232, which is 68 % (72 % before): the goal 3,504
(plate 888, can 744, marker 612, bleach 276, strawberry 228), no drop 1,728
(strawberry 252, can 240, plate 224, marker 220, mug 200, hammer 160). The
pick 2,454, as before. The marker (1,954), the plate (1,664) and the can
(1,584) lose 68 % of all points lost.

- **How the 292 undelivered trials ended:** never lifted 74; dropped and no
  pick succeeded before the attempts ran out 143 (160 before); dropped and
  out of ticks 43 (68); stopped with no drop 32 (56).
- **A drop is recovered in 48 % of trials (172 of 358; 47 % before)**, and
  fewer trials have one (431 before). By object: mug 38 of 50, strawberry
  44 of 63, hammer 31 of 40, sugar box 10 of 12; can 2 of 38, plate 2 of
  38, bleach 2 of 25.
- **Where the strawberry and the mug are lost on their first carry:**
  strawberry, 62 lost: 14 while the pose is taken, 14 within 0.5 m of the
  drive, 30 further on, 4 with no distance recorded. Mug, 49 lost: 23 while
  the pose is taken, 6 within 0.5 m, 20 further on.

**What this changes:** no default. The pose rule could be changed for the
soup can and the bowl (hand down before finger under), but that is a pose
per object, which the user allows only if proven to reach 15 of 15.

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
| V2-20 | Settings for experiments, defaults unchanged: `travel_order` (a fixed order of poses in place of the rule), `carry_max_yaw` (the base's turn per tick in the drive). The record of a refused carry names every route's blocking link and gaps. `tools/trace_trial.py`, `tools/bench.py`. | | No change of behaviour. |
| V2-21 | A held object that rests against something is first taken straight off it (`transport.load_contact`, `plan_free`; `free_gap`, `free_reach`), at the start of the back-off. | The can on seed 5, pressing 11 N on a wall after its re-pick, starts a second carry (6 ticks) where the trial had stopped. | That carry, at 30 mm per tick, lost the can after 1.4 m. Not seen on any first pick in full runs 5 and 6. |
| V2-22 | A carry refused with the object in the hand is planned again with half the margin, then 15 % of it, and the pose is chosen again (`carry_retry_margins`). It was the same plan three times. | | Not exercised in full runs 5 and 6: the one stop it was written for (plate, seed 3) no longer happens, by V2-25. |
| V2-23 | Up to three arms are tried for each pose, the narrowest first, until the carry passes (`travel_arms`, `fold_path(nth=...)`). | Marker delivered on seed 2 (finger under, second arm; was hand down and lost). Mustard bottle and sugar box in a level pose on seed 3. | |
| V2-24 | Two more ways into a pose when the hand cannot be led there straight: the move before the turn; then joint by joint (`travel_more_ways`). | Mug on seed 3 folded (was carried as lifted). Strawberry delivered on seed 2 in full run 5. | Joint by joint: 8 carried, 13 lost (section 7.18). The plate on seed 3 is lost within 6 ticks of it. |
| V2-25 | (with V2-24) The plate on seed 3 no longer stops with the plate in the hand. | | It is lost instead. |
| V2-26 | The other ways come after every end pose has been tried straight; a way made joint by joint is taken with the hand at 6 mm per tick at most (`travel_hand_step`). | Mug on seed 2 delivered on its first carry again. | Strawberry on seed 2 not delivered again (dropped at 16.8 m instead of 21.1 m; its second carry ran out of ticks). |
| V2-27 | Re-seating (`CarryFollower.reseat`, `transport.sag`; `reseat_sag`, `reseat_room`, `reseat_lever`): during the drive, an object that has sunk 35 mm below the axis of a level hand, across the pads, and at least half of that since the carry began, is re-seated: the drive slows to rest, the wrist rolls half a turn, the drive goes on. Only where the wrist is not steering, the pads hold the object within 3 cm of its centre along the hand, and the turn is clear of everything by 4 cm. | The can on seed 5 delivered on its first carry (was lost at 21.0 m). | Does not save the bleach (section 7.21). Each re-seat costs about 100 ticks. |
| V2-28 | A weak hold gets no haste (`carry_fragile`): where the solver's own speed limit for the hold (`_carry_step`) is under 4.5 mm per tick, the object is led as slowly as the ticks left allow, and no way into a pose is made joint by joint. | Plate delivered on seed 2 and bleach on seed 3, both for the first time here, on the first carry. | Those carries arrive at tick 2,677 to 2,904: no time for a second carry after a drop. |
| V2-30 | Before the back-off falls back to the fixed 0.6 m retreat, every retreat is judged again with the travel pose reached by the move before the turn (`choose_back_off`). | The plate on seed 3 takes no retreat and starts its pose 159 ticks sooner (`runs/backoff2_s3`). | It is still lost (1.9 m, at 15.0 mm per tick). The three other trials that use the fallback (mug on seed 3, marker and the strawberry's re-pick on seed 2) are unchanged, tick for tick (`runs/backoff2_s3_mug`, `runs/backoff2_s2`). Not in full runs 7 and 8. |
| V2-31 | `travel_more_ways` is a level: 0 no other way in, 1 the move before the turn, 2 also joint by joint (default 2, as before). A carry planned again with the object in the hand is recorded (`carry_retries`). `tools/bench.py`: every carry and every part of the carry that acted is recorded per trial; `--ablate` runs each part switched off on the trials in which it acted, a sample of the others as a check, and every part off on a sample. `tools/bench_report.py`: baseline against full, paired, and the ablation. | With joint by joint off the bowl on layout 21 is delivered again (section 7.25). | No change of behaviour at the defaults. Benchmark 2 decided the default for joint by joint: it stays on (section 7.26: 29 deliveries gained, 16 lost, in the 323 trials in which it acts). |
| V2-29 | After a re-seat the pace is set again from the ticks left, to end 150 ticks early, never under the plan's (`CarryFollower.set_pace`, `reseat_slack`). | The can on seed 5 at pace 1.0 arrives at 2,856 (2,977 without). | A tick reserve up front instead (tried first) led the bleach at 15.5 mm per tick on seed 3 and lost it at 24.0 m. |

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
- **Seeds other than 2 and 3**: cloud seeds 1, 4 and 5 have not been run
  with a carry. The unseeded cloud and three other layouts have (section
  7.13): four draws, in which the can and the marker were delivered 0 times
  of 4, the bleach and the plate 2, the hammer 3.
- **The can under the unseeded cloud** (section 7.13): dropped on its first
  carry or while the pose is taken, four runs of four. Which pose it was
  given is not known; the logs of those runs were not read.
- **The marker under the unseeded cloud** (section 7.13): not lifted in
  four runs of four. The pick is frozen.
- **The plate on seed 3** stopped at tick 619 with the plate in the hand
  and 2,380 ticks unused: the folded pose was refused ("the arm would
  jump" at step 10 of 24 of turning the hand in place), the arm as lifted
  was refused a route (the wrist 5 and 11 mm from a wall where 12 are asked
  for), and the same plan was tried three times (P22).
- **Leading the first carry faster where a finger is under the object**
  (finding 24). Not tried.
- V2-4, V2-5 and V2-9 have not been measured on their own.
