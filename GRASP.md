# Grasp: factors, their effects, and the change log

This file records what decides whether a pick succeeds, what happened when
each factor was changed, and every change that made one object succeed or
another fail. Add an entry for each such change as it is made.

**Evidence level.** Unless an entry says otherwise, an observation is one
trial per object on maze seed 42 with cloud seed 300, measured from the
simulator by `tools/pick_all.py`. No factor has yet been swept over several
cloud draws, so nothing here is a success *rate*; where the text says an object
"passes", read "passed in the one trial run". A pass means: lifted at least
10 cm with both pads on it, held 50 ticks, and still held after the back-off.

## 1. Factors in a grasp

A pick is decided in five steps. Each has settings; the names are the fields
in the code.

### 1.1 Which grasps are proposed (`grasping/candidates.py`, `CandidateSettings`)

| Factor | Value | What it does |
|---|---|---|
| `cone` | 15° | A pair of cloud points is a grasp if both surface normals lie within this angle of the line between them. |
| width limit | opening − 8 mm (94.85 mm); "tight" up to the full opening (102.85 mm) | Pairs wider than the first limit are proposed only as tight fits, with the hand fully open and no room to spare. |
| `clearances` | 12, 8, 6, 4 mm per side | Gap between each pad and the object before closing. The widest that fits is used; for thin walls every one is proposed. |
| thin walls: `wall_reach`, `max_wall` | 12 mm, 12 mm | A point with opposed-normal points close behind it is a thin wall (rim, flange); the far contact is placed straight through it. |
| contact position along the pad (`offset`) | 0 (pad middle) to 25.5 mm (6 mm from the tip) | Chosen as near the pad middle as the palm (4 mm from the object) and the fingertips (3 mm above the support) allow. |
| `rolls`, `per_pair` | 12, 2 | Approach directions tried around each closing line, and how many are kept. |
| score weights | alignment 0.35, nearness to the cloud's centre 0.25, approach from above 0.25, depth on the pad 0.15 | Order in which grasps are tried. Thin walls count as alignment 0.5; tight fits have their alignment scaled by 0.3. |
| support height | top of the highest surface under each grasp | The level the fingers must stay above. Under a part that overhangs a table it is the floor. |

### 1.2 Which proposals survive (`solver.py`, `grasping/arm_planner.py`)

| Factor | Value | What it does |
|---|---|---|
| weight rule: `grip_safety`, `finger_stiffness`, `squeeze` | 2, 6,000 N/m, 30 mm | Grip force is stiffness × how far the finger target lies inside the object (at most the squeeze; a finger cannot be commanded past closed, so a thin wall is held weakly). Friction must hold twice the weight. |
| open-hand check: `object_margin`, `tight_margin`, `close_margin` | 1.5 mm, −1.5 mm, 2 mm | The hand alone, placed at the grasp in the planning copy, must clear the object's real collision shape and the scene by these gaps. |
| `max_found`, `max_candidates` | 400, 60 | How many proposals are checked, and how many survivors go on. |

### 1.3 Where the base stands (`navigation/parking.py`, `ParkingSettings`)

| Factor | Value | What it does |
|---|---|---|
| `distances`, `directions` | 0.7, 0.6, 0.8, 0.5, 0.9 m; 16 | Places tried around the object, nearest side first. |
| `max_candidates`, `reach_restarts` | 12, 2 | How many grasps are tested for reach at each place, and how hard. |
| `travel_weight` | 0.2 | Preference for grasps that close along the line from base to object. |

### 1.4 How the arm gets there (`grasping/arm_planner.py`, `PlannerSettings`)

| Factor | Value | What it does |
|---|---|---|
| `pregrasp` | 8 cm | Length of the straight approach. |
| `margin`, `transit_object_margin` | 10 mm, 20 mm | Gaps kept in transit. |
| `lift`, `least_lift` | 30 cm, 12 cm | Longest and shortest straight lift planned. |
| `wedged` | 3 mm | An object this deep inside a fixed body is not checked against it on the way out. |

### 1.5 How the grasp is executed (`grasping/executor.py`, `ExecutorSettings`; `solver.py`)

| Factor | Value | What it does |
|---|---|---|
| `transit_step`, `approach_step`, `tight_step`, `lift_step` | 0.06, 0.04, 0.015, 0.02 rad/tick | Arm speed in each phase. The transit speed also decides where the wedged marker lands (section 5). |
| `moved_tolerance` | 15 mm | The object may shift this far before the close. |
| `close_step`, `contact_force` | 3 mm/tick, 1 N | Closing speed, and the force that counts as a contact. |
| `squeeze` | 30 mm | Finger targets go this far inside the object: about 180 N per finger on a wide object, 12–16 N on the plate's 7 mm flange. |
| `grip_ticks`, `lost_ticks` | 8, 4 | Ticks both pads must hold before the lift; ticks without the hold that count as a loss. |
| `lift_goal` | 15 cm | The lift stops when the object itself has risen this far. |
| rake: `rake_gap`, `rake_height`, `rake_leans`, `drag_step`, `overhang` | 6 mm, 6 mm, 0/20/40°, 6 mm/tick, 0.6 | When no grasp can resist the weight's turning moment (`moment_safety` 2) and the object is on a table: where the open hand goes behind the object, how fast the base pulls, and what share of the object's half-length ends up over the edge. |
| back-off speed (`carry_step`, `min_carry_step`, `carry_safety`) | at most 20, at least 3 mm/tick, 0.5 | Limited by grip × friction ÷ mass, so the base's jolt at each tick cannot slide the object. |
| `settle_wait_ticks` | 120 | An object still moving after this is approached anyway. |

## 2. What changing each factor did

| Factor changed | From → to | Observed | Trials |
|---|---|---|---|
| support height | surface under the object's bottom → under its top | Marker: from "no parking pose" on every attempt (grasps inside the table ranked first) to a planned grasp. | marker, 1 before and 1 after |
| thin-wall proposals | none → on | Bowl: 0 → 60 proposals. Plate: 0 → 95. Neither picked yet: the first 12 proposals tested for reach were all ones the real shape rules out. | bowl, plate, 1 each |
| tight-fit proposals | none → on | Can: 0 → proposals. | can, 1 |
| open-hand check before the reach test | off → on | Bowl passes (lift at tick 184). Bleach cleanser passes (403). Can and marker get plans. Plate: 0 of 95 fit (hand overlapped the collision flange by 1–4 mm). | 5 objects, 1 each |
| `clearances` for thin walls | tightest that fits the cloud → all four offered | Plate: 0 → 1–2 grasps that fit; gripped at 12–16 N per finger. | plate, 2 |
| lift | fixed 15 cm of hand travel → until the object has risen 15 cm (path up to 30 cm) | Plate: before, held but its centre rose only 6.7–8.2 cm because it hangs tilted; after, lost at about 50° of tilt. No effect on objects that do not tilt (they stop at 15 cm as before). | plate, 2 before and 2 after |
| following lift | off → on | Plate lost earlier, at 23° instead of 50°. Removed. | plate, 2 |
| `squeeze` | 20 → 30 mm | Can: 132 → 180 N per finger. Not tested alone. | can |
| back-off speed | fixed 20 mm/tick → limited by grip, friction and mass | Can (1.05 kg): lost after 0.18–0.28 m at 20 mm/tick; held for the full 0.6 m at 3 mm/tick. | can, 3 lost, 2 held |
| back-off speed allowance for grasps closing along the travel line | none → 5× faster | Can lost: at 13.6 mm/tick it swung 25° and sank 2 cm in 30 ticks. Removed. | can, 1 |
| order of tight fits | by cloud score → by least overlap with the real shape, after all roomy grasps | Can: with this alone, a weak pinch on the lid's lip was tried first and lost in the lift. Fixed by the weight rule below. | can, 1 |
| weight rule | off → on | Can: lip pinches (a few newtons against 10.3 N of weight) are dropped; the tight fit is tried first and passes on the first attempt (lift at tick 167). | can, 1 |
| arm solver budget | CPU time → iteration counts | Can: results no longer change with machine load (it failed when unit tests ran alongside, passed alone). | can, 2 |
| rake speed | 20 → 6 mm/tick (and overhang share 0.75 → 0.6) | Plate: fell off the table → stays on it. | plate, 1 each |
| support height | one for the object → one per grasp | Plate overhanging by 10 cm (placed by an experiment): picked on the first attempt. | plate, 1 |
| arm retreat after a failed grasp | none → open, back out along the approach, then to the ready pose | Can, plate: second attempts became possible (before, the lowered arm blocked every parking pose: 65 of 65). | can, plate |

Not yet varied on purpose: `cone`, the score weights, `rolls`, `pregrasp`,
`moved_tolerance`, `close_step`, `grip_ticks`, parking `distances`,
`travel_weight`, the arm speeds (changed once, together with other things).

## 3. Change log

Each row is a change to the grasp code and its effect per object.
"13/15" style totals are from one run of all 15 objects.

| # | Change | Fixed | Broke or left failing | Total |
|---|---|---|---|---|
| 1 | First version: opposed point pairs only, best 60 by score, reach tested on the first 12. | sugar box, soup can, mustard, potted meat, banana, strawberry, apple, mug (floor), hammer, foam brick | can, bowl, plate (no proposals); bleach cleanser, marker (no parking pose) | 10/15 |
| 2 | Support taken under the object's top. | marker gets plans | | |
| 3 | Thin-wall and tight-fit proposals. | proposals for bowl, plate, can | none picked yet | |
| 4 | Exact open-hand check on every proposal before the reach test. | bowl, bleach cleanser | plate (nothing fits) | |
| 5 | Arm retreats after a failed grasp. | later attempts for can and plate | | |
| 6 | All four clearances offered for thin walls. | plate is gripped | plate does not reach 10 cm | |
| 7 | Lift until the object has risen; squeeze 30 mm; faster transit, approach and drive; preference for grasps closing along the travel line. | can lifted and held | can lost in the back-off; plate lost at 50° | |
| 8 | Following lift: the hand turns with an object that turns in it during the lift. | | plate lost at 23°; removed | |
| 9 | Back-off speed from grip, friction and mass. | can held through the back-off at 3 mm/tick | | |
| 10 | Faster back-off for grasps closing along the travel line. | | can lost at 13.6 mm/tick; removed | |
| 11 | Tight fits ordered by overlap, after roomy grasps; slower tight approach; solver bounded by iterations. | | can: lip pinch tried first, lost in the lift | |
| 12 | Weight rule. | can, first attempt | | 13/15 |
| 13 | Planned swing: before the lift, the hand swings the object up about its far edge, turning with it; pivot taken from the cloud, then rolled on the real collision shape in the planning copy. Parking also has to reach the top of the swing; rim grasps adjusted locally (opening, depth) by the exact hand check. | | plate lost at 10–25° of swing, then at the first tick of it. Removed. | |
| 14 | Support height taken under each grasp, not once for the object. | plate picked on the first attempt **when an experiment places it overhanging the table edge by 10 cm** (lift at tick 123, held 258 ticks, whole pad on the flange, 13 N per finger) | no effect on an object lying wholly on the table | |
| 15 | Drag by pinch: grip the rim, back the base away. | | plate lost within 6 ticks of the drag starting | |
| 16 | Rake instead: open hand just beyond the object's far edge, fingertips 6 mm above the table, base backs away; used when no grasp can resist the weight's turning moment and the object is on a table. Hand may lean back 20° or 40° (straight down was out of reach). | plate reaches the edge | at 20 mm/tick the plate kept sliding and fell to the floor | |
| 17 | Rake at 6 mm/tick, 60% of the half-length left over the edge. | plate stays on the table, is re-gripped and lifted 15 cm (tick 290) | that grip let go during the 50-tick hold. **Bowl broke** (passed before): lost in the back-off. With 1,600 proposals and the local adjustment, 60 bowl grasps fit instead of 4, and a pinch with the pad tips from straight above (offset 24 mm) outranked the grasp with the whole pad on the rim (offset 0) that had passed. | 12/15 |
| 18 | Grasps ranked in tiers that come before the score: can resist the weight's turning moment; on the overhanging part; at least half the pad on the object; not tried before. (My travel-line ordering had been re-sorting by score alone and discarding the other orderings.) | bowl (first attempt, lift at tick 174); plate (raked, then lifted at tick 313, held 50, backed off 0.6 m) | | bowl and plate rerun only |
| 19 | Rake for an out-of-reach object on the floor under a table: pull it out the nearest way (hand may lean back up to 60°, reach test with 12 restarts). | | marker 0.35 m under the table: still out of reach from all base positions tried | |
| 20 | `transit_step` 0.09 → 0.06. | marker: lands on open floor instead of under the table, picked at tick 410 | all other objects lift 3 to 19 ticks later | 15/15 |

## 4. Result after change 20

Two runs of all 15 on maze seed 42 with the final code, one per cloud draw
(`runs/all_05/results.csv`, `runs/all_05_cloud301/results.csv`): **15 of 15
pass in each**. Every lift is by tick 494 on cloud seed 300 and by tick 560 on
cloud seed 301; the deadline is 600. Cloud seed 301 was not used during
development. This is still two trials per object on one maze.

| Object | Cloud 300: lift tick (attempts) | Cloud 301: lift tick (attempts) | Note |
|---|---|---|---|
| master chef can | 159 (1) | 189 (1) | tight fit |
| sugar box | 188 (1) | 176 (1) |  |
| tomato soup can | 165 (1) | 182 (1) |  |
| mustard bottle | 350 (2) | 334 (2) | first attempt: still rocking, moved during the drive |
| potted meat can | 157 (1) | 157 (1) |  |
| banana | 170 (1) | 177 (1) |  |
| strawberry | 226 (1) | 211 (1) |  |
| apple | 218 (1) | 228 (1) |  |
| bleach cleanser | 494 (3) | 533 (3) | still rocking on the first attempt; one more failed attempt |
| bowl | 193 (1) | 175 (1) | rim, whole pad on it |
| mug | 298 (2) | 301 (2) | floor pick; first attempt: moved during the drive |
| plate | 296 (2) | 322 (2) | first attempt rakes it to the table's edge |
| large marker | 410 (2) | 560 (3) | thrown out of the table by the first approach, then picked from the floor |
| hammer | 165 (1) | 153 (1) |  |
| foam brick | 175 (1) | 160 (1) |  |

## 5. What is fragile

**Plate.** While it lies wholly on the table the only grasps that fit are
pinches of the rim with the pad tips (about 9 mm of pad on the flange, 12–16 N
per finger). That holds the plate's weight twelve times over, but not its
turning moment (0.074 N·m about the grip): as the hand rises the plate levers
round with its far edge on the table and slips out at about 50°. Approaches
along the rim, which would put the full pad width on the flange, hit the table
by 9–12 mm.

What the traces showed about that pinch: the pads sit at 45° on a
wedge-shaped rim; closing pushes the near rim down about 8 mm, tipping the
plate 10° on its near foot. The "grip" is a jam, not a flat pinch, and it lets
go as soon as the hand moves in any direction but straight up (a swing that
tracked the plate to within 0.5° still lost it; so did a pull).

What works: with the plate overhanging the table edge the lower pad goes
right under it and the whole pad lies on the flange. The solver now gets it
there by raking (change 16), and with the tiers of change 18 the grasp taken
there has the whole pad on the flange. One passing trial so far.

**Large marker.** It rests wedged through the tabletop. The simulator throws
it out while the hand is still in transit above it, before any contact, so it
cannot be gripped where it is wedged; it always ends on the floor. Where it
lands is set by the hand's approach and is chaotic (one trial each, maze seed
42, cloud seed 300):

| `transit_step` (rad/tick) | Lands at (x, y) | Outcome |
|---|---|---|
| 0.03 | (1.92, −0.88), open floor | picked, lift at tick 797 |
| 0.06 | (−0.66, 0.63), open floor near the robot | picked, lift at tick 423 |
| 0.09 | (−0.15, 0.15), under the middle of the table | out of reach |
| 0.12 | (−0.19, 0.13), under the middle of the table | out of reach |

`transit_step` is set to 0.06 for this reason alone. This is one favourable
outcome of a chaotic event, not a fix: another cloud draw, maze or code change
that alters the approach can put the marker back under the table. From the
table's edges the arm reaches about 0.25 m under it; a rake from the nearest
edge did not reach the marker 0.35 m in (change 19).

## 6. Carry and reach (segment 3, in progress)

Written on 2026-11-20 after reading the old notes for known problems
(`old/agents/fixes.md` 9, 10, 14, 15, 19-24, 31-35); no old code was used.

**How it works.** After the back-off the arm keeps its post-lift pose. The
torso rises toward the goal's height. The base turns on the spot until the arm
points down the route, then the object is led along the route with the base
trailing an arm's length behind on the same route, heading toward the object,
so the arm stays inside every corner. Where the hand points straight down, the
wrist's roll keeps the pads' closing line along the object's direction of
travel. The whole carry, object in hand, is checked for collisions in the
planning copy before it starts. At the end a short straight reach puts the
object, not the hand, on the goal.

**Result.** One run of all 15 to delivery, maze seed 42, cloud seed 300
(`runs/deliver_04/results.csv`), on the uncommitted working tree: **6 of 15
delivered**, at ticks 2,375-2,680.

| Object | Outcome | Cause |
|---|---|---|
| tomato soup can, mustard bottle, potted meat can, banana, foam brick | delivered first carry, ticks 2,375-2,446 | |
| strawberry | delivered at 2,680 | lost once in the carry, picked up again |
| apple | not delivered | lost in the carry; it was delivered in two earlier runs (change 23 below) |
| mug | 27 cm from the goal at tick 3,000 | lost twice in the carry and picked up again; ran out of ticks in the reach. Delivered at 2,466 in a two-object run before change 23 |
| master chef can | lost in the first 20 ticks of the turn on the spot | the only grasp reachable from the parking pose is tilted 30 degrees, 12 mm from the pad middle |
| sugar box | lost in every carry | not yet traced; the old notes report it turning about the pads' line because its collision faces are not parallel |
| bleach cleanser | lost in the carry | 1.13 kg: at the 9-11 mm per tick the budget requires, the base's jolt is about as large as the grip |
| hammer | lost in every carry | turns in the hand (5 to 27 degrees over 3 m) about the line between the pads, gripped on the handle |
| bowl, plate | no clear route | the robot comes within 1-7 mm of a wall (bowl) or 2-4 mm into it (plate) at one place on every route tried: with these rim grasps the arm sticks out sideways |
| large marker | not delivered | chaotic landing again: this run it is lifted at tick 780 and lost later |

**Changes made for the carry, and their effect on grasps**

| # | Change | Fixed | Broke or left failing |
|---|---|---|---|
| 21 | First carry: object led at up to 30 mm per tick. | soup can, mustard, apple, foam brick delivered (ticks 1,238-1,582) | potted meat can slides down out of the fingers once the speed passes about 14 mm per tick; banana slips in the corners; nine lost in all |
| 22 | The object is led only 1.25 times faster than the ticks left require (about 9-11 mm per tick). | potted meat can, banana, strawberry delivered | deliveries now take about 2,400 ticks; nothing is left for a second carry |
| 23 | Grasp score and tiers use the centre of mass: the closing line should pass beside it by as little as possible (the object turns freely about that line), a centre of mass above the line counts against the grasp, and grasps the weight would swing rank last. Tight fits keep their score order. | no object newly delivered | apple lost in the carry (delivered before); marker's first lift moved from tick 410 to 780 |
| 24 | Arm solver leaves out a joint stuck on its limit. | mug: the final reach, 0.58 m straight up from a fully stretched arm with the torso at its limit, used to stop at "out of reach" | |
| 25 | Squeeze 30 to 20 mm (tested, not kept). | | sugar box, bleach, hammer, can still lost |

Changes 21-25 are in the working tree only. The committed solver (`947a0e8`)
still passes the 15-of-15 pick of section 4 and holds its pose after the
back-off.

## 7. Changes since section 6 (the side hold)

Written on 2026-11-20. Sections 1-6 are left as they were written; where
this section contradicts them, this section is the later finding. The carry
itself, attempt by attempt, is in `NAVIGATE.md`.

**What was wrong in section 6.** Change 23 and the "swings" tier treated an
object turning about the pads' line as a fault of the grasp. Single-object
tests showed the opposite for the carry:

- The pads' line is a loose hinge (no spinning friction). Turning the hand 90
  degrees about it moved the hammer 31 degrees; it settled with its centre of
  mass below the line. A hold that fights the hinge loses; a hold in which
  the object already hangs below the line has nothing to lose.
- The hammer in section 6 was gripped 14 mm from its centre of mass and was
  still lost. Its fault was not swinging but yaw: held level, the long handle
  lagged the base's turn on the spot by 15-19 degrees and worked loose.
- An object whose jolt exceeds the pads' friction creeps down through the
  pads (hammer: 1 mm per 40 ticks). Only something resting on the fingers
  stops that.

**New factors** (add to section 1.2, "which proposals survive"):

| Factor | Setting | Effect |
|---|---|---|
| Hanging grasp for a long object lying down | `Config.hang_long` (on), `long_length` 0.25 m, `long_ratio` 2.0, `hang_grip` 0.035 m | Grasps that close across the length, between the centre of mass and the nearer end, are put first and form their own tier in place of "swings". Applies to the hammer only among the 15. |
| Mass ranking on or off | `Config.rank_by_mass` (on) | Off restores the ranking of section 4 (nearness to the middle of the bounding box, no "swings" tier). |

**Change log, continued**

| # | Change | Fixed | Broke or left failing |
|---|---|---|---|
| 26 | `rank_by_mass` switch for change 23. | Nothing by itself; allows comparison. | On cloud 300 the hammer's first grasp is the same either way. |
| 27a | Hammer gripped at least 6 cm from its centre of mass, either side (first version of the hanging grasp). | Lifted at tick 162 as before; hangs on the z axis in the side hold; carried 9.3 m instead of 2.1 m. | Gripped on the handle side, nothing stops it creeping down: it spun onto the handle's narrow side (grip 115 to 89 N) and slid out. |
| 27b | Hammer gripped between the centre of mass and the head, at least 3.5 cm from the centre of mass. | **Hammer delivered** (ticks 2,392, 2,361 alone; 2,354 in the full run): the head is above the fingers and comes to rest on them. Lift still at tick 162. | |
| 28-31 | The hold for the carry (roll level, lay level, fold the arm). No change to which grasp is chosen. | See `NAVIGATE.md`. | |

**Pick results with these changes** (lift ticks from `runs/deliver_05`, same
maze and cloud as section 4): all 15 lifted and held 50 ticks. Lift ticks are
within 5 of `runs/deliver_04` for every object, hammer included. The marker's
first lift is still at tick 780 (change 23, which is still on); in section 4,
before change 23, it was 410.

**Still open on the grasp side**

- Change 23 has not delivered anything and cost the apple and the marker's
  lift time. Whether to switch it off (`rank_by_mass=False`) is undecided;
  the hammer no longer depends on it.
- The plate's rim grasp gives 13 N per finger. Hung on edge it is lost after
  6 m; it may be better carried flat with a finger under it.
- Master chef can: still only the 30-degree tilted grasp is within reach.

**Correction (later the same day).** A full run with `rank_by_mass=False`
(`runs/deliver_07`, `NAVIGATE.md` attempt 7) did not bring back the apple or
the marker's lift time: the apple is still lost in the carry and the marker
is first lifted at tick 757. The statements above and in section 6 that
change 23 cost those two are therefore not supported by that switch alone.
The switch does trade two objects: the sugar box is delivered with it off
(tick 2,431) and the mug with it on (tick 2,505).

**Later the same day: what else was tried on the grasp side** (details in
`NAVIGATE.md`, attempts 8-10). None of these is on by default.

| Tried | Result |
|---|---|
| Fingers fully closed for the carry (`carry_squeeze`) | 9 of 15: apple, mug and sugar box squeezed out |
| Half the squeeze (15 mm) on the hanging sugar box | lost sooner |
| The hanging-grasp rule widened to the sugar box (`long_length` 0.17, `long_ratio` 1.7, `long_mass` 0.3) | sugar box hangs and is carried 18.7 m of 21.4, then creeps out of the pads; in the full run the mustard bottle needed four re-picks |
| Surfaces under the pads checked for being parallel (`PlanningSim.wedge`, recorded as `wedge_deg`) | sugar box 2 degrees, soup can 1: the sugar box is not squeezed out as a wedge |

The pick itself is unchanged by all of the above: in `runs/deliver_11` every
object is lifted at the tick it was in `runs/deliver_09`.

