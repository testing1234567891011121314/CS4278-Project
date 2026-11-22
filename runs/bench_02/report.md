# bench_02 against bench_01

Baseline bench_01: commit `376e759`, 1500 trials. Full bench_02: commit `4e155e7`, 1500 trials. Paired on 1500 trials: 15 objects on 100 seeds (1000 to 1099).

## 1. Baseline against full

| Measure | Baseline | Full | Difference (95 % interval) | Gained | Lost | p (sign test) |
|---|---|---|---|---|---|---|
| Delivered, % of trials | 76.1 | 80.5 | +4.4 (+3.2 to +5.7) | 99 | 33 | 7.5e-09 |
| Delivered with no drop, % | 62.6 | 69.1 | +6.5 (+5.0 to +8.0) | 123 | 26 | 2.75e-16 |
| Points of 40, mean | 34.15 | 34.88 | +0.72 (+0.55 to +0.90) | | | |
| Lifted, % (the pick, not changed) | 95.1 | 95.1 | +0.0 (+0.0 to +0.0) | 0 | 0 | 1 |

First lift at the same tick in both: 1500 of 1500 trials.

## 2. By object

| Object | Trials | Delivered: baseline | full | gained | lost | p | No drop: baseline | full | gained | lost | p | Points: baseline | full |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 master_chef_can | 100 | 30 | 38 | 9 | 1 | 0.021 | 29 | 36 | 8 | 1 | 0.039 | 23.2 | 24.2 |
| 1 sugar_box | 100 | 84 | 98 | 16 | 2 | 0.0013 | 53 | 88 | 38 | 3 | 1e-08 | 36.2 | 39.3 |
| 2 tomato_soup_can | 100 | 94 | 90 | 1 | 5 | 0.22 | 91 | 85 | 0 | 6 | 0.031 | 38.9 | 38.2 |
| 3 mustard_bottle | 100 | 98 | 100 | 2 | 0 | 0.5 | 94 | 95 | 1 | 0 | 1 | 39.5 | 39.7 |
| 4 potted_meat_can | 100 | 100 | 100 | 0 | 0 | 1 | 100 | 100 | 0 | 0 | 1 | 40.0 | 40.0 |
| 5 banana | 100 | 96 | 99 | 3 | 0 | 0.25 | 89 | 91 | 2 | 0 | 0.5 | 39.1 | 39.5 |
| 6 strawberry | 100 | 77 | 81 | 9 | 5 | 0.42 | 36 | 37 | 3 | 2 | 1 | 33.8 | 34.4 |
| 7 apple | 100 | 84 | 86 | 4 | 2 | 0.69 | 75 | 78 | 6 | 3 | 0.51 | 37.1 | 37.4 |
| 8 bleach_cleanser | 100 | 53 | 77 | 26 | 2 | 3e-06 | 49 | 75 | 26 | 0 | 3e-08 | 32.1 | 36.0 |
| 9 bowl | 100 | 87 | 87 | 6 | 6 | 1 | 78 | 81 | 8 | 5 | 0.58 | 37.8 | 37.7 |
| 10 mug | 100 | 84 | 86 | 5 | 3 | 0.73 | 45 | 48 | 6 | 3 | 0.51 | 35.3 | 35.7 |
| 11 plate | 100 | 24 | 26 | 4 | 2 | 0.69 | 21 | 24 | 5 | 2 | 0.45 | 23.4 | 23.4 |
| 12 large_marker | 100 | 47 | 49 | 3 | 1 | 0.62 | 40 | 40 | 1 | 1 | 1 | 20.3 | 20.5 |
| 13 hammer | 100 | 84 | 91 | 11 | 4 | 0.12 | 44 | 60 | 16 | 0 | 3.1e-05 | 35.8 | 37.3 |
| 14 foam_brick | 100 | 100 | 100 | 0 | 0 | 1 | 95 | 98 | 3 | 0 | 0.25 | 39.8 | 39.9 |

## 3. Points by item (sum over the paired trials)

| Item | Baseline | Full | Difference |
|---|---|---|---|
| lift_600 | 8178 | 8178 | +0 |
| lift_10cm | 8556 | 8556 | +0 |
| hold_50 | 8406 | 8406 | +0 |
| grasp_pose | 8406 | 8406 | +0 |
| reach_10cm | 6852 | 7248 | +396 |
| reach_in_time | 6852 | 7248 | +396 |
| no_drop | 3980 | 4272 | +292 |

## 4. Each part of the carry switched off

Full against full without the part, on the trials of the full run in which the part acted. "Gained by the part" is a trial the full run delivers and the run without the part does not. The effect is over all trials of the full run, the trials in which the part did not act counting as no change; the check is a sample of those, run without the part.

| Part | Switched off by | Acted in | Delivered: gained by the part | lost by it | p | No drop: gained | lost | p | Effect on delivered, points of % (95 %) | Effect on no drop | Effect on points of 40 | Check: ran the same |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arms | `travel_arms=1` | 142 (9.5 %) | 14 | 9 | 0.4 | 18 | 6 | 0.023 | +0.3 (-0.2 to +0.9) | +0.8 (+0.1 to +1.5) | +0.07 (-0.01 to +0.15) | 40 of 40 |
| joint_by_joint | `travel_more_ways=1` | 323 (21.5 %) | 29 | 16 | 0.072 | 46 | 13 | 1.9e-05 | +0.9 (+0.0 to +1.7) | +2.2 (+1.2 to +3.3) | +0.19 (+0.07 to +0.32) | 40 of 40 |
| other_ways | `travel_more_ways=0` | 489 (32.6 %) | 60 | 24 | 0.00011 | 88 | 22 | 1.6e-10 | +2.4 (+1.3 to +3.6) | +4.4 (+3.1 to +5.7) | +0.46 (+0.29 to +0.63) | 40 of 40 |
| weak_holds | `carry_fragile=0` | 243 (16.2 %) | 23 | 15 | 0.26 | 25 | 13 | 0.073 | +0.5 (-0.2 to +1.3) | +0.8 (+0.1 to +1.5) | +0.10 (-0.02 to +0.21) | 40 of 40 |
| reseat | `reseat_sag=0` | 27 (1.8 %) | 1 | 1 | 1 | 1 | 1 | 1 | +0.0 (-0.2 to +0.2) | +0.0 (-0.2 to +0.2) | +0.00 (-0.03 to +0.03) | 40 of 40 |
| stall | `free_reach=0` `carry_retry_margins=(1.0,)` | 62 (4.1 %) | 11 | 1 | 0.0063 | 9 | 0 | 0.0039 | +0.7 (+0.2 to +1.1) | +0.6 (+0.2 to +1.1) | +0.06 (+0.00 to +0.11) | 40 of 40 |

The system with one part removed, over all 1500 paired trials:

| System | Delivered, % | Delivered with no drop, % | Points of 40 |
|---|---|---|---|
| baseline | 76.1 | 62.6 | 34.15 |
| full | 80.5 | 69.1 | 34.88 |
| full without arms | 80.2 | 68.3 | 34.81 |
| full without joint_by_joint | 79.7 | 66.9 | 34.69 |
| full without other_ways | 78.1 | 64.7 | 34.42 |
| full without weak_holds | 80.0 | 68.3 | 34.78 |
| full without reseat | 80.5 | 69.1 | 34.88 |
| full without stall | 79.9 | 68.5 | 34.82 |

Every part off, on 100 trials drawn at random, against the baseline's same trials: 100 ran the same (lift, delivery and last tick, and the failures), 100 agree on delivered and on no drop. Delivered: baseline 73, every part off 73.

Repeats: 40 trials of the full run drawn at random and run again with the same settings; 40 ran the same.

## 5. First carry of each trial by travel pose, full (bench_02)

| Travel pose | Way in | Carries | Not lost | Lost taking the pose | Lost in the carry | Lost otherwise |
|---|---|---|---|---|---|---|
| level, finger under | straight | 489 | 366 (74.8 %) | 46 | 76 | 1 |
| down, pads along | straight | 256 | 168 (65.6 %) | 5 | 83 | 0 |
| level, finger under | joint by joint | 188 | 157 (83.5 %) | 4 | 27 | 0 |
| folded | straight | 80 | 37 (46.2 %) | 18 | 16 | 9 |
| level (low) | the move, then the turn | 62 | 60 (96.8 %) | 0 | 2 | 0 |
| level | joint by joint | 61 | 58 (95.1 %) | 1 | 2 | 0 |
| level | straight | 60 | 57 (95.0 %) | 0 | 3 | 0 |
| level, finger under (low) | the move, then the turn | 48 | 36 (75.0 %) | 4 | 8 | 0 |
| level, finger under | the move, then the turn | 42 | 30 (71.4 %) | 2 | 10 | 0 |
| as lifted | straight | 41 | 11 (26.8 %) | 0 | 9 | 21 |
| folded (low) | straight | 35 | 14 (40.0 %) | 1 | 14 | 6 |
| folded | joint by joint | 17 | 10 (58.8 %) | 0 | 7 | 0 |
| level, finger under (low) | straight | 13 | 9 (69.2 %) | 0 | 4 | 0 |
| level, finger under (low) | joint by joint | 8 | 5 (62.5 %) | 2 | 1 | 0 |
| level (low) | straight | 5 | 4 (80.0 %) | 0 | 1 | 0 |
| level | the move, then the turn | 3 | 3 (100.0 %) | 0 | 0 | 0 |
| folded (low) | the move, then the turn | 2 | 1 (50.0 %) | 0 | 1 | 0 |
| folded | the move, then the turn | 2 | 0 (0.0 %) | 2 | 0 | 0 |
| level (low) | joint by joint | 1 | 0 (0.0 %) | 0 | 1 | 0 |
| down, pads along | the move, then the turn | 1 | 0 (0.0 %) | 0 | 1 | 0 |

### Every carry, full (bench_02)

| Travel pose | Way in | Carries | Not lost | Lost taking the pose | Lost in the carry | Lost otherwise |
|---|---|---|---|---|---|---|
| level, finger under | straight | 687 | 484 (70.5 %) | 82 | 120 | 1 |
| down, pads along | straight | 314 | 194 (61.8 %) | 17 | 103 | 0 |
| level, finger under | joint by joint | 248 | 192 (77.4 %) | 11 | 45 | 0 |
| level | straight | 95 | 69 (72.6 %) | 15 | 11 | 0 |
| folded | straight | 80 | 37 (46.2 %) | 18 | 16 | 9 |
| level | joint by joint | 67 | 64 (95.5 %) | 1 | 2 | 0 |
| level (low) | the move, then the turn | 63 | 61 (96.8 %) | 0 | 2 | 0 |
| level, finger under (low) | the move, then the turn | 51 | 39 (76.5 %) | 4 | 8 | 0 |
| level, finger under | the move, then the turn | 47 | 34 (72.3 %) | 3 | 10 | 0 |
| as lifted | straight | 46 | 11 (23.9 %) | 0 | 9 | 26 |
| folded (low) | straight | 38 | 16 (42.1 %) | 2 | 14 | 6 |
| folded | joint by joint | 18 | 11 (61.1 %) | 0 | 7 | 0 |
| level, finger under (low) | straight | 13 | 9 (69.2 %) | 0 | 4 | 0 |
| level, finger under (low) | joint by joint | 8 | 5 (62.5 %) | 2 | 1 | 0 |
| level (low) | straight | 5 | 4 (80.0 %) | 0 | 1 | 0 |
| level | the move, then the turn | 3 | 3 (100.0 %) | 0 | 0 | 0 |
| folded (low) | the move, then the turn | 2 | 1 (50.0 %) | 0 | 1 | 0 |
| folded | the move, then the turn | 2 | 0 (0.0 %) | 2 | 0 | 0 |
| down, pads along | the move, then the turn | 2 | 0 (0.0 %) | 1 | 1 | 0 |
| down, pads along (low) | straight | 2 | 1 (50.0 %) | 0 | 1 | 0 |
| down, pads along (low) | joint by joint | 1 | 0 (0.0 %) | 0 | 1 | 0 |
| level (low) | joint by joint | 1 | 0 (0.0 %) | 0 | 1 | 0 |

## 6. First carry by object: carries not lost / carries, by travel pose (full)

| Object | level, finger under | down, pads along | level | folded | level, finger under (low) | level (low) | as lifted | folded (low) |
|---|---|---|---|---|---|---|---|---|
| 0 master_chef_can | 3 / 5 | 2 / 14 | 4 / 6 | 28 / 44 | 0 / 1 | 0 / 2 | 0 / 2 |  |
| 1 sugar_box | 20 / 23 | 2 / 4 | 51 / 54 | 0 / 2 | 0 / 3 | 14 / 14 |  |  |
| 2 tomato_soup_can | 53 / 68 | 31 / 31 |  |  | 1 / 1 |  |  |  |
| 3 mustard_bottle | 29 / 30 | 3 / 3 | 37 / 38 |  | 2 / 4 | 24 / 25 |  |  |
| 4 potted_meat_can | 68 / 68 | 19 / 19 |  |  | 13 / 13 |  |  |  |
| 5 banana | 61 / 65 | 24 / 29 |  |  | 6 / 6 |  |  |  |
| 6 strawberry | 30 / 74 | 6 / 20 |  | 1 / 2 | 1 / 4 |  |  |  |
| 7 apple | 51 / 59 | 21 / 32 |  |  | 6 / 9 |  |  |  |
| 8 bleach_cleanser | 17 / 23 | 2 / 16 | 26 / 26 | 0 / 1 | 4 / 7 | 26 / 27 |  |  |
| 9 bowl | 39 / 50 | 20 / 24 |  | 11 / 17 | 4 / 6 |  | 2 / 3 |  |
| 10 mug | 31 / 66 | 5 / 8 |  | 5 / 15 |  |  | 6 / 7 | 1 / 2 |
| 11 plate |  |  |  | 2 / 18 |  |  | 3 / 23 | 14 / 35 |
| 12 large_marker | 32 / 50 | 7 / 10 |  |  |  |  | 0 / 6 |  |
| 13 hammer | 48 / 67 | 6 / 25 |  |  | 6 / 8 |  |  |  |
| 14 foam_brick | 71 / 71 | 20 / 22 |  |  | 7 / 7 |  |  |  |

