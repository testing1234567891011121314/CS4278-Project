# Benchmark: 1500 trials, 100 seeds (1000 to 1099), 15 objects

Commit `376e759`, cloud seeded with the trial seed, 3000 ticks, settings default, host FRC260719-001.

| Object | Trials | Lifted | Lifted by tick 600 | Held 50 | Delivered | Delivered, no drop | Trials with a drop | Mean points of 40 | Median delivery tick |
|---|---|---|---|---|---|---|---|---|---|
| 0 master_chef_can | 100 | 78 | 74 | 74 | 30 (30 %) | 29 (29 %) | 38 | 23.2 | 2446 |
| 1 sugar_box | 100 | 100 | 100 | 100 | 84 (84 %) | 53 (53 %) | 46 | 36.2 | 2449 |
| 2 tomato_soup_can | 100 | 100 | 100 | 100 | 94 (94 %) | 91 (91 %) | 9 | 38.9 | 2486 |
| 3 mustard_bottle | 100 | 100 | 99 | 100 | 98 (98 %) | 94 (94 %) | 6 | 39.5 | 2465 |
| 4 potted_meat_can | 100 | 100 | 100 | 100 | 100 (100 %) | 100 (100 %) | 0 | 40.0 | 2446 |
| 5 banana | 100 | 100 | 100 | 100 | 96 (96 %) | 89 (89 %) | 11 | 39.1 | 2459 |
| 6 strawberry | 100 | 100 | 100 | 93 | 77 (77 %) | 36 (36 %) | 64 | 33.8 | 2527 |
| 7 apple | 100 | 100 | 100 | 100 | 84 (84 %) | 75 (75 %) | 25 | 37.1 | 2497 |
| 8 bleach_cleanser | 100 | 100 | 96 | 100 | 53 (53 %) | 49 (49 %) | 50 | 32.1 | 2485 |
| 9 bowl | 100 | 100 | 100 | 100 | 87 (87 %) | 78 (78 %) | 16 | 37.8 | 2471 |
| 10 mug | 100 | 100 | 99 | 95 | 84 (84 %) | 45 (45 %) | 53 | 35.3 | 2561 |
| 11 plate | 100 | 82 | 78 | 74 | 24 (24 %) | 21 (21 %) | 32 | 23.4 | 2279 |
| 12 large_marker | 100 | 66 | 17 | 65 | 47 (47 %) | 40 (40 %) | 20 | 20.3 | 2567 |
| 13 hammer | 100 | 100 | 100 | 100 | 84 (84 %) | 44 (44 %) | 56 | 35.8 | 2481 |
| 14 foam_brick | 100 | 100 | 100 | 100 | 100 (100 %) | 95 (95 %) | 5 | 39.8 | 2454 |
| **all** | 1500 | 1426 | 1363 | 1401 | 1142 (76 %) | 939 (63 %) | 431 | 34.2 | 2476 |

| Delivered on a seed | Seeds |
|---|---|
| 15 of 15 | 1 |
| 14 of 15 | 4 |
| 13 of 15 | 15 |
| 12 of 15 | 32 |
| 11 of 15 | 22 |
| 10 of 15 | 19 |
| 9 of 15 | 5 |
| 8 of 15 | 1 |
| 7 of 15 | 1 |

| Object | Failure of an attempt | Times |
|---|---|---|
| master_chef_can | no grasp plan | 228 |
| large_marker | no parking pose | 180 |
| bleach_cleanser | object moved during the drive | 138 |
| large_marker | object moved before the close | 133 |
| bleach_cleanser | no grasp plan | 132 |
| plate | object lost during the lift | 116 |
| mustard_bottle | object moved during the drive | 103 |
| master_chef_can | object moved during the drive | 103 |
| mug | object moved during the drive | 97 |
| strawberry | object lost during the carry | 90 |
| plate | dragged to the edge | 82 |
| plate | no parking pose | 82 |
| hammer | object lost during the carry | 80 |
| master_chef_can | no parking pose | 76 |
| large_marker | object moved during the drive | 74 |
| large_marker | no grasp candidates | 69 |
| sugar_box | object lost during the carry | 66 |
| bleach_cleanser | object lost during the carry | 63 |
| plate | no grasp candidates | 57 |
| bowl | no grasp candidates | 48 |
| mug | object lost during the carry | 48 |
| bleach_cleanser | object lost while taking the travel pose | 44 |
| large_marker | no grasp plan | 43 |
| plate | arm did not reach the end of the transit | 37 |
| mug | object lost while taking the travel pose | 36 |
| mug | no parking pose | 35 |
| hammer | object lost while taking the travel pose | 34 |
| master_chef_can | object lost during the carry | 33 |
| bleach_cleanser | no parking pose | 33 |
| apple | object moved before the close | 31 |
| apple | object lost during the carry | 30 |
| plate | no grasp plan | 30 |
| apple | object moved during the drive | 29 |
| mug | object lost during the lift | 28 |
| master_chef_can | object lost during the lift | 27 |
| master_chef_can | object moved before the close | 27 |
| plate | no clear route to the goal | 25 |
| plate | hold not verified after the close | 22 |
| plate | object lost during the carry | 22 |
| apple | no grasp plan | 21 |
