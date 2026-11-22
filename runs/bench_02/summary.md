# Benchmark: 1500 trials, 100 seeds (1000 to 1099), 15 objects

Commit `4e155e7`, cloud seeded with the trial seed, 3000 ticks, settings default, host FRC260719-001.

| Object | Trials | Lifted | Lifted by tick 600 | Held 50 | Delivered | Delivered, no drop | Trials with a drop | Mean points of 40 | Median delivery tick |
|---|---|---|---|---|---|---|---|---|---|
| 0 master_chef_can | 100 | 78 | 74 | 74 | 38 (38 %) | 36 (36 %) | 38 | 24.2 | 2862 |
| 1 sugar_box | 100 | 100 | 100 | 100 | 98 (98 %) | 88 (88 %) | 12 | 39.3 | 2453 |
| 2 tomato_soup_can | 100 | 100 | 100 | 100 | 90 (90 %) | 85 (85 %) | 15 | 38.2 | 2453 |
| 3 mustard_bottle | 100 | 100 | 99 | 100 | 100 (100 %) | 95 (95 %) | 5 | 39.7 | 2470 |
| 4 potted_meat_can | 100 | 100 | 100 | 100 | 100 (100 %) | 100 (100 %) | 0 | 40.0 | 2446 |
| 5 banana | 100 | 100 | 100 | 100 | 99 (99 %) | 91 (91 %) | 9 | 39.5 | 2451 |
| 6 strawberry | 100 | 100 | 100 | 93 | 81 (81 %) | 37 (37 %) | 63 | 34.4 | 2506 |
| 7 apple | 100 | 100 | 100 | 100 | 86 (86 %) | 78 (78 %) | 22 | 37.4 | 2477 |
| 8 bleach_cleanser | 100 | 100 | 96 | 100 | 77 (77 %) | 75 (75 %) | 25 | 36.0 | 2864 |
| 9 bowl | 100 | 100 | 100 | 100 | 87 (87 %) | 81 (81 %) | 18 | 37.7 | 2450 |
| 10 mug | 100 | 100 | 99 | 95 | 86 (86 %) | 48 (48 %) | 50 | 35.7 | 2548 |
| 11 plate | 100 | 82 | 78 | 74 | 26 (26 %) | 24 (24 %) | 38 | 23.4 | 2666 |
| 12 large_marker | 100 | 66 | 17 | 65 | 49 (49 %) | 40 (40 %) | 21 | 20.5 | 2554 |
| 13 hammer | 100 | 100 | 100 | 100 | 91 (91 %) | 60 (60 %) | 40 | 37.3 | 2459 |
| 14 foam_brick | 100 | 100 | 100 | 100 | 100 (100 %) | 98 (98 %) | 2 | 39.9 | 2450 |
| **all** | 1500 | 1426 | 1363 | 1401 | 1208 (81 %) | 1036 (69 %) | 358 | 34.9 | 2473 |

| Delivered on a seed | Seeds |
|---|---|
| 15 of 15 | 4 |
| 14 of 15 | 6 |
| 13 of 15 | 24 |
| 12 of 15 | 37 |
| 11 of 15 | 21 |
| 10 of 15 | 5 |
| 9 of 15 | 3 |

| Object | Failure of an attempt | Times |
|---|---|---|
| master_chef_can | no grasp plan | 238 |
| large_marker | no parking pose | 183 |
| large_marker | object moved before the close | 134 |
| bleach_cleanser | object moved during the drive | 117 |
| bleach_cleanser | no grasp plan | 117 |
| plate | object lost during the lift | 116 |
| plate | no parking pose | 115 |
| mustard_bottle | object moved during the drive | 101 |
| master_chef_can | object moved during the drive | 98 |
| mug | object moved during the drive | 97 |
| bowl | no grasp candidates | 82 |
| plate | dragged to the edge | 82 |
| strawberry | object lost during the carry | 77 |
| large_marker | object moved during the drive | 75 |
| master_chef_can | no parking pose | 73 |
| large_marker | no grasp candidates | 65 |
| plate | no grasp candidates | 51 |
| hammer | object lost during the carry | 50 |
| large_marker | no grasp plan | 42 |
| mug | object lost during the carry | 42 |
| mug | no parking pose | 40 |
| plate | arm did not reach the end of the transit | 37 |
| mug | object lost while taking the travel pose | 33 |
| apple | object moved before the close | 33 |
| bleach_cleanser | object lost during the carry | 32 |
| plate | no grasp plan | 31 |
| mug | object lost during the lift | 30 |
| bleach_cleanser | object lost while taking the travel pose | 30 |
| master_chef_can | object lost during the carry | 29 |
| master_chef_can | object lost during the lift | 27 |
| apple | object lost during the carry | 26 |
| hammer | object lost while taking the travel pose | 26 |
| plate | object lost during the carry | 26 |
| apple | object moved during the drive | 26 |
| strawberry | object lost while taking the travel pose | 24 |
| master_chef_can | object moved before the close | 23 |
| plate | hold not verified after the close | 22 |
| bowl | object lost during the carry | 18 |
| plate | no clear route to the goal | 17 |
| tomato_soup_can | no parking pose | 16 |

| Travel pose | Way in | Carries | Not lost | Lost taking the pose | Lost in the carry | Lost otherwise |
|---|---|---|---|---|---|---|
| level, finger under | straight | 687 | 484 (70 %) | 82 | 120 | 1 |
| down, pads along | straight | 314 | 194 (62 %) | 17 | 103 | 0 |
| level, finger under | joint by joint | 248 | 192 (77 %) | 11 | 45 | 0 |
| level | straight | 95 | 69 (73 %) | 15 | 11 | 0 |
| folded | straight | 80 | 37 (46 %) | 18 | 16 | 9 |
| level | joint by joint | 67 | 64 (96 %) | 1 | 2 | 0 |
| level (low) | the move, then the turn | 63 | 61 (97 %) | 0 | 2 | 0 |
| level, finger under (low) | the move, then the turn | 51 | 39 (76 %) | 4 | 8 | 0 |
| level, finger under | the move, then the turn | 47 | 34 (72 %) | 3 | 10 | 0 |
| as lifted | straight | 46 | 11 (24 %) | 0 | 9 | 26 |
| folded (low) | straight | 38 | 16 (42 %) | 2 | 14 | 6 |
| folded | joint by joint | 18 | 11 (61 %) | 0 | 7 | 0 |
| level, finger under (low) | straight | 13 | 9 (69 %) | 0 | 4 | 0 |
| level, finger under (low) | joint by joint | 8 | 5 (62 %) | 2 | 1 | 0 |
| level (low) | straight | 5 | 4 (80 %) | 0 | 1 | 0 |
| level | the move, then the turn | 3 | 3 (100 %) | 0 | 0 | 0 |
| folded (low) | the move, then the turn | 2 | 1 (50 %) | 0 | 1 | 0 |
| folded | the move, then the turn | 2 | 0 (0 %) | 2 | 0 | 0 |
| down, pads along | the move, then the turn | 2 | 0 (0 %) | 1 | 1 | 0 |
| down, pads along (low) | straight | 2 | 1 (50 %) | 0 | 1 | 0 |
| down, pads along (low) | joint by joint | 1 | 0 (0 %) | 0 | 1 | 0 |
| level (low) | joint by joint | 1 | 0 (0 %) | 0 | 1 | 0 |
