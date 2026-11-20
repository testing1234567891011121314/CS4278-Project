# Collecting training data for a learned grasp ranker

Goal: one row per trial with the features of the grasp candidate that was
executed (known at planning time) and the outcome of the whole episode
(lifted, held, delivered, dropped, slid in the grip). A model trained on these
rows scores a candidate by the probability that the object survives to
delivery. No model is trained here.

The planner normally executes the best-scoring feasible candidate, so a
score-order run only shows what happens to the candidates the current score
already likes. Exploration runs try the first pass's top K candidates in a
seeded random order instead, so lower-ranked candidates get executed too.

## Settings

Both are `Solver` settings, so `SOLVER_SET` reaches them through
`tools/batch.py` and `tools/score.py`:

| setting | default | meaning |
|---|---|---|
| `grasp_explore` | `0` | 0: candidates in score order (unchanged behaviour). K > 0: the first pass's top K antipodal candidates are tried in a random order, the rest (up to `max_candidates` = 60) in score order after them. The shuffle groups near-duplicates into families (contact centre rounded to 1 cm plus the unsigned grasp line rounded to 0.25, so the roll and closing-direction variants of one contact pair are one family), shuffles the families and the members of each, then takes them round robin (every family's first member, then every second member...). The pre-check fallback pass and the rim (bowl) and flange (plate) candidates are never shuffled. |
| `grasp_explore_seed` | `-1` | Seed of the shuffle. Negative: `tools/score.py` uses `crc32("OBJ:SEED:REPEAT")`, so repeats of one object and seed differ and any trial can be replayed with the same shuffle; outside `score.py` (e.g. `run.sh`) the planner draws fresh entropy. The seed used is logged as `g_explore_seed`. |

The shuffle seed depends only on object, seed and repeat. Two machines that
run the same objects, seeds and repeats therefore execute the same shuffles:
give each machine its own seed list (below), not the same list.

## Collection run

One third in score order (the baseline, and the only data on the candidates
the current score picks), two thirds with exploration. Example for all 15
objects with seeds split over two machines (A: seeds 1-10, B: seeds 11-20);
use `-j` = number of cores, about 1 GB of memory per job:

```bash
cd /path/to/CS4278-Project
PY=pnr/bin/python
OBJ="0 1 2 3 4 5 6 7 8 9 10 11 12 13 14"
SEEDS="1 2 3 4 5 6 7 8 9 10"          # machine B: 11 ... 20

# 1/3: score order (1 repeat)
$PY tools/batch.py runs/ranker_A_base --objects $OBJ --seeds $SEEDS -j 16
# 2/3: exploration over the top 40 (2 repeats -> 2 different shuffles per trial)
SOLVER_SET="grasp_explore=40" $PY tools/batch.py runs/ranker_A_explore \
    --objects $OBJ --seeds $SEEDS --repeat 2 -j 16
```

Keep the default `--max-steps` (15000): the label is delivery. An interrupted
batch continues with the same command plus `--resume`. `batch.py` copies the
code to `OUT_DIR/snap` first; delete `snap/` when the batch is done (the logs
and `results.csv` are all that is needed). `tools/batch.py OUT_DIR --report`
rewrites `results.csv` from the logs.

## Columns of results.csv

The existing columns (object ... `wall_s`, `pts_*`) are unchanged and come
first; the new ones are appended. A trial whose grasp planning failed has no
`g_*` values; a trial that never lifted has no `slide_*` values. Empty cells
or `nan` mean "not defined" (e.g. no cloud point under a pad).

### Executed candidate: how it was chosen

| column | meaning |
|---|---|
| `g_rank` | index (0 = best) of the executed candidate in the sampler's score order of the planning attempt that succeeded; empty for a candidate added by the pre-check retry sampling |
| `g_order` | its index in the order actually tried (differs from `g_rank` under exploration or in the pre-check pass) |
| `g_tried_before` | candidates tried (and rejected) before it in that attempt |
| `g_pass` | `normal` (sampler order, first pass) or `precheck` (fallback pass with the pre-check ordering) |
| `g_source` | `normal`, `precheck`, `precheck_retry` (added by the 2000-sample retry), `rim` (bowl fallback) or `flange` (plate fallback) |
| `g_family` | index of the candidate's near-duplicate family, families numbered in score order of their best member (0 = family of the top candidate) |
| `g_family_key` | the family key: contact centre / 1 cm (3 ints) and unsigned grasp line / 0.25 (3 ints) |
| `g_family_size` | candidates of the sampler's list in that family |
| `g_explore` | K used for this attempt (0: score order) |
| `g_explore_seed` | shuffle seed (empty when not exploring) |
| `g_tcp_attempts` | planning attempts made (depth x pass): 1 = first depth of the first pass |
| `g_mass_kg` | object mass (kg), read by `score.py` from PyBullet |

### Executed candidate: features (`GraspPlanner.candidate_features`)

Pure geometry from the observed cloud, normals, centre of mass and the
candidate at the grasp depth it was planned with. For rim (bowl) and flange
(plate) candidates the contacts are constructed, not cloud points, so
`g_quality` and the pad features are approximate (a flange candidate's 5 mm
nominal width usually leaves no cloud under the pads: coverage 0, spread
`nan`); filter on `g_source` if that matters.

| column | unit | meaning |
|---|---|---|
| `g_depth_mm` | mm | grasp depth: TCP offset of the contact centre ahead of the pad centre (10 deep, 25 shallow; flange 25) |
| `g_tilt_deg` | deg | approach axis from straight down (0 = top-down, 90 = horizontal) |
| `g_closing_deg` | deg | closing axis from horizontal (90 = vertical pinch) |
| `g_roll_deg` | deg | roll variant: approach rotated about the grasp line from the sampler's base approach (0, ±30, ±90 for antipodal candidates) |
| `g_width_mm` | mm | distance between the two contacts |
| `g_margin_mm` | mm | maximum opening (closed gap + 2 × finger travel = 102.85 mm) minus the width |
| `g_centre_h_mm` | mm | contact centre above the lowest cloud point |
| `g_centre_h_frac` | - | the same as a fraction of the cloud height |
| `g_com_dist_mm` | mm | perpendicular distance from the grasp line to the centre of mass |
| `g_com_dz_mm` | mm | its vertical component, signed (+: centre of mass above the grasp line) |
| `g_com_dxy_mm` | mm | its horizontal component |
| `g_torque_arm_mm` | mm | gravity's moment about the grasp line per unit weight, \|((com − centre) × down) · line\|: what turns the object about the closing axis (0 for a vertical closing axis) |
| `g_quality` | - | friction-cone quality min(−n₁·d, n₂·d) from the normals of the cloud points nearest the contacts (exact for antipodal candidates; ≥ 0.894 passes the cone) |
| `g_score` | - | the sampler's final score (the order the planner uses) |
| `g_pad_cov_min`, `g_pad_cov_max` | - | per pad, fraction of a 9 × 4 grid over the 63 × 24 mm pad footprint with a cloud point within 3 mm of the pad plane; the smaller and larger of the two pads |
| `g_pad_spread_min_deg`, `g_pad_spread_max_deg` | deg | per pad, mean angle between the normals of those points and their mean normal (≈0 on a flat face, larger on curved surfaces and edges) |
| `g_pad_pts_min` | points | the fewer of the two pads' patch point counts |
| `g_finger_low_mm` | mm | lowest finger-box corner at the grasp pose above the lowest cloud point (negative: below the support) |
| `g_palm_low_mm` | mm | lowest palm-box corner, likewise |
| `g_flag_tight`, `g_flag_palm`, `g_flag_support` | 0/1 | the planner's pre-check flags (opening clearance < 16 mm; cloud inside the palm box; a finger or the palm below the support) |
| `g_ext1_mm`, `g_ext2_mm`, `g_ext3_mm` | mm | object (cloud) extents along its principal axes, largest first |
| `g_height_mm` | mm | cloud height (highest minus lowest point) |

### Slide in the grip (labels)

The object's pose in the gripper frame, relative to its value at the end of
the lift (the step the solver enters state 3), sampled on steps with both
fingers in contact, at the last such step of each phase the trial reaches.

| column | unit | meaning |
|---|---|---|
| `slide_hold_x_mm` | mm | displacement along the gripper x axis (toward the fingertips: + = sliding out) at the end of the hold (state 3) |
| `slide_hold_rot_deg` | deg | total rotation relative to the gripper at that time |
| `slide_hold_pivot_deg` | deg | its component about the closing axis (gripper y), signed: the pivot gravity drives |
| `slide_retreat_*` | | the same at the end of the retreat (state 5) |
| `slide_setup_*` | | the same at the end of the yaw-carry setup (wrist roll and turn on the spot) |
| `slide_end_*` | | the same at the last step with both fingers in contact: just before a drop, at delivery, or when the trial stopped |
| `slide_max_x_mm` | mm | largest \|displacement along x\| over the held steps |
| `slide_max_rot_deg` | deg | largest rotation over the held steps |

Outcome labels already in the CSV: `lift_step` (lifted at all), `hold_50`,
`success_step` (delivered), `drop_step` / `drop_state` (where it was lost),
`stopped`, `total`.

## Concatenating results from several machines

Copy each `results.csv` (rename it after its run, e.g.
`ranker_A_base.csv`), then:

```bash
python - ranker_*.csv > ranker_all.csv <<'EOF'
import csv, sys
rows, cols = [], []
for path in sys.argv[1:]:
    for r in csv.DictReader(open(path)):
        if r.get("name") == "CRASH":
            continue
        r["run"] = path.rsplit("/", 1)[-1][:-4]
        rows.append(r)
        cols += [c for c in r if c not in cols]
w = csv.DictWriter(sys.stdout, cols)
w.writeheader()
w.writerows(rows)
EOF
```

Columns are matched by name, so files whose `g_*` columns appear in a
different order (or are missing, e.g. a run where no plan succeeded) still
line up. The `run` column records which file a row came from; with
`g_explore` it separates baseline and exploration rows. CRASH rows (a trial
whose log has no SCORE line) are dropped.
