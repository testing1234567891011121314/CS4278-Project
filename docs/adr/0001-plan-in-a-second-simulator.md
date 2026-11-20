# Plan in a second simulator, never write to the live one

The old solver planned inside the env's own simulator: it teleported joints to
test poses and wrapped each planning call in `saveState`/`restoreState`.
Restoring is not neutral. It perturbs the contacts of resting objects, which
flipped marginal grasps (bowl rim, apple) between delivered and lost and
kicked the wedged marker 17 mm. The rewrite therefore never writes to the
env's simulator (R30) and answers every planning query in a second PyBullet
client that holds a copy of the scene and is set to the observed state before
each planning call (R31).

## Considered options

- **`restoreState` with a hand-written write-back of poses and velocities.**
  Tried in the old solver; outcomes still flipped, in both directions.
- **Our own geometric collision checker, no simulator.** Would need the
  convex pieces of the robot and of 15 objects handled in our code; far more
  work for the same answers.

## Consequences

Measured on 2026-11-20: the second client adds 286 MB to the live env's
986 MB, loads in 1.2 s, and checks one robot pose against the table, object
and maze in 0.26 ms. A trial therefore peaks near 1.3 GB, inside the 2 GB cap
that simulator runs use on the 6 GB machine.
