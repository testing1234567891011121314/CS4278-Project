# Navigation tests

Extract navigation_tests into the project root beside solver.py.
Run from that root:

    python -m unittest discover -s navigation_tests -v

Dependencies: numpy and the production navigation modules. unittest is built in.
The suite expects custom_libraries/a_star.py and
custom_libraries/curved_navigator.py. If that package is absent it also supports
the originally supplied standalone a_star.py and curved_navigation.py.
It does not include or overwrite your production modules.

In solver.py, import the callable explicitly:

    from custom_libraries.a_star import a_star
    from custom_libraries import curved_navigator

For a different solver filename, in PowerShell:

    $env:NAV_SOLVER_FILE = "path/to/solver.py"
    python -m unittest discover -s navigation_tests -v

## Coverage

25 test methods, including 40 seeded maps inside the Dijkstra comparison test:
- Straight/diagonal optimal costs, corner cutting, blocked endpoints, map bounds,
  disconnected goals, identical start/goal, unchanged input map.
- Independent Dijkstra oracle checks optimal A* path cost and valid edges.
- Open and obstructed corner rounding, dense analytic curve sampling,
  exact corner touches, tiny obstacle crossings, map-boundary rejection.
- Invalid diagonal/start connections, empty paths, already-reached goal.
- Position-feedback progress, no progress without actual movement,
  safe target chords, replanning, lateral correction in open space.
- A* to curve follower integration around a wall.
- Actual solver get_action method: 15-element position actions, preservation of
  non-base targets, and returned array ownership.
- Actual solver import: a_star must be callable, not a module.

The previously run small smoke checks are consolidated into this suite.

## What was run here

The supplied a_star.py and curved_navigation.py implementations were used,
with the latter named custom_libraries/curved_navigator.py for package tests.
custom_libraries/__init__.py was empty. With the latest uploaded solver,
24 tests passed and the callable-import test failed. With only its A* import
changed to the explicit function import, all 25 passed.
If your __init__.py already re-exports the a_star function, the original import
may work; the explicit function import avoids that ambiguity.

## Limits

These are synthetic geometry and controller tests, NOT a PyBullet scene run.
The tracking tests use a simple partial-position-response model without inertia,
contacts, or motor dynamics. For action tests, only the real get_action method
is executed against test doubles; Solver.__init__ and geometry queries are not
executed. The import test uses a placeholder pybullet module only when PyBullet
is not installed; no physics world is created.

These tests do not verify Fetch mesh coordinate conventions, radius accuracy,
actual obstacle identification, collision margins, arm/hand clearance, stopping
with real velocities, grasping, or success on hidden evaluation scenes.

For physical validation, run your normal environment on multiple supported
seeds/objects, record navigation status, base positions/velocities, and allowed
read-only contact queries. Do not reset bodies or alter collision properties
inside the planner. Report base-goal arrival separately from object-task success.
