"""Why does no grasp candidate fit the plate? Prints, for the candidates found
on the cloud, how far the open hand is from the object and from the table.

    pnr/bin/python tools/experiments/plate_candidates.py [OBJECT=11]
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from trial import close_env, make_env, quiet

from custom_libraries.collision import CollisionChecker
from custom_libraries.grasping.candidates import antipodal_candidates
from custom_libraries.planning_sim import PlanningSim
from custom_libraries.world import ObjectState, World

env = make_env(int(sys.argv[1]) if len(sys.argv) > 1 else 11, 42, 300)
with quiet():
    for _ in range(120):
        obs, _ = env.step(env._get_obs()["qpos"])
sim, world, obj = PlanningSim(env), World.from_env(env, obs), ObjectState.from_obs(obs)
collision = CollisionChecker(sim)
sim.set_object(obj.position, obj.orientation)
collision.focus(obj.centre[:2])
support = world.support_under(np.array([*obj.centre[:2], obj.cloud[:, 2].max()]))
found = antipodal_candidates(obj, sim.gripper, support, limit=400)
print(f"cloud z {obj.cloud[:, 2].min():.4f}..{obj.cloud[:, 2].max():.4f}, support {support:.4f}, {len(found)} candidates")
rows = []
for c in found:
    position, turn = c.gripper_pose()
    to_object, to_scene = sim.hand_clearance(position, turn, c.opening, collision.near)
    rows.append((to_object, to_scene, c))
print("fit both: %d   object only: %d   scene only: %d" % (
    sum(o >= 0.0015 and s >= 0.002 for o, s, _ in rows), sum(o >= 0.0015 for o, s, _ in rows),
    sum(s >= 0.002 for o, s, _ in rows)))
for o, s, c in sorted(rows, key=lambda r: -min(r[0], r[1]))[:12]:
    print(f"  {c.kind:9s} object {1000 * o:6.1f} mm  table {1000 * s:6.1f} mm  width {1000 * c.width:5.1f}  opening {1000 * c.opening:5.1f}  "
          f"offset {1000 * c.offset:4.1f}  height {1000 * (c.centre[2] - support):5.1f} mm  r {1000 * np.linalg.norm(c.centre[:2] - obj.centre[:2]):5.1f} mm  "
          f"closing {np.round(c.closing_axis, 2)}  approach {np.round(c.approach_axis, 2)}")
sim.close()
close_env(env)
