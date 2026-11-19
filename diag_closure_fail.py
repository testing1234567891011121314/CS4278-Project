"""Why does the reference planner reject every candidate at closure?

Monkeypatches GraspPlanner._valid to log the first rejecting contact
whenever touch=True (only the closure/lift sweeps use touch), and prints
scene geometry for sanity.
"""
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0

from pick_nav_reach_env import PickNavReachEnv
from custom_libraries.reference import grasp as G

TRAVEL = {6: 0.0, 7: -1.0, 8: 0.0, 9: -2.0, 10: 0.0, 11: 1.0, 12: 0.0}
LOG = []


def patched(self, q, *, touch=False, attached=None, deadline=float("inf")):
    cfg = self.config
    if (not np.isfinite(q).all() or np.any(q < self.lower - 1e-8)
            or np.any(q > self.upper + 1e-8)):
        return False
    self._set(q)
    if attached is not None:
        raise RuntimeError("lift reached; not expected in this diag")
    for body in self.obstacles:
        for c in p.getClosestPoints(self.robot, body, cfg.collision_margin,
                                    physicsClientId=self.client):
            if c[3] not in self.moving_links and not (attached is not None and body == self.object):
                continue
            if body == self.object and touch and c[3] in self.finger_ids:
                if c[8] >= -cfg.contact_penetration:
                    continue
            if touch:
                LOG.append(("obstacle", body, c[3], round(float(c[8]), 4)))
            return False
    for c in p.getClosestPoints(self.robot, self.robot, cfg.collision_margin,
                                physicsClientId=self.client):
        if frozenset((c[3], c[4])) in self.self_pairs:
            if touch:
                LOG.append(("self", c[3], c[4], round(float(c[8]), 4)))
            return False
    return True


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    obs = env.reset()
    for _ in range(150):
        obs = env._get_obs()
        env.step(np.asarray(obs["qpos"], float))
    o, _ = p.getBasePositionAndOrientation(env.object_id,
                                           physicsClientId=env.pb_physics_client)
    print("object at:", np.round(o, 3))
    for _ in range(120):
        obs = env._get_obs()
        qpos = np.asarray(obs["qpos"], float)
        target = qpos.copy()
        for i, v in TRAVEL.items():
            target[i] = v
        err = target - qpos
        if np.max(np.abs(err[[6, 7, 8, 9, 10, 11, 12]])) < 0.02:
            break
        env.step(np.clip(qpos + np.clip(err, -0.05, 0.05),
                         env.joint_lower, env.joint_upper))

    planner = G.GraspPlanner(env, robot_urdf_path="assets/fetch/fetch.urdf")
    G.GraspPlanner._valid = patched
    obs = env._get_obs()
    plan = planner.plan(obs)
    print("plan:", "found" if plan else "None", planner.last_stats)
    from collections import Counter
    names = {j: planner.info[j][12].decode() for j in planner.info}
    names[-1] = "base"
    body_names = {}
    for i in range(p.getNumBodies(physicsClientId=planner.client)):
        b = p.getBodyUniqueId(i, physicsClientId=planner.client)
        body_names[b] = p.getBodyInfo(b, physicsClientId=planner.client)[0].decode()
    counted = Counter()
    for rec in LOG:
        if rec[0] == "obstacle":
            _, body, lk, d = rec
            counted[(body_names.get(body, body), names.get(lk, lk))] += 1
        else:
            _, a, b, d = rec
            counted[("self", f"{names.get(a,a)}|{names.get(b,b)}")] += 1
    for k, n in counted.most_common(12):
        print(n, k)


if __name__ == "__main__":
    main()
