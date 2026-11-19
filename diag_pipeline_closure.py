"""Run the real pipeline but log the first rejecting contact for each
candidate-stage _valid failure during state-1 planning."""
import contextlib
import io
from collections import Counter
import numpy as np
import pybullet as p

p.loadPlugin = lambda *a, **k: 0
_buf = io.StringIO()
with contextlib.redirect_stdout(_buf):
    from pick_nav_reach_env import PickNavReachEnv
    from custom_libraries import grasp as G
    from solver import Solver

LOG = []
STAGE = ["?"]


def main():
    env = PickNavReachEnv(seed=42, gui=False)
    env.reset()
    for _ in range(200):
        env.step(np.zeros(env.action_size))
    solver = Solver(env)

    planner = solver.grasp_planner
    orig_valid = G.GraspPlanner._valid

    def logging_valid(self, q, *, touch=False, attached=None, deadline=float("inf")):
        ok = orig_valid(self, q, touch=touch, attached=attached, deadline=deadline)
        if not ok and len(LOG) < 400:
            cfg = self.config
            who = None
            if not np.isfinite(q).all() or np.any(q < self.lower - 1e-3) \
                    or np.any(q > self.upper + 1e-3):
                who = ("limits",)
            else:
                self._set(q)
                if attached is not None:
                    ee = p.getLinkState(self.robot, self.ee,
                                        computeForwardKinematics=True,
                                        physicsClientId=self.client)
                    pos, orn = p.multiplyTransforms(ee[4], ee[5], *attached)
                    p.resetBasePositionAndOrientation(self.object, pos, orn,
                                                      physicsClientId=self.client)
                for body in self.obstacles:
                    for c in p.getClosestPoints(self.robot, body,
                                                cfg.collision_margin,
                                                physicsClientId=self.client):
                        if c[3] not in self.moving_links and not (
                                attached is not None and body == self.object):
                            continue
                        if (body == self.object and touch
                                and c[3] in self.finger_ids
                                and c[8] >= -cfg.contact_penetration):
                            continue
                        who = ("body", c[3], round(float(c[8]), 4), touch)
                        break
                    if who:
                        break
                if who is None:
                    for c in p.getClosestPoints(self.robot, self.robot,
                                                cfg.collision_margin,
                                                physicsClientId=self.client):
                        if frozenset((c[3], c[4])) in self.self_pairs:
                            who = ("self", c[3], c[4], round(float(c[8]), 4), touch)
                            break
            LOG.append((STAGE[0], who, q.copy() if who and who[0] == "self" else None))
        return ok

    G.GraspPlanner._valid = logging_valid

    last_state = None
    for i in range(900):
        action = solver.get_action(env)
        obs, info = env.step(action)
        if solver.state != last_state:
            print(f">>> step {i}: state {last_state} -> {solver.state}", flush=True)
            if solver.state == 1:
                LOG.clear()
                STAGE[0] = "attempt"
            last_state = solver.state
        if info.get("success"):
            print("SUCCESS", flush=True)
            break
        if solver.state == 1 and getattr(solver, "grasp_follower", None):
            break

    names = {j: planner.info[j][12].decode() for j in planner.info}
    names[-1] = "base"
    cnt = Counter()
    for stage, w, _q in LOG:
        if w is None:
            cnt[("none",)] += 1
        elif w[0] == "limits":
            cnt[("limits",)] += 1
        elif w[0] == "body":
            cnt[("body", names.get(w[1], w[1]), w[3])] += 1
        else:
            cnt[("self", names.get(w[1], w[1]), names.get(w[2], w[2]))] += 1
    for k, n in cnt.most_common(15):
        print("REJ", n, k)
    print("total logged:", len(LOG))
    for stage, w, q in LOG:
        if q is not None:
            print("REJECTED arm:", np.round(q[6:13], 3))
            print("REJECTED torso/fingers:", np.round(q[[3, 13, 14]], 4))
            planner._set(q)
            link_ids = {planner.info[j][12].decode(): j for j in planner.info}
            for a, b in [("torso_lift_link", "wrist_flex_link"),
                         ("torso_lift_link", "forearm_roll_link"),
                         ("torso_lift_link", "elbow_flex_link"),
                         ("torso_lift_link", "upperarm_roll_link")]:
                for c in p.getClosestPoints(planner.robot, planner.robot, 0.1,
                                            physicsClientId=planner.client):
                    an = names.get(c[3], str(c[3]))
                    bn = names.get(c[4], str(c[4]))
                    if {an, bn} == {a, b}:
                        print(f"DIST {a} ~ {b}: {c[8]:.4f}")
                        break
            obs2 = env._get_obs()
            print("base xy:", np.round(obs2["qpos"][:2], 3),
                  "object:", np.round(obs2["object_pos"], 3))
            break


if __name__ == "__main__":
    main()
