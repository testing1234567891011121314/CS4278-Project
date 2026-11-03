import numpy as np
from pick_nav_reach_env import PickNavReachEnv

env = PickNavReachEnv(seed=42, gui=True)
obs = env.reset()
for _ in range(200):
    obs, _ = env.step(np.zeros(env.action_size))
pts = np.asarray(obs["object_pc"], float); nrm = np.asarray(obs["object_normals"], float)
nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)

def cascade(outer_tol, mu, plane_tol, level_abs):
    diff = pts[:, None, :] - pts[None, :, :]
    dist = np.linalg.norm(diff, axis=2)
    outv = ((pts[None, :, :] - pts[:, None, :]) * nrm[:, None, :]).sum(2).max(1)
    shell = outv <= outer_tol
    valid = (dist > 0.015) & (dist <= 0.08) & shell[:, None] & shell[None, :]
    valid &= (nrm @ nrm.T) <= -np.cos(np.radians(35))
    ax = diff / np.maximum(dist, 1e-12)[:, :, None]
    cone = np.cos(np.arctan(mu))
    n1d = (nrm[:, None, :] * ax).sum(2); n2d = -(nrm[None, :, :] * ax).sum(2)
    valid &= (n1d >= cone) & (n2d >= cone)
    n1g = np.abs((nrm[:, None, :] * diff).sum(2)); n2g = np.abs((nrm[None, :, :] * diff).sum(2))
    valid &= (np.abs(n1g - dist) < plane_tol) & (np.abs(n2g - dist) < plane_tol)
    valid &= np.abs(diff[:, :, 2]) <= level_abs
    i, j = np.triu_indices(len(pts), k=1)
    sel = valid[i, j]
    return i[sel], j[sel], dist

for ot in (0.01, 0.02, 0.03):
    for mu in (0.5, 0.7, 1.0):
        i, j, dist = cascade(ot, mu, 0.015, 0.008)
        # straddling check: pairs whose midpoint is near the object center in x
        good = np.abs((pts[i][:, 0] + pts[j][:, 0]) / 2) < 0.015 if i.size else np.zeros(0, bool)
        print(f"outer_tol={ot} mu={mu}: pairs={i.size} straddle-x={int(good.sum())}", flush=True)
        if i.size and good.any():
            k = np.nonzero(good)[0][0]
            a, b = i[k], j[k]
            print(f"   e.g. p1={np.round(pts[a],3)} n1={np.round(nrm[a],2)} "
                  f"p2={np.round(pts[b],3)} n2={np.round(nrm[b],2)} d={dist[a,b]:.3f}", flush=True)
import pybullet as p
p.disconnect(physicsClientId=env.pb_physics_client)
