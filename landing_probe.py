import numpy as np
import pybullet as p
from pick_nav_reach_env import PickNavReachEnv

env = PickNavReachEnv(seed=42, gui=True)
for _ in range(200):
    env.step(np.zeros(env.action_size))
obs = env._get_obs()
points = np.asarray(obs["object_pc"], dtype=float)
normals = np.asarray(obs["object_normals"], dtype=float)
normals = normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)

ext = points.max(0) - points.min(0)
beyond = (points[None, :, :] - points[:, None, :])
outward = (beyond * normals[:, None, :]).sum(2).max(1)
outer_shell = outward <= 0.03

diff = points[:, None, :] - points[None, :, :]
dist = np.linalg.norm(diff, axis=2)
n_dot = normals @ normals.T
cone_cos = np.cos(np.arctan(0.5))
axis = diff / np.maximum(dist, 1e-12)[:, :, None]
n1_dir = (normals[:, None, :] * axis).sum(2)
n2_dir = -(normals[None, :, :] * axis).sum(2)
n1_gap = np.abs((normals[:, None, :] * diff).sum(2))
n2_gap = np.abs((normals[None, :, :] * diff).sum(2))

m_dist = (dist > 0.015) & (dist <= 0.09)
m_shell = m_dist & outer_shell[:, None] & outer_shell[None, :]
m_anti = m_shell & (n_dot <= -np.cos(np.radians(35.0)))
m_cone = m_anti & (n1_dir >= cone_cos) & (n2_dir >= cone_cos)
m_plane = m_cone & (np.abs(n1_gap - dist) < 0.015) & (np.abs(n2_gap - dist) < 0.015)

ii, jj = np.triu_indices(len(points), k=1)
sel = m_plane[ii, jj]
pairs_i, pairs_j = ii[sel], jj[sel]
d = dist[pairs_i, pairs_j]
dz = np.abs(diff[pairs_i, pairs_j, 2])
z1, z2 = points[pairs_i, 2], points[pairs_j, 2]
horiz = np.linalg.norm(diff[pairs_i, pairs_j][:, :2], axis=1)
print("m_plane pairs:", sel.sum())
print("dz/dist: min %.2f p50 %.2f p90 %.2f max %.2f" % tuple(
    np.percentile(dz / np.maximum(d, 1e-12), [0, 50, 90, 100])))
print("dist: min %.3f p50 %.3f max %.3f" % tuple(
    np.percentile(d, [0, 50, 100])))
print("z-range per pair (max|z1-z2|):", np.round(np.percentile(dz, [50, 90, 100]), 4))
print("horiz sep p50:", np.round(np.percentile(horiz, 50), 4),
      " vertical sep p50:", np.round(np.percentile(np.abs(z1 - z2), 50), 4))
# where do the points sit? z histogram of shell points
zs = points[outer_shell, 2]
hist, edges = np.histogram(zs, bins=8)
print("shell-point z histogram:", dict(zip(np.round(edges, 3), hist)))
# how many shell points on the SIDES (low z, between bottom and top faces)
side = outer_shell & (points[:, 2] > 0.632) & (points[:, 2] < 0.658)
print("shell points in mid-z band (sides):", side.sum())
# best same-height side pairs if we relax plane tolerance
dz_full = np.abs(diff[:, :, 2])
axis_z = np.abs(diff[:, :, 2]) / np.maximum(dist, 1e-12)
for atol in [0.1, 0.2, 0.35, 0.5]:
    v = m_plane & (axis_z <= atol)
    print(f"closing-axis-z <= {atol}: plane pairs surviving={v[ii, jj].sum()}")
# inspect plane pair axis tilt
az = axis_z[pairs_i, pairs_j]
print("plane-pair |axis.z| percentiles:", np.round(np.percentile(az, [0, 25, 50, 75, 100]), 3))

from custom_libraries.antipodal import AntipodalGraspPlanner
planner = AntipodalGraspPlanner()
cands = planner.sample_antipodal_grasps(
    env.object_id, points=points, normals=normals, num_samples=500,
    level_tol=1.0)
print("level_tol=1.0 candidates:", len(cands), planner.sample_stats)
seen = set()
for p1, p2, center, quat, score in sorted(cands, key=lambda c: c[4]):
    key = (tuple(np.round(center, 2)), tuple(np.round(np.asarray(p2) - np.asarray(p1), 2)))
    if key in seen:
        continue
    seen.add(key)
    nrm = np.asarray(p2) - np.asarray(p1)
    print(f"center {np.round(center, 3)} width {np.linalg.norm(nrm):.3f} "
          f"axis.z {nrm[2] / np.linalg.norm(nrm):+.2f} "
          f"z1 {p1[2]:.3f} z2 {p2[2]:.3f} score {score:.3f}")
