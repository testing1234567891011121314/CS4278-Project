"""Bounded smoke test: sampler on a synthetic box, IK path on Fetch + cube."""
import numpy as np
import pybullet as p
import pybullet_data

from custom_libraries.antipodal import AntipodalGraspPlanner

# --- 1. Sampler: 6x4x2 cm box, 2 mm grid on the surface -------------------
def box_cloud(lx, ly, lz, n=10):
    xs = np.linspace(-lx / 2, lx / 2, n)
    ys = np.linspace(-ly / 2, ly / 2, n)
    zs = np.linspace(-lz / 2, lz / 2, n)
    pts, nrm = [], []
    for ax, normal in ((0, [1, 0, 0]), (1, [0, 1, 0]), (2, [0, 0, 1])):
        grid2 = np.array(np.meshgrid(*([ys, zs] if ax == 0 else [xs, zs] if ax == 1 else [xs, ys]), indexing="ij")).reshape(2, -1).T
        grid = np.zeros((len(grid2), 3))
        a, b = (1, 2) if ax == 0 else (0, 2) if ax == 1 else (0, 1)
        grid[:, a] = grid2[:, 0]; grid[:, b] = grid2[:, 1]
        face = grid.copy()
        face[:, ax] = [lx, ly, lz][ax] / 2
        pts.append(face); nrm.append(np.tile(normal, (len(face), 1)))
        face2 = face.copy(); face2[:, ax] *= -1
        pts.append(face2); nrm.append(np.tile(-np.array(normal), (len(face2), 1)))
    return np.vstack(pts), np.vstack(nrm)

planner = AntipodalGraspPlanner(gripper_max_width=0.08)
pts, nrm = box_cloud(0.06, 0.04, 0.02)
cands = planner.sample_antipodal_grasps(points=pts, normals=nrm)
print(f"box 6x4x2cm: {len(cands)} candidates")
assert len(cands) >= 5, "expected diverse candidates"
scores = [c[4] for c in cands]
assert scores == sorted(scores), "must be best-first"
centers = np.array([c[2] for c in cands])
d = np.linalg.norm(centers[:, None] - centers[None, :], axis=2)
np.fill_diagonal(d, 1e9)
print(f"min center separation: {d.min():.4f} (>=0.03*expected)")
widths = [np.linalg.norm(c[1] - c[0]) for c in cands]
print(f"widths: min {min(widths):.3f} max {max(widths):.3f}")
assert all(0.015 <= w <= 0.08 for w in widths)

# cylinder: top-down AND side grasps should both appear
th = np.linspace(0, 2 * np.pi, 60, endpoint=False)
cz = np.linspace(-0.03, 0.03, 20)
T, Z = np.meshgrid(th, cz)
wall = np.stack([0.025 * np.cos(T).ravel(), 0.025 * np.sin(T).ravel(), Z.ravel()], 1)
wn = np.stack([np.cos(T).ravel(), np.sin(T).ravel(), np.zeros_like(T).ravel()], 1)
cap_pts, cap_n = [], []
for sgn in (-1, 1):
    r = np.sqrt(np.linspace(0, 0.025 ** 2, 15))
    Rg, Tg = np.meshgrid(r, th)
    cap = np.stack([Rg.ravel() * np.cos(Tg.ravel()), Rg.ravel() * np.sin(Tg.ravel()), np.full(Rg.size, sgn * 0.03)], 1)
    cap_pts.append(cap); cap_n.append(np.tile([0, 0, sgn], (len(cap), 1)))
cyl_pts = np.vstack([wall] + cap_pts); cyl_n = np.vstack([wn] + cap_n)
cyl = planner.sample_antipodal_grasps(points=cyl_pts, normals=cyl_n)
if cyl:
    axis_z = sum(abs(c[1][2] - c[0][2]) / np.linalg.norm(c[1] - c[0]) > 0.9 for c in cyl)
    print(f"cylinder: {len(cyl)} candidates, {axis_z} vertical-axis (side) grasps")
    assert axis_z > 0 and len(cyl) - axis_z > 0, "want both side and top-down"

# --- 2. IK path: Fetch + cube in DIRECT ------------------------------------
cid = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
p.setGravity(0, 0, -9.81, physicsClientId=cid)
plane = p.loadURDF("plane.urdf", physicsClientId=cid)
robot = p.loadURDF("assets/fetch/fetch.urdf", [0, 0, 0], physicsClientId=cid)
gripper = [i for i in range(p.getNumJoints(robot, physicsClientId=cid))
           if p.getJointInfo(robot, i, physicsClientId=cid)[12] == b"gripper_link"][0]
obj = p.loadURDF("assets/ycb_objects/002_master_chef_can/coacd_decomposed_object_one_link.urdf",
                 [0.6, 0, 0.06], physicsClientId=cid)

pc = np.array(p.getContactPoints(bodyA=obj, physicsClientId=cid) and 0 or 0)
rng = np.random.default_rng(1)
pts_o, nrm_o = [], []
for _ in range(400):
    ray_from = [0.6 + rng.uniform(-.1, .1), rng.uniform(-.1, .1), 0.5]
    to = [0.6, 0, 0.06]
    hits = p.rayTestBatch([ray_from], [to], physicsClientId=cid)
    if hits[0][0] == obj:
        loc = hits[0][3]
        n = p.getNormalAtWorldPosition(obj, loc, physicsClientId=cid)
        nrm_o.append(n[0]); pts_o.append(loc)
pts_o, nrm_o = np.array(pts_o), np.array(nrm_o)
print(f"can cloud: {len(pts_o)} pts")
cands = planner.sample_antipodal_grasps(points=pts_o, normals=nrm_o)
print(f"can candidates: {len(cands)}")
best = planner.select_optimal_grasp(robot, gripper, cands, strategy="score_all")
print("score_all:", "FOUND" if best else "None", planner.ik_stats)
best2 = planner.select_optimal_grasp(robot, gripper, cands, strategy="greedy_first")
print("greedy_first:", "FOUND" if best2 else "None", planner.ik_stats)
if best:
    assert len(best["joint_angles"]) == 7
    print("arm joints:", np.round(best["joint_angles"], 3))
    print("ee_target:", np.round(best["ee_target"], 3))
p.disconnect(physicsClientId=cid)
print("SMOKE OK")
