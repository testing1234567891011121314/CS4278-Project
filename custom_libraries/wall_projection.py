"""Whole-robot XY wall projection using read-only PyBullet queries.

Fixed posture/yaw only. Geometry is projected regardless of height: conservative
for finite-height walls. No carried object is included automatically. Mesh data
uses the rigid-body local-inertial convention; AABB containment is a sanity
check, not proof of that convention on every PyBullet build. Unsupported shapes
raise instead of silently omitting collision geometry. Designed for the supplied
Fetch mesh links and box fingers. NumPy is the only non-PyBullet dependency.

Run pure-geometry tests (no PyBullet required): python wall_projection.py --test
"""
from itertools import product
import numpy as np


def convex_hull(points):
    """Own 2D monotone-chain hull; separate hull for each collision part."""
    points = sorted(set(map(tuple, np.asarray(points, float))))
    if len(points) <= 1:
        return np.asarray(points).reshape(-1, 2)
    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2],lower[-1],point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2],upper[-1],point) <= 0:
            upper.pop()
        upper.append(point)
    return np.asarray(lower[:-1]+upper[:-1], float)


def hull_hits_square(hull, center, half_size):
    """Separating-axis intersection, inclusive of touching boundaries."""
    hull = np.asarray(hull)
    axes = [np.array([1.,0.]), np.array([0.,1.])]
    if len(hull) > 1:
        for edge in np.roll(hull,-1,axis=0)-hull:
            if np.linalg.norm(edge) > 1e-12:
                axes.append(np.array([-edge[1],edge[0]]))
    for axis in axes:
        values = hull @ axis
        mid = np.dot(center,axis)
        extent = half_size*np.abs(axis).sum()
        if values.max() < mid-extent-1e-10 or values.min() > mid+extent+1e-10:
            return False
    return True


def footprint_offsets(polygons, cell_size, safety_margin=0.05):
    """Return (dr, dc) offsets from base centre towards obstacle centres.

    Includes the extent of both obstacle and candidate-base cells (one cell
    width total), plus a square safety margin. Thus a free output cell is
    safe for ANY reference point in that cell, for the frozen footprint.
    This is deliberately more conservative than checking cell centres only.
    """
    if cell_size <= 0 or safety_margin < 0:
        raise ValueError('Invalid resolution or safety margin')
    half = cell_size+safety_margin
    offsets = set()
    for polygon in polygons:
        hull = convex_hull(polygon)
        if not len(hull) or not np.isfinite(hull).all():
            raise ValueError('Empty/nonfinite footprint')
        low = np.floor((hull.min(axis=0)-half)/cell_size).astype(int)
        high = np.ceil((hull.max(axis=0)+half)/cell_size).astype(int)
        for dx in range(low[0],high[0]+1):
            for dy in range(low[1],high[1]+1):
                if hull_hits_square(hull,np.array([dx,dy])*cell_size,half):
                    offsets.add((-dy,dx))  # world +Y is decreasing row
    if not offsets:
        raise ValueError('No robot collision footprint obtained')
    return sorted(offsets)


def inflate_walls(walls, offsets):
    """C-space rule: base = obstacle - footprint offset. No array wrapping."""
    walls = np.asarray(walls, bool)
    rows, cols = walls.shape
    blocked = np.zeros_like(walls)
    for dr, dc in offsets:
        # blocked[r,c] |= walls[r+dr,c+dc]
        r0,r1 = max(0,-dr),min(rows,rows-dr)
        c0,c1 = max(0,-dc),min(cols,cols-dc)
        if r0 < r1 and c0 < c1:
            blocked[r0:r1,c0:c1] |= walls[r0+dr:r1+dr,c0+dc:c1+dc]
    return blocked.astype(np.uint8)


def robot_xy_polygons(env):
    """Read robot collision geometry at its CURRENT pose; do not mutate it.

    The base reference is the physical base_link frame origin. Returned XY
    polygons keep current world yaw, with world translation removed.
    """
    import pybullet as p
    client, robot = env.pb_physics_client, env.robot_id
    names = {-1:p.getBodyInfo(robot,physicsClientId=client)[0].decode('utf-8')}
    for link in range(p.getNumJoints(robot,physicsClientId=client)):
        names[link] = p.getJointInfo(robot,link,physicsClientId=client)[12].decode('utf-8')
    matches = [link for link,name in names.items() if name=='base_link']
    if len(matches)!=1:
        raise RuntimeError(f'Cannot uniquely identify base_link: {names}')

    def pose(link):
        if link==-1:
            position,rotation=p.getBasePositionAndOrientation(robot,physicsClientId=client)
            dynamics=p.getDynamicsInfo(robot,link,physicsClientId=client)
            ip,iq=p.invertTransform(dynamics[3],dynamics[4])
            link_position,_=p.multiplyTransforms(position,rotation,ip,iq)
        else:
            state=p.getLinkState(robot,link,computeForwardKinematics=True,physicsClientId=client)
            position,rotation,link_position=state[0],state[1],state[4]
        matrix=np.asarray(p.getMatrixFromQuaternion(rotation)).reshape(3,3)
        return np.asarray(position),matrix,np.asarray(link_position)

    base_xy=pose(matches[0])[2][:2]
    polygons=[]
    included=[]
    for link,name in names.items():
        shapes=p.getCollisionShapeData(robot,link,physicsClientId=client)
        if not shapes:
            continue
        position,rotation,_=pose(link)
        groups=[]
        if len(shapes)==1 and shapes[0][2]==p.GEOM_MESH:
            count,vertices=p.getMeshData(robot,linkIndex=link,
                flags=p.MESH_DATA_SIMULATION_MESH,physicsClientId=client)
            vertices=np.asarray(vertices,float)
            if count==0 or vertices.shape!=(count,3) or not np.isfinite(vertices).all():
                raise RuntimeError(f'No usable simulation mesh for {name}')
            # Rigid simulation vertices already contain collision-local transforms.
            groups.append(vertices)
        else:
            for shape in shapes:
                if shape[2]!=p.GEOM_BOX:
                    raise RuntimeError(f'Unsupported/compound collision geometry on {name}: {shape[2]}')
                corners=np.asarray(list(product((-1.,1.),repeat=3)))*(np.asarray(shape[3])/2)
                local_rotation=np.asarray(p.getMatrixFromQuaternion(shape[6])).reshape(3,3)
                groups.append(corners@local_rotation.T+np.asarray(shape[5]))
        lower,upper=map(np.asarray,p.getAABB(robot,link,physicsClientId=client))
        for vertices in groups:
            world=vertices@rotation.T+position
            if np.any(world<lower-1e-3) or np.any(world>upper+1e-3):
                raise RuntimeError(f'Mesh-frame/AABB mismatch on {name}; do not use this projection')
            polygons.append(convex_hull(world[:,:2]-base_xy))
        included.append(name)
    if not polygons:
        raise RuntimeError('Robot has no usable collision geometry')
    print('Projected collision links:', ', '.join(included))
    return polygons


def build_wall_collision_grid(env, wall_occupancy, cell_size, safety_margin=0.05):
    polygons=robot_xy_polygons(env)
    offsets=footprint_offsets(polygons,cell_size,safety_margin)
    all_points=np.vstack(polygons)
    print('Robot XY footprint bounds relative to base:',
          all_points.min(axis=0),all_points.max(axis=0))
    return inflate_walls(wall_occupancy,offsets)


def _tests():
    import unittest
    class ProjectionTests(unittest.TestCase):
        def test_reflection(self):
            wall=np.zeros((10,20)); wall[:,12]=1
            # Forward-only offset: base behind obstacle must be blocked.
            result=inflate_walls(wall,[(0,4)])
            self.assertTrue(np.all(result[:,8])); self.assertFalse(result[:,16].any())
        def test_y_sign(self):
            wall=np.zeros((12,12)); wall[3,:]=1
            result=inflate_walls(wall,[(-2,0)])
            self.assertTrue(result[5,:].all()); self.assertFalse(result[1,:].any())
        def test_no_wrapping(self):
            wall=np.zeros((5,5)); wall[:,0]=1
            self.assertFalse(inflate_walls(wall,[(0,2)]).any())
        def test_hull(self):
            hull=convex_hull([[0,0],[1,0],[1,1],[0,1],[.5,.5]])
            self.assertEqual(len(hull),4)
            self.assertTrue(hull_hits_square(hull,[1.1,.5],.1))
            self.assertFalse(hull_hits_square(hull,[1.2,.5],.1))
        def test_separate_links_preserve_gaps(self):
            polygons=[np.array([[-.7,-.1],[-.5,-.1],[-.5,.1],[-.7,.1]]),
                      np.array([[.5,-.1],[.7,-.1],[.7,.1],[.5,.1]])]
            offsets=footprint_offsets(polygons,.1,0)
            self.assertNotIn((0,0),offsets)
            self.assertIn((0,6),offsets); self.assertIn((0,-6),offsets)
        def test_robot_cell_and_obstacle_cell_extents(self):
            offsets=footprint_offsets([np.array([[0.,0.]])],.1,0)
            self.assertIn((0,1),offsets)
            self.assertNotIn((0,2),offsets)
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ProjectionTests))
    if not result.wasSuccessful():
        raise SystemExit(1)


if __name__=='__main__':
    import sys
    if '--test' in sys.argv:
        _tests()
    else:
        print('Import build_wall_collision_grid into solver.py; use --test for geometry tests.')
