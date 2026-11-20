"""Objects of the full YCB library, converted to the course's object format.

The library is the git submodule `assets/ycb_urdfs` (78 objects). Its URDFs are
not usable as they are: every mesh is scaled by 10, every mass is 1 kg and
every inertia is the unit matrix. The course's 15 objects
(`assets/ycb_objects/NAME/`) have instead a mesh in metres centred on the
centre of mass, a mass that matches the mesh's volume at about the density of
water, and the inertia that goes with it. This tool writes the library's
other objects in that same form to `assets/ycb_extra/NAME/`:

    coacd_decomposed_object_one_link.urdf   the name the env loads (the collision
                                            mesh here is the library's V-HACD
                                            decomposition, not CoACD)
    textured.obj                            the surface the env samples the cloud from
    collision_vhacd.obj                     the convex pieces

    pnr/bin/python tools/ycb_library.py --check     compare the conversion with the course's own 15
    pnr/bin/python tools/ycb_library.py             convert every object the course does not have

What is assumed, and cannot be checked against the course for a new object:
the density (`--check` prints what the course's 15 imply), uniform mass, and
the default contact parameters of a URDF without contact tags, as the
course's objects have.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
LIBRARY = ROOT / "assets" / "ycb_urdfs" / "ycb_assets"
COURSE = ROOT / "assets" / "ycb_objects"
EXTRA = ROOT / "assets" / "ycb_extra"
DENSITY = 900.0             # kg per cubic metre of the convex pieces: what the course's solid objects imply (`--check`)

URDF = """<robot name="root">
  <link name="link_original">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="textured.obj" scale="1.0000E+00 1.0000E+00 1.0000E+00"/>
      </geometry>
    </visual>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="{mass:.4E}"/>
      <inertia ixx="{ixx:.2E}" ixy="{ixy:.2E}" ixz="{ixz:.2E}" iyy="{iyy:.2E}" iyz="{iyz:.2E}" izz="{izz:.2E}"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="collision_vhacd.obj" scale="1.0000E+00 1.0000E+00 1.0000E+00"/>
      </geometry>
    </collision>
  </link>
</robot>
"""


def names() -> list[str]:
    """Every object of the library that has both meshes."""
    return sorted(d.name for d in LIBRARY.iterdir()
                  if (d / "google_16k" / "textured.obj").exists() and (d / "google_16k" / "textured_vhacd.obj").exists())


def pieces(path: Path) -> list[trimesh.Trimesh]:
    """The convex pieces of a decomposed collision mesh: one per `o` object
    of the OBJ file, as the simulator reads it."""
    groups: list[list] = []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("o ") or not groups:
            groups.append([])
        if line.startswith("v "):
            groups[-1].append([float(v) for v in line.split()[1:4]])
    out = []
    for points in groups:
        if len(points) >= 4:
            try:
                out.append(trimesh.convex.convex_hull(np.array(points)))
            except Exception:                       # a flat sliver has no volume
                pass
    return out


def solid(name: str) -> dict:
    """Mass properties of the library's object at the assumed density, from
    its convex pieces (its surface mesh is not closed, and the hull of a
    bowl or a mug is mostly air)."""
    surface = trimesh.load(str(LIBRARY / name / "google_16k" / "textured.obj"), force="mesh", process=False)
    parts = pieces(LIBRARY / name / "google_16k" / "textured_vhacd.obj")
    masses = np.array([DENSITY * part.volume for part in parts])
    centres = np.array([part.center_mass for part in parts])
    mass = float(masses.sum())
    centre = (masses[:, None] * centres).sum(axis=0) / mass
    inertia = np.zeros((3, 3))
    for part, m, c in zip(parts, masses, centres):
        part = part.copy()
        part.density = DENSITY
        d = c - centre
        inertia += part.moment_inertia + m * ((d @ d) * np.eye(3) - np.outer(d, d))     # parallel axes
    return dict(mass=mass, centre=centre, inertia=inertia, extents=np.array(surface.extents), pieces=len(parts))


def moved(path: Path, shift: np.ndarray) -> str:
    """The text of an OBJ file with every vertex moved by `-shift`, its
    material lines dropped and everything else (the grouping into convex
    pieces, the faces) left as it is."""
    out = []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("v "):
            parts = line.split()
            xyz = np.array([float(v) for v in parts[1:4]]) - shift
            out.append("v " + " ".join(f"{v:.6f}" for v in xyz) + ("" if len(parts) == 4 else " " + " ".join(parts[4:])))
        elif not line.startswith(("mtllib", "usemtl")):
            out.append(line)
    return "\n".join(out) + "\n"


def convert(name: str) -> dict:
    props = solid(name)
    out = EXTRA / name
    out.mkdir(parents=True, exist_ok=True)
    source = LIBRARY / name / "google_16k"
    (out / "textured.obj").write_text(moved(source / "textured.obj", props["centre"]))
    (out / "collision_vhacd.obj").write_text(moved(source / "textured_vhacd.obj", props["centre"]))
    i = props["inertia"]
    (out / "coacd_decomposed_object_one_link.urdf").write_text(URDF.format(
        mass=props["mass"], ixx=i[0, 0], ixy=i[0, 1], ixz=i[0, 2], iyy=i[1, 1], iyz=i[1, 2], izz=i[2, 2]))
    return props


def check() -> None:
    """The course's own 15, converted from the library, against the course's files."""
    import re
    print(f"{'object':24s} {'course kg':>9s} {'ours kg':>8s} {'density the course implies':>27s}  {'extents, course / library (cm)'}")
    for name in sorted(d.name for d in COURSE.iterdir()):
        if name not in names():
            print(f"{name:24s} not in the library")
            continue
        text = (COURSE / name / "coacd_decomposed_object_one_link.urdf").read_text()
        mass = float(re.search(r'mass value="([^"]+)"', text).group(1))
        theirs = trimesh.load(str(COURSE / name / "textured.obj"), force="mesh", process=False)
        props = solid(name)
        print(f"{name:24s} {mass:9.3f} {props['mass']:8.3f} {DENSITY * mass / props['mass']:22.0f} kg/m3  "
              f"{np.round(100 * np.sort(theirs.extents), 1)} / {np.round(100 * np.sort(props['extents']), 1)}"
              f"  {props['pieces']} pieces")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="compare the conversion with the course's 15 objects")
    args = parser.parse_args()
    if not LIBRARY.exists():
        print("assets/ycb_urdfs is missing: git submodule update --init --depth 1 assets/ycb_urdfs")
        return 1
    if args.check:
        check()
        return 0
    have = {d.name for d in COURSE.iterdir()}
    for name in names():
        if name in have:
            continue
        props = convert(name)
        print(f"{name:28s} {props['mass']:7.3f} kg  {np.round(100 * np.sort(props['extents']), 1)} cm"
              f"  {props['pieces']} pieces")
    print("written to", EXTRA)
    return 0


if __name__ == "__main__":
    sys.exit(main())
