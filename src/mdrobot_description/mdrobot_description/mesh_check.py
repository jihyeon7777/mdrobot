"""Tell you whether an STL will work in the URDF before you find out in RViz.

Three things go wrong when a mesh comes out of CAD, and all three look like a
model that is simply "not there" or "wrong" in RViz, with no error to read:

**Units.** CAD exports STL in millimetres by default and URDF is metres, so a
mesh comes in a thousand times too big — which puts the whole machine outside
the camera's near/far planes, and reads as invisible rather than as huge.

**Origin.** This is the one that bites on assemblies. Exporting a wheel from an
assembly usually writes it in ASSEMBLY coordinates, so the file's origin is the
middle of the robot, not the middle of the wheel. A wheel like that does not
spin in place when its joint turns — it ORBITS, on a lever arm as long as its
offset. It looks like the model has come apart.

**Axle direction.** A wheel is thin along its axle, so which axis is thin says
which way it was modelled. This robot's joints turn about Y.

Nothing here is guesswork about the file format: it reads the triangles and
reports what is in them.

    ros2 run mdrobot_description mesh_check meshes/wheel.stl
    python3 -m mdrobot_description.mesh_check meshes/*.stl
"""

from __future__ import annotations

import argparse
import struct
import sys

AXES = ("x", "y", "z")


def read_stl(path: str) -> tuple[str, list[tuple[float, float, float]]]:
    """Return ("binary"|"ascii", vertices). Raises ValueError on anything else."""
    with open(path, "rb") as handle:
        head = handle.read(84)
        if len(head) < 84:
            raise ValueError(f"{path}: too short to be an STL")
        count = struct.unpack("<I", head[80:84])[0]
        rest = handle.read()
    # An ASCII STL starts "solid", but so do some binary ones written by sloppy
    # exporters — so decide on the SIZE, which is exact for binary: 50 bytes a
    # triangle after the 84-byte header.
    if len(rest) == count * 50:
        vertices = []
        for i in range(count):
            block = rest[i * 50:i * 50 + 50]
            # 3 floats of normal, then 3 vertices of 3 floats, then 2 spare.
            values = struct.unpack("<12fH", block)[:12]
            vertices.extend(
                (values[3 + j * 3], values[4 + j * 3], values[5 + j * 3])
                for j in range(3)
            )
        return "binary", vertices
    with open(path, "r", errors="replace") as handle:
        text = handle.read()
    if "vertex" not in text:
        raise ValueError(
            f"{path}: not a binary STL of {count} triangles and no ASCII "
            f"vertices either. Is it really an STL?"
        )
    vertices = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return "ascii", vertices


def report(path: str) -> int:
    try:
        kind, vertices = read_stl(path)
    except (OSError, ValueError, struct.error) as exc:
        print(f"{path}: {exc}")
        return 1
    if not vertices:
        print(f"{path}: no triangles")
        return 1

    lo = tuple(min(v[i] for v in vertices) for i in range(3))
    hi = tuple(max(v[i] for v in vertices) for i in range(3))
    size = tuple(hi[i] - lo[i] for i in range(3))
    mid = tuple((hi[i] + lo[i]) / 2.0 for i in range(3))
    span = max(size)

    print(f"\n{path}")
    print(f"  {kind}, {len(vertices) // 3} triangles")
    print(f"  bounding box  {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} (file units)")
    print(f"  spans         x {lo[0]:+.3f}..{hi[0]:+.3f}   "
          f"y {lo[1]:+.3f}..{hi[1]:+.3f}   z {lo[2]:+.3f}..{hi[2]:+.3f}")

    # Units. A machine this size is ~0.1-2 m, or ~100-2000 mm.
    if span > 20.0:
        scale, unit = "0.001 0.001 0.001", "millimetres"
    elif span > 0.02:
        scale, unit = "1 1 1", "metres"
    else:
        scale, unit = None, None
    if scale:
        print(f"  units         looks like {unit} "
              f"({span:.1f} across)  ->  mesh_scale:=\"{scale}\"")
        metres = span * (0.001 if scale.startswith("0.001") else 1.0)
        print(f"                that is {metres:.3f} m across")
    else:
        print(f"  units         UNCLEAR — {span:g} across in file units. "
              f"Neither mm nor m gives a sane size.")

    # Origin. offset >> size means assembly coordinates.
    offset = max(abs(c) for c in mid)
    print(f"  origin        bbox centre sits at "
          f"({mid[0]:+.3f}, {mid[1]:+.3f}, {mid[2]:+.3f})")
    if offset < 0.05 * span:
        print(f"                OK — the file's origin is at the part's own "
              f"centre.")
    else:
        print(f"                ** ASSEMBLY COORDINATES ** — the origin is "
              f"{offset:.3f} away,")
        print(f"                which is {offset / span:.1f}x the part's own "
              f"size. A wheel exported")
        print(f"                like this ORBITS instead of spinning. Re-export "
              f"it from the")
        print(f"                part, or say so and the URDF can offset it back.")

    # Axle. A wheel is thin along its axle; a body is not thin at all.
    thin = min(range(3), key=lambda i: size[i])
    if size[thin] < 0.5 * span:
        print(f"  thinnest axis {AXES[thin]} ({size[thin]:.3f} vs {span:.3f}) — "
              f"for a wheel that is the axle.")
        if AXES[thin] != "y":
            print(f"                This robot's wheel joints turn about Y, so "
                  f"it needs an rpy")
            print(f"                in the URDF. Tell me and it goes in.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("paths", nargs="+", help="STL files to inspect")
    args = parser.parse_args(argv)
    worst = 0
    for path in args.paths:
        worst = max(worst, report(path))
    print()
    return worst


if __name__ == "__main__":
    raise SystemExit(main())
