"""Shrink a CAD mesh to something a Raspberry Pi can draw.

CAD writes meshes for manufacturing, not for a viewer: a wheel came out of this
machine's assembly at a million triangles, and four of those plus a body is
several million for a picture that is a few hundred pixels across. RViz on the
robot itself has to draw all of it every frame.

The method is vertex clustering: drop a grid over the mesh, replace every
vertex in a cell with the average of that cell's vertices, and throw away the
triangles that collapse to a line or a point as a result. It is the crude
decimation — it does not preserve sharp edges the way an edge-collapse with a
quadric error metric does — but it is a few lines of numpy, it has no
dependencies to install on a robot, and for a body shell seen from two metres
away in RViz the difference does not show. Nothing dimensional depends on it:
the mesh is a picture, and the kinematics reads its numbers from
supervisor.yaml.

The bounding box is preserved to within one cell, so a decimated mesh still
lines up with the transforms worked out from the original.

    ros2 run mdrobot_description mesh_simplify in.STL out.stl --cells 120
"""

from __future__ import annotations

import argparse
import struct

import numpy as np


def read_binary_stl(path: str) -> np.ndarray:
    """(n, 3, 3) float32 of triangle vertices."""
    with open(path, "rb") as handle:
        blob = handle.read()
    if len(blob) < 84:
        raise ValueError(f"{path}: too short to be an STL")
    count = struct.unpack("<I", blob[80:84])[0]
    body = blob[84:84 + count * 50]
    if len(body) != count * 50:
        raise ValueError(
            f"{path}: header claims {count} triangles, which needs "
            f"{count * 50} bytes and the file has {len(body)}. ASCII STL? "
            f"Convert it first."
        )
    records = np.frombuffer(body, dtype=np.uint8).reshape(count, 50)
    return records[:, 12:48].copy().view(np.float32).reshape(count, 3, 3)


def write_binary_stl(path: str, triangles: np.ndarray) -> None:
    count = len(triangles)
    records = np.zeros((count, 50), dtype=np.uint8)
    # Normals left at zero: every renderer worth using recomputes them from the
    # winding, and a stale normal from before decimation would be wrong anyway.
    # 9 floats a triangle, which is the 36 bytes at offset 12 in the record.
    records[:, 12:48] = triangles.astype("<f4").reshape(count, 9).view(np.uint8)
    with open(path, "wb") as handle:
        handle.write(b"decimated by mdrobot_description.mesh_simplify".ljust(80, b"\0"))
        handle.write(struct.pack("<I", count))
        handle.write(records.tobytes())


def simplify(triangles: np.ndarray, cells: int) -> np.ndarray:
    """Vertex-cluster onto a grid `cells` across the longest side."""
    if cells < 2:
        raise ValueError(f"cells must be at least 2, got {cells}")
    points = triangles.reshape(-1, 3).astype(np.float64)
    lo = points.min(axis=0)
    span = points.max(axis=0) - lo
    step = span.max() / cells
    if step <= 0:
        return triangles

    # Which cell each vertex falls in, as one integer per vertex.
    grid = np.floor((points - lo) / step).astype(np.int64)
    dims = grid.max(axis=0) + 1
    keys = (grid[:, 0] * dims[1] + grid[:, 1]) * dims[2] + grid[:, 2]

    unique, inverse = np.unique(keys, return_inverse=True)
    # Cell representative = the mean of its vertices, which keeps the surface
    # where it was instead of snapping it to a lattice.
    sums = np.zeros((len(unique), 3))
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse, minlength=len(unique))
    representatives = sums / counts[:, None]

    faces = inverse.reshape(-1, 3)
    # A triangle whose corners landed in the same cell has no area left.
    keep = (
        (faces[:, 0] != faces[:, 1])
        & (faces[:, 1] != faces[:, 2])
        & (faces[:, 0] != faces[:, 2])
    )
    return representatives[faces[keep]].astype(np.float32)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("source")
    parser.add_argument("destination")
    parser.add_argument(
        "--cells", type=int, default=120,
        help="grid cells across the longest side; lower is coarser (default 120)")
    args = parser.parse_args(argv)

    triangles = read_binary_stl(args.source)
    reduced = simplify(triangles, args.cells)
    write_binary_stl(args.destination, reduced)

    before = triangles.reshape(-1, 3)
    after = reduced.reshape(-1, 3)
    size_before = before.max(axis=0) - before.min(axis=0)
    size_after = after.max(axis=0) - after.min(axis=0)
    drift = np.abs(size_after - size_before).max()
    print(f"{args.source} -> {args.destination}")
    print(f"  {len(triangles)} -> {len(reduced)} triangles "
          f"({len(reduced) / len(triangles):.1%})")
    print(f"  bounding box changed by at most {drift:.3f} file units "
          f"(one cell is {size_before.max() / args.cells:.3f})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
