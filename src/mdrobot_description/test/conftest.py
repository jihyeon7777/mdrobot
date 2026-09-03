"""A cylinder writer, so the mesh checks run without any CAD in the loop."""

import math
import struct

import pytest


def write_cylinder(path, radius, width, axis, centre=(0.0, 0.0, 0.0), facets=24):
    """A closed cylinder of `radius` and `width`, axle along `axis`, at `centre`."""
    def point(along, r, angle):
        c = [0.0, 0.0, 0.0]
        c[axis] = along
        first, second = [i for i in range(3) if i != axis]
        c[first] = r * math.cos(angle)
        c[second] = r * math.sin(angle)
        return tuple(c[i] + centre[i] for i in range(3))

    triangles = []
    for i in range(facets):
        a0 = 2 * math.pi * i / facets
        a1 = 2 * math.pi * (i + 1) / facets
        for end in (-width / 2, width / 2):
            triangles.append((point(end, 0, 0), point(end, radius, a0),
                              point(end, radius, a1)))
        triangles.append((point(-width / 2, radius, a0),
                          point(width / 2, radius, a0),
                          point(width / 2, radius, a1)))
    with open(path, "wb") as handle:
        handle.write(b"\0" * 80)
        handle.write(struct.pack("<I", len(triangles)))
        for tri in triangles:
            handle.write(struct.pack("<3f", 0.0, 0.0, 1.0))
            for vertex in tri:
                handle.write(struct.pack("<3f", *vertex))
            handle.write(struct.pack("<H", 0))
    return str(path)


@pytest.fixture
def cylinder():
    return write_cylinder
