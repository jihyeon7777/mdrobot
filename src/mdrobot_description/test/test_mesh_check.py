"""Reading an STL, and the three ways a CAD export breaks the model."""

import struct

import pytest

from mdrobot_description.mesh_check import read_stl, report


def test_a_binary_stl_is_read_by_size_not_by_its_first_word(tmp_path, cylinder):
    # Some exporters write "solid" at the start of a BINARY file, so the header
    # word cannot be trusted. 84 + 50 * count bytes exactly can be.
    path = tmp_path / "w.stl"
    cylinder(path, 62.5, 60.0, axis=1)
    data = bytearray(open(path, "rb").read())
    data[:5] = b"solid"
    open(path, "wb").write(bytes(data))
    kind, vertices = read_stl(str(path))
    assert kind == "binary"
    assert len(vertices) % 3 == 0 and vertices


def test_an_ascii_stl_is_read_too(tmp_path):
    path = tmp_path / "a.stl"
    path.write_text(
        "solid s\nfacet normal 0 0 1\nouter loop\n"
        "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
        "endloop\nendfacet\nendsolid s\n")
    kind, vertices = read_stl(str(path))
    assert kind == "ascii"
    assert vertices == [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]


def test_something_that_is_not_an_stl_is_rejected(tmp_path):
    path = tmp_path / "no.stl"
    path.write_bytes(b"x" * 200)
    with pytest.raises(ValueError):
        read_stl(str(path))


def test_a_truncated_file_is_rejected(tmp_path):
    path = tmp_path / "cut.stl"
    path.write_bytes(b"\0" * 40)
    with pytest.raises(ValueError, match="too short"):
        read_stl(str(path))


def test_millimetres_are_recognised_and_the_scale_offered(tmp_path, cylinder, capsys):
    report(cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=1))
    out = capsys.readouterr().out
    assert 'mesh_scale:="0.001 0.001 0.001"' in out
    assert "0.125 m across" in out


def test_metres_are_recognised(tmp_path, cylinder, capsys):
    report(cylinder(tmp_path / "w.stl", 0.0625, 0.06, axis=1))
    out = capsys.readouterr().out
    assert 'mesh_scale:="1 1 1"' in out


def test_assembly_coordinates_are_called_out(tmp_path, cylinder, capsys):
    # The failure that matters: a wheel written in assembly coordinates does
    # not spin when its joint turns, it orbits on a lever arm that long.
    report(cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=1,
                    centre=(250.0, 287.5, 62.5)))
    out = capsys.readouterr().out
    assert "ASSEMBLY COORDINATES" in out
    assert "ORBITS" in out


def test_part_coordinates_pass_quietly(tmp_path, cylinder, capsys):
    report(cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=1))
    out = capsys.readouterr().out
    assert "ASSEMBLY COORDINATES" not in out
    assert "origin is at the part's own centre" in out


def test_an_axle_along_y_needs_no_rotation(tmp_path, cylinder, capsys):
    report(cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=1))
    out = capsys.readouterr().out
    assert "thinnest axis y" in out
    assert "needs an rpy" not in out


@pytest.mark.parametrize("axis, name", [(0, "x"), (2, "z")])
def test_an_axle_off_y_is_flagged(tmp_path, cylinder, capsys, axis, name):
    report(cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=axis))
    out = capsys.readouterr().out
    assert f"thinnest axis {name}" in out
    assert "needs an rpy" in out


def test_mirroring_keeps_the_bounding_box_where_it_was(tmp_path, cylinder):
    # The placement offsets are worked out from the original, so a reflection
    # about the mesh's OWN centre has to leave that centre alone or every
    # mirrored wheel lands somewhere different.
    from mdrobot_description.mesh_simplify import mirror, read_binary_stl

    path = cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=2,
                    centre=(74.3, 74.2, 107.6))
    original = read_binary_stl(path)
    flipped = mirror(original, 2)
    lo_a, hi_a = original.reshape(-1, 3).min(0), original.reshape(-1, 3).max(0)
    lo_b, hi_b = flipped.reshape(-1, 3).min(0), flipped.reshape(-1, 3).max(0)
    assert lo_a == pytest.approx(lo_b, abs=1e-4)
    assert hi_a == pytest.approx(hi_b, abs=1e-4)


def test_mirroring_reverses_the_winding_so_the_surface_still_faces_out(
        tmp_path, cylinder):
    # A reflection flips orientation. Left uncorrected the wheel renders inside
    # out -- which is what a negative <mesh scale> in the URDF would have done.
    import numpy as np

    from mdrobot_description.mesh_simplify import mirror, read_binary_stl

    path = cylinder(tmp_path / "w.stl", 62.5, 60.0, axis=2)
    original = read_binary_stl(path)

    def outward(tri):
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        points = tri.reshape(-1, 3)
        centre = (points.min(0) + points.max(0)) / 2
        return float((np.einsum("ij,ij->i", normals,
                                tri.mean(1) - centre) > 0).mean())

    corrected = mirror(original, 2)
    uncorrected = corrected[:, [0, 2, 1], :]
    assert outward(corrected) == pytest.approx(outward(original), abs=1e-6)
    # The two are exact complements: every face that faced out now faces in.
    assert outward(uncorrected) == pytest.approx(1.0 - outward(original), abs=1e-6)
