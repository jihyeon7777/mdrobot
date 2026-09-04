"""Band skew: the second measurement, and its sign.

The plate's centre offset alone cannot separate "I am beside the plate" from
"I am turned away from it" — sliding sideways and rotating both move the plate
across the frame the same way. One measurement, two unknowns. This is what
makes the pair solvable, so its SIGN has to be right: taken backwards, a
controller built on it would turn away from square rather than onto it.

Built through the real detector's mask rather than around it, because the
number that matters is the one the running system produces.
"""

import numpy as np
import pytest

from mdrobot_plate_ocr.reader import ReadSettings, band_skew

SETTINGS = ReadSettings()


def strokes(left_height: int, right_height: int, width: int = 600,
            height: int = 200) -> np.ndarray:
    """A light field with dark vertical bars: tall ones left, short ones right.

    A trapezoid is what a flat rectangle projects to when it is seen from off
    to one side, and the taller end is the nearer one. This fakes that directly
    rather than rendering a plate in perspective.
    """
    image = np.full((height, width), 220, dtype=np.uint8)
    for i, x in enumerate(range(20, width - 20, 40)):
        # Interpolate the bar height across the band.
        t = i / max(1, (width - 40) // 40 - 1)
        bar = int(round(left_height * (1 - t) + right_height * t))
        top = (height - bar) // 2
        image[top:top + bar, x:x + 12] = 30
    return image


def test_a_square_band_reads_as_no_skew():
    result = band_skew(strokes(120, 120), (0, 0, 600, 200), SETTINGS)
    assert result is not None
    assert result[0] == pytest.approx(0.0, abs=0.05)


def test_a_taller_left_end_reads_positive():
    # Positive means the LEFT end is taller, so the left side is nearer.
    skew, left, right = band_skew(strokes(150, 60), (0, 0, 600, 200), SETTINGS)
    assert skew > 0.1
    assert left > right


def test_a_taller_right_end_reads_negative():
    skew, left, right = band_skew(strokes(60, 150), (0, 0, 600, 200), SETTINGS)
    assert skew < -0.1
    assert right > left


def test_the_sign_flips_with_the_geometry():
    # The one property a controller depends on: mirroring the scene mirrors the
    # answer. A metric that did not would drive the error the wrong way.
    a = band_skew(strokes(150, 60), (0, 0, 600, 200), SETTINGS)[0]
    b = band_skew(strokes(60, 150), (0, 0, 600, 200), SETTINGS)[0]
    assert a == pytest.approx(-b, abs=0.15)


def test_it_is_scale_free():
    # Approaching the plate makes everything bigger without turning the
    # machine, so the answer must not grow with it.
    near = band_skew(strokes(150, 60, width=600, height=200),
                     (0, 0, 600, 200), SETTINGS)[0]
    far = band_skew(strokes(75, 30, width=300, height=100),
                    (0, 0, 300, 100), SETTINGS)[0]
    assert near == pytest.approx(far, abs=0.2)


def test_an_empty_region_gives_nothing_rather_than_zero():
    # Zero would read as "square on", which is a claim. None is the absence of
    # one, and the caller drops the frame instead of steering on it.
    blank = np.full((200, 600), 220, dtype=np.uint8)
    assert band_skew(blank, (0, 0, 600, 200), SETTINGS) is None


def test_a_region_off_the_edge_of_the_frame_gives_nothing():
    assert band_skew(strokes(120, 120), (900, 0, 600, 200), SETTINGS) is None


def test_a_region_too_narrow_to_have_two_ends_gives_nothing():
    assert band_skew(strokes(120, 120), (0, 0, 4, 200), SETTINGS) is None
