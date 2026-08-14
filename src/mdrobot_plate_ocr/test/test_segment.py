"""Unit tests for character segmentation. Needs OpenCV, but no camera and no OCR."""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from mdrobot_plate_ocr.segment import (  # noqa: E402
    Segment,
    band,
    drop_border_components,
    ink_mass,
    segment_characters,
    trim_to_length,
)


def strip(*runs, height=100, width=400):
    """A binary canvas with a filled block for each (start, end) run."""
    canvas = np.zeros((height, width), np.uint8)
    for start, end in runs:
        canvas[10 : height - 10, start:end] = 255
    return canvas


class TestSegmentCharacters:
    def test_finds_one_run_per_block(self):
        segments = segment_characters(strip((10, 40), (60, 90), (110, 140)))
        assert [(s.start, s.end) for s in segments] == [(10, 40), (60, 90), (110, 140)]

    def test_merges_runs_closer_than_the_merge_ratio(self):
        # 5 px apart on a 100 px-tall crop is under the 10% default: this is one
        # character split into parts, the way 가 is ㄱ plus ㅏ.
        segments = segment_characters(strip((10, 40), (45, 70), (150, 180)))
        assert [(s.start, s.end) for s in segments] == [(10, 70), (150, 180)]

    def test_keeps_runs_further_apart_than_the_merge_ratio(self):
        segments = segment_characters(strip((10, 40), (60, 90)))
        assert len(segments) == 2

    def test_discards_runs_narrower_than_the_width_floor(self):
        # 4 px wide against a 100 px crop is under the 10% minimum.
        segments = segment_characters(strip((10, 40), (200, 204)))
        assert [(s.start, s.end) for s in segments] == [(10, 40)]

    def test_blank_image_yields_nothing(self):
        assert segment_characters(np.zeros((100, 400), np.uint8)) == []


class TestDropBorderComponents:
    def test_removes_a_frame_that_encloses_the_content(self):
        canvas = np.zeros((100, 400), np.uint8)
        cv2.rectangle(canvas, (1, 1), (398, 98), 255, 3)  # the printed border
        canvas[40:60, 100:120] = 255  # a character well inside it
        cleaned = drop_border_components(canvas)
        assert cleaned[40:60, 100:120].all()
        assert cleaned[1:4, 1:398].sum() == 0

    def test_keeps_a_glyph_that_touches_the_edge_without_spanning_it(self):
        canvas = np.zeros((100, 400), np.uint8)
        canvas[40:60, 0:20] = 255  # clipped character at the left edge
        assert drop_border_components(canvas).sum() == canvas.sum()

    def test_keeps_a_wide_shape_that_does_not_reach_the_edge(self):
        canvas = np.zeros((100, 400), np.uint8)
        canvas[40:60, 20:380] = 255
        assert drop_border_components(canvas).sum() == canvas.sum()


class TestTrimToLength:
    def test_drops_the_faintest_end_first(self):
        canvas = strip((10, 40), (60, 90), (110, 140))
        canvas[10:90, 10:40] = 0  # hollow out the leftmost run
        canvas[45:55, 10:40] = 255
        segments = segment_characters(canvas)
        assert len(segments) == 3
        trimmed = trim_to_length(canvas, segments, 2)
        assert [(s.start, s.end) for s in trimmed] == [(60, 90), (110, 140)]

    def test_drops_from_the_right_when_that_end_is_fainter(self):
        canvas = strip((10, 40), (60, 90), (110, 140))
        canvas[10:90, 110:140] = 0
        canvas[45:55, 110:140] = 255
        segments = segment_characters(canvas)
        trimmed = trim_to_length(canvas, segments, 2)
        assert [(s.start, s.end) for s in trimmed] == [(10, 40), (60, 90)]

    def test_is_a_no_op_when_already_short_enough(self):
        canvas = strip((10, 40), (60, 90))
        segments = segment_characters(canvas)
        assert trim_to_length(canvas, segments, 5) == segments

    def test_a_narrow_but_solid_character_survives_a_wider_faint_one(self):
        # The digit 1 is the narrowest real character on a plate; trimming by
        # width instead of ink would throw it away.
        canvas = np.zeros((100, 400), np.uint8)
        canvas[10:90, 10:25] = 255  # narrow, solid: the "1"
        canvas[10:90, 60:120] = 255  # wide, solid
        canvas[48:52, 200:280] = 255  # wide, faint: a border fragment
        segments = segment_characters(canvas)
        assert len(segments) == 3
        trimmed = trim_to_length(canvas, segments, 2)
        assert [(s.start, s.end) for s in trimmed] == [(10, 25), (60, 120)]


class TestHelpers:
    def test_ink_mass_counts_set_pixels(self):
        canvas = strip((10, 40))
        assert ink_mass(canvas, Segment(10, 40)) == 80 * 30

    def test_band_pads_and_clips_to_the_image(self):
        canvas = np.zeros((100, 400), np.uint8)
        assert band(canvas, 50, 100, pad=10).shape[1] == 70
        assert band(canvas, 0, 100, pad=10).shape[1] == 110  # clipped on the left
        assert band(canvas, 300, 400, pad=10).shape[1] == 110  # clipped on the right
