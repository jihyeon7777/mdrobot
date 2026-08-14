"""Unit tests for the plate-centre offset. Pure geometry — no camera, no OCR."""

import pytest

pytest.importorskip("cv2")  # reader imports OpenCV at module scope

from mdrobot_plate_ocr.reader import offset_from_centre  # noqa: E402

FRAME = (1080, 1920)  # a frame's .shape


def centred(width=400, height=80):
    """A region sitting exactly in the middle of FRAME."""
    return ((1920 - width) // 2, (1080 - height) // 2, width, height)


class TestSigns:
    def test_a_centred_plate_reads_zero(self):
        offset = offset_from_centre(centred(), FRAME)
        assert offset.dx_px == 0 and offset.dy_px == 0
        assert offset.dx_norm == 0 and offset.dy_norm == 0
        assert offset.bearing_deg == 0 and offset.elevation_deg == 0

    def test_right_of_centre_is_positive_x(self):
        x, y, w, h = centred()
        offset = offset_from_centre((x + 100, y, w, h), FRAME)
        assert offset.dx_px == 100
        assert offset.dx_norm > 0
        assert offset.bearing_deg > 0

    def test_left_of_centre_is_negative_x(self):
        x, y, w, h = centred()
        offset = offset_from_centre((x - 100, y, w, h), FRAME)
        assert offset.dx_px == -100
        assert offset.bearing_deg < 0

    def test_below_centre_is_positive_y(self):
        x, y, w, h = centred()
        offset = offset_from_centre((x, y + 50, w, h), FRAME)
        assert offset.dy_px == 50
        assert offset.dy_norm > 0
        assert offset.elevation_deg > 0

    def test_above_centre_is_negative_y(self):
        x, y, w, h = centred()
        offset = offset_from_centre((x, y - 50, w, h), FRAME)
        assert offset.dy_px == -50
        assert offset.elevation_deg < 0


class TestNormalisation:
    def test_the_frame_corners_map_to_plus_and_minus_one(self):
        # A zero-size region at the bottom-right corner.
        offset = offset_from_centre((1920, 1080, 0, 0), FRAME)
        assert offset.dx_norm == pytest.approx(1.0)
        assert offset.dy_norm == pytest.approx(1.0)
        top_left = offset_from_centre((0, 0, 0, 0), FRAME)
        assert top_left.dx_norm == pytest.approx(-1.0)
        assert top_left.dy_norm == pytest.approx(-1.0)

    def test_width_ratio_is_the_fraction_of_the_frame(self):
        assert offset_from_centre(centred(width=192), FRAME).width_ratio == pytest.approx(0.1)

    def test_centre_px_is_the_region_centre(self):
        assert offset_from_centre((100, 200, 40, 20), FRAME).centre_px == (120.0, 210.0)


class TestBearing:
    def test_the_frame_edge_is_half_the_field_of_view(self):
        offset = offset_from_centre((1920, 540, 0, 0), FRAME, hfov_deg=70.0)
        assert offset.bearing_deg == pytest.approx(35.0)

    def test_a_wider_lens_reports_a_wider_angle_for_the_same_pixels(self):
        x, y, w, h = centred()
        region = (x + 200, y, w, h)
        narrow = offset_from_centre(region, FRAME, hfov_deg=40.0)
        wide = offset_from_centre(region, FRAME, hfov_deg=90.0)
        assert wide.bearing_deg > narrow.bearing_deg
        assert narrow.dx_px == wide.dx_px  # pixels do not depend on the lens

    def test_a_degenerate_frame_does_not_divide_by_zero(self):
        offset = offset_from_centre((0, 0, 0, 0), (0, 0))
        assert offset.dx_norm == 0.0 and offset.dy_norm == 0.0
        assert offset.width_ratio == 0.0
