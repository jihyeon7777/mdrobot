"""Unit tests for the reader's settings, region search and judging.

Needs OpenCV. The OCR engine is faked, so no Tesseract and no camera.
"""

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from mdrobot_plate_ocr.ocr import OcrResult  # noqa: E402
from mdrobot_plate_ocr.reader import (  # noqa: E402
    MAX_SCALED_PIXELS,
    PlateReader,
    ReadSettings,
    find_regions,
    prepare,
)


class FakeEngine:
    """Returns canned text and records what it was shown."""

    name = "fake"
    lang = "fake"

    def __init__(self, *texts):
        self._texts = list(texts) or [""]
        self.calls = []

    def recognize(self, image, psm=7, whitelist=""):
        self.calls.append((image.shape, psm, whitelist))
        text = self._texts[min(len(self.calls) - 1, len(self._texts) - 1)]
        return OcrResult(text, 90.0, 1.0)

    def close(self):
        pass


class TestSettingsValidation:
    @pytest.mark.parametrize(
        "kwargs",
        [
            {"detector": "magic"},
            {"strategy": "guess"},
            {"syllable_source": "vibes"},
            {"preprocess": "sharpen"},
            {"upscale": 0.0},
            {"psm": 99},
            {"max_candidates": 0},
            {"roi": (0, 0, 0, 10)},
        ],
    )
    def test_rejects_impossible_settings(self, kwargs):
        with pytest.raises(ValueError):
            ReadSettings(**kwargs)

    def test_defaults_are_the_measured_ones(self):
        settings = ReadSettings()
        assert (settings.detector, settings.strategy) == ("textband", "split")
        assert settings.syllable_source == "template"
        assert settings.preprocess == "none"  # split binarises for itself


class TestFindRegions:
    def test_no_detector_uses_the_whole_frame(self):
        gray = np.zeros((480, 640), np.uint8)
        assert find_regions(gray, ReadSettings(detector="none")) == [(0, 0, 640, 480)]

    def test_no_detector_uses_the_roi_when_set(self):
        gray = np.zeros((480, 640), np.uint8)
        settings = ReadSettings(detector="none", roi=(10, 20, 100, 50))
        assert find_regions(gray, settings) == [(10, 20, 100, 50)]

    def test_roi_is_clipped_to_the_frame(self):
        gray = np.zeros((480, 640), np.uint8)
        settings = ReadSettings(detector="none", roi=(600, 460, 500, 500))
        assert find_regions(gray, settings) == [(600, 460, 40, 20)]

    def test_a_detector_that_finds_nothing_falls_back_to_the_base_region(self):
        # Flat grey has no text bands at all; silence would be worse than
        # reading the whole frame and failing loudly.
        gray = np.full((480, 640), 128, np.uint8)
        assert find_regions(gray, ReadSettings(detector="textband")) == [(0, 0, 640, 480)]

    def test_textband_finds_a_line_of_strokes_and_reports_frame_coordinates(self):
        gray = np.full((480, 640), 240, np.uint8)
        for x in range(200, 400, 20):  # a row of vertical bars = a text line
            cv2.rectangle(gray, (x, 200), (x + 8, 250), 0, -1)
        regions = find_regions(gray, ReadSettings(detector="textband"))
        x, y, w, h = regions[0]
        assert (x, y) != (0, 0)  # not the fallback
        # The bars span x 200..388 and y 200..250; the mask is eroded, so the
        # band is allowed to sit a few pixels inside them.
        assert 190 <= x <= 210 and 380 <= x + w <= 400
        assert 190 <= y <= 210 and 240 <= y + h <= 260

    def test_a_detector_searches_inside_the_roi(self):
        gray = np.full((480, 640), 240, np.uint8)
        for x in range(200, 400, 20):
            cv2.rectangle(gray, (x, 200), (x + 8, 250), 0, -1)
        settings = ReadSettings(detector="textband", roi=(150, 150, 300, 200))
        x, y, w, h = find_regions(gray, settings)[0]
        assert x >= 150 and y >= 150


class TestPrepare:
    def test_upscales_without_binarising_by_default(self):
        gray = np.random.default_rng(0).integers(0, 255, (50, 100), dtype=np.uint8)
        prepared = prepare(gray, "none", 2.0)
        assert prepared.shape == (100, 200)
        assert len(np.unique(prepared)) > 2  # still grayscale

    @pytest.mark.parametrize("mode", ["otsu", "adaptive", "sauvola"])
    def test_binarising_modes_return_two_levels(self, mode):
        gray = np.random.default_rng(0).integers(0, 255, (60, 120), dtype=np.uint8)
        assert set(np.unique(prepare(gray, mode, 1.0))) <= {0, 255}

    def test_output_is_dark_text_on_a_light_ground(self):
        gray = np.full((60, 120), 20, np.uint8)  # mostly dark: needs inverting
        gray[20:40, 20:100] = 240
        assert prepare(gray, "otsu", 1.0).mean() >= 127

    def test_unknown_mode_is_rejected(self):
        with pytest.raises(ValueError):
            prepare(np.zeros((10, 10), np.uint8), "sharpen", 1.0)


class TestJudging:
    def _read(self, text, **kwargs):
        engine = FakeEngine(text)
        settings = ReadSettings(strategy="line", detector="none", upscale=1.0, **kwargs)
        return PlateReader(engine, settings).read(np.zeros((60, 200), np.uint8))

    def test_accepts_a_valid_plate(self):
        result = self._read("123가4568")
        assert result.accepted is not None
        assert result.text == "123가4568"

    def test_rejects_a_syllable_no_plate_carries(self):
        result = self._read("123강4568")
        assert result.accepted is None
        assert "illegal syllable" in result.best.reason

    def test_allows_any_syllable_when_told_to(self):
        assert self._read("123강4568", require_legal_syllable=False).accepted is not None

    def test_rejects_below_the_confidence_floor(self):
        result = self._read("123가4568", min_confidence=95.0)  # the fake returns 90
        assert result.accepted is None
        assert "confidence" in result.best.reason

    def test_reports_no_text_when_nothing_was_read(self):
        assert self._read("").text == ""


class TestScaleGuard:
    def test_a_huge_region_is_capped_instead_of_hanging(self):
        # Upscaling a full 1080p frame 3x makes 5760x3240, where the split
        # path's morphology took over two minutes.
        engine = FakeEngine("")
        settings = ReadSettings(strategy="split", detector="none", upscale=3.0)
        frame = np.zeros((1080, 1920), np.uint8)
        result = PlateReader(engine, settings).read(frame)
        # Nothing to segment in a blank frame, but it must return promptly.
        assert result.accepted is None
        assert MAX_SCALED_PIXELS < 1920 * 1080 * 9
