"""Unit tests for the exposure loop. No camera — it is handed pixel arrays."""

import pytest

np = pytest.importorskip("numpy")

from mdrobot_plate_ocr.exposure import ExposureController, ExposureSettings  # noqa: E402


def region(value, height=40, width=100):
    return np.full((height, width), value, np.uint8)


class TestValidation:
    @pytest.mark.parametrize(
        "kwargs",
        [
            {"mode": "clever"},
            {"minimum": 0},
            {"minimum": 100, "maximum": 50},
            {"start": 5000},  # outside minimum..maximum
            {"step": 1.0},
            {"settle_frames": -1},
        ],
    )
    def test_rejects_impossible_settings(self, kwargs):
        with pytest.raises(ValueError):
            ExposureSettings(**kwargs)


class TestModes:
    @pytest.mark.parametrize("mode", ["manual", "camera"])
    def test_inactive_modes_never_move(self, mode):
        controller = ExposureController(ExposureSettings(mode=mode))
        assert controller.update(region(0)) is None
        assert controller.update(region(255)) is None
        assert controller.value == 60

    def test_adaptive_is_active(self):
        assert ExposureController().active


class TestTracking:
    def _controller(self, **kwargs):
        return ExposureController(ExposureSettings(settle_frames=0, **kwargs))

    def test_holds_inside_the_dead_band(self):
        controller = self._controller()
        assert controller.update(region(150)) is None
        assert "ok" in controller.reason

    def test_opens_up_in_the_dark(self):
        controller = self._controller()
        value = controller.update(region(40))
        assert value is not None and value > 60
        assert "dark" in controller.reason

    def test_stops_down_when_bright(self):
        controller = self._controller()
        value = controller.update(region(220))
        assert value is not None and value < 60
        assert "bright" in controller.reason

    def test_saturation_outranks_a_reasonable_mean(self):
        # A plate can be blown out while the average still looks fine, and
        # clipped pixels carry no detail to recover.
        canvas = region(120)
        canvas[:, :20] = 255  # 20% of pixels clipped
        controller = self._controller()
        value = controller.update(canvas)
        assert value is not None and value < 60
        assert "saturated" in controller.reason

    def test_converges_on_a_scene_that_needs_opening_up(self):
        controller = self._controller()
        for _ in range(20):
            # Brightness rises with exposure; 40 units per 100 us of exposure.
            controller.update(region(min(255, int(controller.value * 2.4))))
        assert abs(controller.value * 2.4 - 150) < 40

    def test_clamps_to_the_configured_limits(self):
        controller = self._controller(minimum=50, maximum=70)
        for _ in range(20):
            controller.update(region(0))
        assert controller.value == 70
        for _ in range(20):
            controller.update(region(255))
        assert controller.value == 50

    def test_reports_when_it_is_stuck_at_a_limit(self):
        controller = self._controller(minimum=50, maximum=60, start=60)
        controller.update(region(0))
        assert controller.update(region(0)) is None
        assert "limit" in controller.reason

    def test_an_empty_region_is_ignored(self):
        assert ExposureController().update(np.zeros((0, 0), np.uint8)) is None


class TestSettling:
    def test_waits_for_the_change_to_reach_the_sensor(self):
        # A UVC exposure change takes a few frames to appear; reacting to stale
        # frames in the meantime makes the loop oscillate.
        controller = ExposureController(ExposureSettings(settle_frames=2))
        assert controller.update(region(0)) is not None
        assert controller.update(region(0)) is None
        assert controller.update(region(0)) is None
        assert controller.reason == "settling"
        assert controller.update(region(0)) is not None
