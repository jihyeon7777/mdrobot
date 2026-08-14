"""Unit tests for the publish debouncer. No camera, no Tesseract, no clock."""

import pytest

from mdrobot_plate_ocr.debounce import Debouncer


class TestValidation:
    @pytest.mark.parametrize(
        "kwargs",
        [
            {"confirm_count": 0},
            {"confirm_count": 3, "history_size": 2},
            {"repeat_interval_s": -1.0},
            {"max_tracked": 0},
        ],
    )
    def test_rejects_impossible_settings(self, kwargs):
        with pytest.raises(ValueError):
            Debouncer(**kwargs)


class TestConfirmation:
    def test_a_single_read_is_not_enough(self):
        d = Debouncer(confirm_count=2)
        assert d.offer("12가3456", now=0.0) is None

    def test_publishes_on_the_confirming_read(self):
        d = Debouncer(confirm_count=2)
        assert d.offer("12가3456", now=0.0) is None
        assert d.offer("12가3456", now=0.1) == "12가3456"

    def test_disagreeing_reads_do_not_confirm_each_other(self):
        d = Debouncer(confirm_count=2)
        assert d.offer("12가3456", now=0.0) is None
        assert d.offer("99나8888", now=0.1) is None

    def test_misses_push_evidence_out_of_the_window(self):
        d = Debouncer(confirm_count=2, history_size=3)
        assert d.offer("12가3456", now=0.0) is None
        d.miss()
        d.miss()
        d.miss()  # the original read has now left the window
        assert d.offer("12가3456", now=1.0) is None

    def test_empty_text_counts_as_a_miss(self):
        d = Debouncer(confirm_count=2, history_size=2)
        assert d.offer("12가3456", now=0.0) is None
        assert d.offer("", now=0.1) is None
        assert d.offer("12가3456", now=0.2) is None  # window holds "" + the new read

    def test_confirm_count_of_one_publishes_immediately(self):
        d = Debouncer(confirm_count=1)
        assert d.offer("12가3456", now=0.0) == "12가3456"


class TestSuppression:
    def test_same_plate_is_quiet_inside_the_interval(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=3.0)
        assert d.offer("12가3456", now=0.0) == "12가3456"
        assert d.offer("12가3456", now=1.0) is None
        assert d.offer("12가3456", now=2.9) is None

    def test_same_plate_publishes_again_after_the_interval(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=3.0)
        assert d.offer("12가3456", now=0.0) == "12가3456"
        assert d.offer("12가3456", now=3.0) == "12가3456"

    def test_a_different_plate_is_never_suppressed(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=3.0)
        assert d.offer("12가3456", now=0.0) == "12가3456"
        assert d.offer("99나8888", now=0.1) == "99나8888"

    def test_zero_interval_never_suppresses(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=0.0)
        assert d.offer("12가3456", now=0.0) == "12가3456"
        assert d.offer("12가3456", now=0.0) == "12가3456"


class TestBounds:
    def test_published_history_stays_bounded(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=1000.0, max_tracked=4)
        for i in range(20):
            assert d.offer(f"{i:02d}가3456", now=float(i)) is not None
        assert len(d._published) == 4

    def test_evicted_plates_lose_their_suppression(self):
        d = Debouncer(confirm_count=1, repeat_interval_s=1000.0, max_tracked=2)
        assert d.offer("11가1111", now=0.0) == "11가1111"
        assert d.offer("22나2222", now=1.0) == "22나2222"
        assert d.offer("33다3333", now=2.0) == "33다3333"  # evicts 11가1111
        # Still inside the interval, but no longer tracked, so it publishes.
        assert d.offer("11가1111", now=3.0) == "11가1111"
