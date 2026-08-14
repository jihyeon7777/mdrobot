#!/usr/bin/env python3
"""Decide when a repeatedly-read plate string is worth publishing.

Pure standard library — no OpenCV, no Tesseract, no ROS 2, and no wall clock of
its own: the caller passes the timestamp in, so the tests are deterministic.

Two independent jobs:

* **Confirmation** — a single OCR frame is not evidence. A string must appear
  ``confirm_count`` times inside the last ``history_size`` reads before it is
  published, which throws away the one-off misreads that dominate raw output.
  The default of 3-in-5 is measured, not chosen for symmetry: over 143 live
  frames the correct plate read 131 times and four different wrong plates read
  once or twice each, never three times inside any window of five. Confidence
  cannot do this job — a wrong read scored 96 while a correct one scored 86.
* **Suppression** — once published, the same string stays quiet for
  ``repeat_interval_s`` so a plate parked in front of the camera does not flood
  the topic. A *different* plate is never suppressed.
"""

from __future__ import annotations

from collections import OrderedDict, deque


class Debouncer:
    """Sliding-window confirmation with per-string repeat suppression."""

    def __init__(
        self,
        confirm_count: int = 3,
        history_size: int = 5,
        repeat_interval_s: float = 3.0,
        max_tracked: int = 64,
    ) -> None:
        if confirm_count < 1:
            raise ValueError(f"confirm_count must be >= 1, got {confirm_count}")
        if history_size < confirm_count:
            raise ValueError(
                f"history_size ({history_size}) must be >= confirm_count ({confirm_count})"
            )
        if repeat_interval_s < 0.0:
            raise ValueError(f"repeat_interval_s must be >= 0, got {repeat_interval_s}")
        if max_tracked < 1:
            raise ValueError(f"max_tracked must be >= 1, got {max_tracked}")

        self._confirm_count = confirm_count
        self._repeat_interval_s = repeat_interval_s
        self._max_tracked = max_tracked
        self._history: deque[str] = deque(maxlen=history_size)
        # Bounded on purpose: an unbounded dict keyed by OCR output grows for as
        # long as the node runs.
        self._published: OrderedDict[str, float] = OrderedDict()

    def miss(self) -> None:
        """Record a frame that produced no acceptable plate.

        Misses occupy a slot in the window, so an intermittent misread cannot
        accumulate towards confirmation across a long stretch of blank frames.
        """
        self._history.append("")

    def offer(self, text: str, now: float) -> str | None:
        """Record an accepted read; return the text when it should be published.

        Returns ``None`` while the string is unconfirmed or still suppressed.
        """
        if not text:
            self.miss()
            return None

        self._history.append(text)
        if self._history.count(text) < self._confirm_count:
            return None

        last = self._published.get(text)
        if last is not None and now - last < self._repeat_interval_s:
            return None

        self._published[text] = now
        self._published.move_to_end(text)
        while len(self._published) > self._max_tracked:
            self._published.popitem(last=False)
        return text
