"""When a lamp should be lit. Pure logic — no GPIO, no ROS.

The only interesting part of an indicator is the hold: the plate detector
drops out for a beat at a time on a plate that is plainly in view, and a lamp
wired straight to "a reading arrived this instant" would flicker so much it
would be harder to read than the topic it replaced. So a sighting lights it and
it stays lit until nothing has arrived for `hold`.

Kept separate from the node so the timing can be tested without a Pi.
"""

from __future__ import annotations


class Latch:
    """Lit on a signal, dark once nothing has come for `hold` seconds."""

    def __init__(self, hold: float) -> None:
        if hold <= 0:
            raise ValueError(f"hold must be positive, got {hold}")
        self.hold = hold
        self._last: float | None = None

    def signal(self, now: float) -> None:
        """Something arrived."""
        self._last = now

    def clear(self) -> None:
        """Go dark at once, without waiting the hold out.

        For a signal that is a STATE rather than an event — a brake that is
        either on or off is known to be off, where a detector going quiet only
        means nothing has arrived yet.
        """
        self._last = None

    def lit(self, now: float) -> bool:
        if self._last is None:
            return False
        # A reading stamped in the future would latch the lamp on forever, so
        # treat the clock going backwards as staleness rather than freshness.
        return 0.0 <= now - self._last <= self.hold
