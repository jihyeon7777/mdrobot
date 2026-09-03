#!/usr/bin/env python3
"""Drive the mecanum base from the keyboard, straight from a bare clone.

SAFETY: THE MOTORS WILL TURN. Start with the wheels off the ground, keep a power cut
within reach, and start slow — the default is 20% of the configured limits.

    python3 examples/mecanum_teleop.py                    # ./mecanum.yaml, 20% speed
    python3 examples/mecanum_teleop.py --scale 40
    python3 examples/mecanum_teleop.py --config other.yaml --port /dev/ttyUSB1

Keys are printed on screen when it starts. In short: WASD translates (a/d STRAFE,
they do not turn), q/e rotate, SPACE is a hard stop, any unmapped key is a soft stop,
and Ctrl-C stops the motors and cuts torque before exiting.

Run examples/mecanum_identify.py first — this needs the wheel map in mecanum.yaml.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Allow running straight from the repo without installing either package.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src" / "mdrobot"))
sys.path.insert(0, str(_ROOT / "src" / "mdrobot_mecanum"))

from mdrobot_mecanum.teleop_keyboard import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
