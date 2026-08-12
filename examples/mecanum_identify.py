#!/usr/bin/env python3
"""Run the mecanum bring-up / identification wizard from a bare clone.

SAFETY: the default mode TURNS THE MOTORS, one at a time. Put the wheels off the
ground and keep a power cut within reach. Use --scan-only or --preflight to stop
before any motion.

    python3 examples/mecanum_identify.py --scan-only     # read-only, nothing turns
    python3 examples/mecanum_identify.py --preflight     # + config writes, nothing turns
    python3 examples/mecanum_identify.py                 # + spins each wheel in turn
    python3 examples/mecanum_identify.py --rpm 20 --spin 3.0

With --port omitted the MDROBOT_PORT environment variable is used.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Allow running straight from the repo without installing either package.
_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src" / "mdrobot"))
sys.path.insert(0, str(_ROOT / "src" / "mdrobot_mecanum"))

from mdrobot_mecanum.identify import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
