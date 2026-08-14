#!/usr/bin/env python3
"""Run the licence plate OCR probe from a bare clone — no colcon, no ROS 2.

    python3 examples/plate_ocr_probe.py --synthetic
    python3 examples/plate_ocr_probe.py --shot debug/shot.jpg
    python3 examples/plate_ocr_probe.py --matrix debug/shot.jpg
    python3 examples/plate_ocr_probe.py --live

Needs OpenCV, Pillow and Tesseract:

    sudo apt install tesseract-ocr tesseract-ocr-kor python3-tesserocr v4l-utils

No motor is involved, so nothing here can move the robot.
"""

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROOT / "src" / "mdrobot_plate_ocr"))

from mdrobot_plate_ocr.probe import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
