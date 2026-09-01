#!/usr/bin/env python3
"""Step the camera exposure across a range and report what the plate reads at each.

The reads that worked early in bring-up were found by hand-tuning exposure, and
the value that works indoors will not be the one that works in a car park. This
does the sweep and prints a table, so the number goes into plate_ocr.yaml from
a measurement rather than from a guess.

What it found on this machine, for the record: exposure is not the limit.
Across 8 to 320 in 25 steps, two exposures produced the right text once each,
and repeating the read five times at each of the four best values gave 0 hits
out of 20. Sweeping upscale from 0.25 to 3.0 at a fixed exposure did no better.
The failures are consistent — 3 read as 5, and the Hangul syllable coming out
as any of 고 오 조 우 허 — which is a recognition problem, not a lighting one.

Hold the plate where the machine will see it and leave it there. Nothing else
may hold the camera: stop plate_ocr_node and the live view first.

    python3 examples/plate_exposure_sweep.py
    python3 examples/plate_exposure_sweep.py --expect 52가3108
    python3 examples/plate_exposure_sweep.py --from 20 --to 400 --step 20
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot_plate_ocr"))

from mdrobot_plate_ocr.camera import Camera, CameraSettings  # noqa: E402
from mdrobot_plate_ocr.ocr import make_engine  # noqa: E402
from mdrobot_plate_ocr.reader import PlateReader, ReadSettings  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--roi-height", type=int, default=700)
    ap.add_argument("--from", dest="start", type=int, default=20)
    ap.add_argument("--to", dest="stop", type=int, default=320)
    ap.add_argument("--step", type=int, default=20)
    ap.add_argument("--settle", type=float, default=0.6,
                    help="seconds to let the sensor follow a new exposure")
    ap.add_argument("--expect", default="",
                    help="the plate's actual text, to mark exact hits")
    ap.add_argument("--strategy", default="both", choices=("split", "line", "both"))
    ap.add_argument("--save", action="store_true", help="write a frame per step")
    args = ap.parse_args()

    try:
        camera = Camera(CameraSettings(device=args.device, width=args.width,
                                       height=args.height))
    except Exception as exc:
        print(f"could not open {args.device}: {exc}")
        print("stop plate_ocr_node / plate_live_view first — they hold the camera")
        return 1

    kor = make_engine("tesserocr", lang="kor")
    eng = make_engine("tesserocr", lang="eng")
    strategies = ("split", "line") if args.strategy == "both" else (args.strategy,)

    print(f"sweeping exposure {args.start}..{args.stop} step {args.step}"
          f"   expect {args.expect or '(not given)'}\n")
    header = f"{'exp':>5s} {'bright':>7s} {'focus':>7s} {'width':>6s}"
    for name in strategies:
        header += f" {name:>26s}"
    print(header)

    hits: list[tuple[int, str, str]] = []
    for exposure in range(args.start, args.stop + 1, args.step):
        camera.set_exposure(exposure)
        time.sleep(args.settle)
        frame = camera.grab()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        roi = (0, 0, w, min(args.roi_height, h))

        row = f"{exposure:5d}"
        band_done = False
        for name in strategies:
            settings = ReadSettings(roi=roi, detector="textband", strategy=name,
                                    syllable_source="template",
                                    preprocess="none" if name == "split" else "otsu",
                                    upscale=3.0, psm=7)
            result = PlateReader(kor, settings, eng).read(frame)
            best = result.best
            if not band_done:
                if best is not None:
                    x, y, bw, bh = best.region
                    band = gray[y : y + bh, x : x + bw]
                    row += (f" {band.mean():7.0f}"
                            f" {cv2.Laplacian(band, cv2.CV_64F).var():7.0f}"
                            f" {bw / w * 100:5.0f}%")
                else:
                    row += f" {gray.mean():7.0f} {'-':>7s} {'-':>6s}"
                band_done = True
            raw = (result.text or (best.ocr.text.strip() if best else "")).replace("\n", "")
            mark = ""
            if args.expect and args.expect in raw.replace(" ", ""):
                mark = " *"
                hits.append((exposure, name, raw))
            row += f" {raw[:24]!r:>25s}{mark}"
        print(row)
        if args.save:
            cv2.imwrite(f"debug/exp_{exposure:04d}.jpg", frame)

    camera.close()
    print()
    if args.expect:
        if hits:
            print(f"read {args.expect} at:")
            for exposure, name, raw in hits:
                print(f"  exposure {exposure}   strategy {name}   raw {raw!r}")
            print("\nPut the value in plate_ocr.yaml as exposure_start with "
                  "exposure_mode: manual.")
        else:
            print(f"{args.expect} was not read at any exposure in this range.")
            print("Widen the range, or move the plate: exposure is not the "
                  "limit if the band is already sharp and filling the frame.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
