#!/usr/bin/env python3
"""Live camera window with the plate detector drawn on top. Needs the Pi's monitor.

Over SSH the only view of the camera was a JPEG written to disk after the fact,
which is how several wrong diagnoses got made — a focus score measured over the
carpet, a "glyph height" that belonged to floor texture. On the monitor this
shows, at frame rate, exactly what the detector is deciding and why.

    ROI          the region searched, in blue. Everything outside is ignored
    detection    the chosen plate band, in green — red when nothing was found
    crosshair    the band's centre against the frame's, which is the offset the
                 alignment steers on

The readout is the numbers that actually decide recognition: focus measured
over the DETECTED BAND rather than the whole frame, the glyph height, the
band's width as a fraction of the frame, and the offset.

Nothing else may hold the camera: stop the plate_ocr node first.

    python3 examples/plate_live_view.py
    python3 examples/plate_live_view.py --ocr        # also try to read it
    python3 examples/plate_live_view.py --no-roi     # see what the ROI hides
    python3 examples/plate_live_view.py --rotate 180 # camera mounted upside down

Keys:  q quit   r toggle the ROI   o toggle OCR   s save the frame
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot_plate_ocr"))

from mdrobot_plate_ocr.camera import Camera, CameraSettings  # noqa: E402
from mdrobot_plate_ocr.reader import (  # noqa: E402
    PlateReader,
    ReadSettings,
    estimate_glyph_height,
    find_regions,
    offset_from_centre,
)

GREEN, RED, BLUE, WHITE, BLACK = ((0, 220, 0), (0, 0, 255), (255, 160, 0),
                                  (255, 255, 255), (0, 0, 0))


def focus_of(gray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def banner(frame, lines: list[str]) -> None:
    """Readable over any scene: white on a solid strip."""
    height = 26 * len(lines) + 10
    cv2.rectangle(frame, (0, 0), (frame.shape[1], height), BLACK, -1)
    for i, text in enumerate(lines):
        cv2.putText(frame, text, (10, 26 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, WHITE, 2, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270),
                    help="degrees clockwise; 180 if the camera is mounted upside "
                         "down. Must match plate_ocr.yaml's rotate")
    ap.add_argument("--roi-height", type=int, default=700,
                    help="rows from the top to search; the floor must be below it")
    ap.add_argument("--no-roi", action="store_true")
    ap.add_argument("--ocr", action="store_true", help="also read the plate (slow)")
    ap.add_argument("--show-width", type=int, default=1280, help="window width")
    args = ap.parse_args()

    # The package's own Camera, not a bare VideoCapture: the V4L2 queue on this
    # webcam is four frames deep, so a plain read() hands back something from a
    # few hundred milliseconds ago and the picture appears not to follow the
    # plate. Camera.grab drains the queue first.
    try:
        camera = Camera(CameraSettings(device=args.device, width=args.width,
                                       height=args.height, rotate=args.rotate))
    except Exception as exc:
        print(f"could not open {args.device}: {exc}")
        print("is plate_ocr_node still running? it holds the camera")
        return 1

    window = "plate detector"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.show_width, int(args.show_width * 9 / 16))

    use_roi = not args.no_roi
    use_ocr = args.ocr
    reader = None
    print("q quit   r toggle ROI   o toggle OCR   s save")

    frames = 0
    while True:
        try:
            frame = camera.grab()
        except Exception as exc:
            print(f"frame grab failed: {exc}")
            break
        frames += 1
        if frames <= 3:
            # If the window comes up black, this says whether the camera is
            # handing over an all-dark frame or the drawing is at fault.
            print(f"frame {frames}: {frame.shape[1]}x{frame.shape[0]} "
                  f"mean brightness {frame.mean():.1f}")
        started = time.perf_counter()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        roi = (0, 0, w, min(args.roi_height, h)) if use_roi else None
        settings = ReadSettings(roi=roi, detector="textband", detect_only=not use_ocr,
                                strategy="split", syllable_source="template",
                                preprocess="none", upscale=3.0, psm=7)

        text = ""
        if use_ocr:
            if reader is None:
                from mdrobot_plate_ocr.ocr import make_engine
                reader = PlateReader(make_engine("tesserocr", lang="kor"), settings,
                                     make_engine("tesserocr", lang="eng"))
            reader.settings = settings
            result = reader.read(frame)
            regions = [result.best.region] if result.best else []
            text = result.text or (result.best.ocr.text.strip() if result.best else "")
        else:
            regions = find_regions(gray, settings)
        elapsed = (time.perf_counter() - started) * 1e3

        if roi is not None:
            cv2.rectangle(frame, (roi[0], roi[1]),
                          (roi[0] + roi[2], roi[1] + roi[3]), BLUE, 3)
        cv2.drawMarker(frame, (w // 2, h // 2), WHITE, cv2.MARKER_CROSS, 40, 2)

        if regions:
            x, y, rw, rh = regions[0]
            band = gray[y : y + rh, x : x + rw]
            off = offset_from_centre(regions[0], gray.shape)
            cv2.rectangle(frame, (x, y), (x + rw, y + rh), GREEN, 4)
            cx, cy = int(x + rw / 2), int(y + rh / 2)
            cv2.drawMarker(frame, (cx, cy), GREEN, cv2.MARKER_TILTED_CROSS, 40, 3)
            cv2.line(frame, (w // 2, h // 2), (cx, cy), GREEN, 2)
            lines = [
                f"focus(band) {focus_of(band):6.0f}   glyph {estimate_glyph_height(band):4.0f}px"
                f"   width {off.width_ratio * 100:3.0f}%   {elapsed:5.0f}ms",
                f"offset x {off.dx_norm:+.3f}  y {off.dy_norm:+.3f}"
                f"   band {rw}x{rh}",
            ]
            if use_ocr:
                lines.append(f"read: {text or '(nothing)'}")
        else:
            cv2.putText(frame, "NO PLATE FOUND", (w // 2 - 260, h // 2 + 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, RED, 4, cv2.LINE_AA)
            lines = [f"no detection   focus(frame) {focus_of(gray):6.0f}"
                     f"   {elapsed:5.0f}ms",
                     "nothing published — the sequence would wait"]
        lines.append(f"ROI {'on' if use_roi else 'OFF'}   OCR {'on' if use_ocr else 'off'}"
                     f"   rotate {args.rotate}"
                     f"   [q]uit [r]oi [o]cr [s]ave")
        banner(frame, lines)

        scale = args.show_width / w
        cv2.imshow(window, cv2.resize(frame, (args.show_width, int(h * scale))))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r"):
            use_roi = not use_roi
        if key == ord("o"):
            use_ocr = not use_ocr
        if key == ord("s"):
            name = f"debug/live_{int(time.time())}.jpg"
            cv2.imwrite(name, frame)
            print(f"saved {name}")

    camera.close()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
