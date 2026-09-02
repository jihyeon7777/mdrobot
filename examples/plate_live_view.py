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
    python3 examples/plate_live_view.py --flip-display  # only the window is wrong

Keys:  q quit   r ROI   o read once   a all candidates   m mask view
       [ ] exposure   s save the frame
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
    textband_mask,
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
    ap.add_argument("--ocr", action="store_true",
                    help="read the plate on every frame. Slow enough that the view "
                         "stops being live — press o for a single read instead")
    ap.add_argument("--show-width", type=int, default=1280, help="window width")
    ap.add_argument("--exposure", type=int, default=0,
                    help="manual exposure in 100us units; 0 leaves the camera on "
                         "auto. Adjust live with [ and ]")
    ap.add_argument("--all", action="store_true",
                    help="draw every candidate region, not only the chosen one — "
                         "shows what else is competing with the plate")
    ap.add_argument("--flip-display", action="store_true",
                    help="turn the WINDOW the other way up. Display only — the "
                         "detector still works on the frame as captured, so the "
                         "offset numbers keep the sign the robot uses. Use "
                         "--rotate 180 instead if the camera itself delivers an "
                         "upside-down picture")
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

    exposure = args.exposure
    if exposure:
        camera.set_exposure(exposure)

    window = "plate detector"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.show_width, int(args.show_width * 9 / 16))

    use_roi = not args.no_roi
    use_ocr = args.ocr
    show_all = args.all
    once = False          # o requests one read, then the view goes back to fast
    last_read = ""
    # m cycles the picture: the camera, or the binary mask the detector actually
    # decides from. When detection comes and goes on a plate that is plainly in
    # frame, the mask is where the answer is.
    view = "camera"
    reader = None
    print("q quit   r ROI   o read once (O = always)   a all candidates"
          "   m mask view   [ ] exposure   s save")

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
        reading = use_ocr or once
        settings = ReadSettings(roi=roi, detector="textband", detect_only=not reading,
                                strategy="split", syllable_source="template",
                                preprocess="none", upscale=3.0, psm=7,
                                # Only while looking: with OCR on, every extra
                                # candidate is another Tesseract pass.
                                max_candidates=8 if (show_all and not reading) else 3)

        text = ""
        if reading:
            if reader is None:
                from mdrobot_plate_ocr.ocr import make_engine
                reader = PlateReader(make_engine("tesserocr", lang="kor"), settings,
                                     make_engine("tesserocr", lang="eng"))
            reader.settings = settings
            result = reader.read(frame)
            regions = [result.best.region] if result.best else []
            text = result.text or (result.best.ocr.text.strip() if result.best else "")
            last_read = text or "(nothing)"
            once = False
        else:
            regions = find_regions(gray, settings)
        elapsed = (time.perf_counter() - started) * 1e3

        if view == "mask":
            search = gray[roi[1] : roi[1] + roi[3], roi[0] : roi[0] + roi[2]] \
                if roi is not None else gray
            mask = textband_mask(search, settings)
            shown_gray = frame.copy()
            shown_gray[:] = 0
            painted = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            oy, ox = (roi[1], roi[0]) if roi is not None else (0, 0)
            shown_gray[oy : oy + painted.shape[0], ox : ox + painted.shape[1]] = painted
            frame = shown_gray

        if roi is not None:
            cv2.rectangle(frame, (roi[0], roi[1]),
                          (roi[0] + roi[2], roi[1] + roi[3]), BLUE, 3)
        cv2.drawMarker(frame, (w // 2, h // 2), WHITE, cv2.MARKER_CROSS, 40, 2)

        # Everything the detector would accept, so it is obvious what the plate
        # is competing against. The chosen one is green; the rest are yellow
        # with the numbers that decide between them.
        if show_all and len(regions) > 1:
            for rx, ry, rw2, rh2 in regions[1:]:
                patch = gray[ry : ry + rh2, rx : rx + rw2]
                cv2.rectangle(frame, (rx, ry), (rx + rw2, ry + rh2), (0, 210, 210), 2)
                cv2.putText(frame,
                            f"{rw2 / w * 100:.0f}%  bright {patch.mean():.0f}"
                            f"  ar {rw2 / max(rh2, 1):.1f}",
                            (rx, max(ry - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 210, 210), 2, cv2.LINE_AA)

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
                f"   width {off.width_ratio * 100:3.0f}%   bright {band.mean():3.0f}"
                f"   {elapsed:5.0f}ms",
                f"offset x {off.dx_norm:+.3f}  y {off.dy_norm:+.3f}"
                f"   band {rw}x{rh}",
            ]
            if last_read:
                lines.append(f"read: {last_read}")
        else:
            cv2.putText(frame, "NO PLATE FOUND", (w // 2 - 260, h // 2 + 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.6, RED, 4, cv2.LINE_AA)
            lines = [f"no detection   focus(frame) {focus_of(gray):6.0f}"
                     f"   {elapsed:5.0f}ms",
                     "nothing published — the sequence would wait"]
        lines.append(
            f"ROI {'on' if use_roi else 'OFF'}  OCR {'always' if use_ocr else 'on o'}"
            f"  all {'on' if show_all else 'off'}  candidates {len(regions)}"
            f"  view {view}  exposure {exposure or 'auto'}"
            f"   [q] [r] [o] [a] [ ] [s]")
        banner(frame, lines)

        scale = args.show_width / w
        shown = cv2.resize(frame, (args.show_width, int(h * scale)))
        if args.flip_display:
            # After the drawing, so the boxes stay on the thing they mark.
            shown = cv2.rotate(shown, cv2.ROTATE_180)
        cv2.imshow(window, shown)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("r"):
            use_roi = not use_roi
        if key == ord("o"):
            # One read on the next frame. Holding OCR on drops the view to a
            # few frames a second, which makes tuning the exposure by eye
            # impossible — the thing o is usually wanted for.
            once = True
        if key == ord("O"):
            use_ocr = not use_ocr
        if key == ord("a"):
            show_all = not show_all
        if key == ord("m"):
            view = "mask" if view == "camera" else "camera"
        if key in (ord("["), ord("]")):
            # Manual exposure, in the same 100us units the node uses.
            exposure = max(1, (exposure or 60) + (10 if key == ord("]") else -10))
            camera.set_exposure(exposure)
            print(f"exposure -> {exposure}")
        if key == ord("s"):
            name = f"debug/live_{int(time.time())}.jpg"
            cv2.imwrite(name, frame)
            print(f"saved {name}")

    camera.close()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
