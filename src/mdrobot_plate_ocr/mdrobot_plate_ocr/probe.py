#!/usr/bin/env python3
"""Bring-up command line for the plate reader. No ROS 2 — runs from a bare clone.

The modes are meant to be worked in order, because each one removes a variable:

``--synthetic``   Render a perfect plate with Pillow and OCR it. No camera. If
                  this cannot read a pixel-perfect ``12가3456``, nothing
                  downstream can work and the camera is not the problem.
``--shot FILE``   Apply the v4l2 controls, capture one frame, report the focus
                  score and glyph height. Answers "is the plate in frame and
                  sharp" without streaming anything.
``--matrix FILE`` Run every preprocess x psm x language combination over one
                  saved frame and print a ranked table. **This is what chooses
                  the defaults** — everything else is a guess until it runs.
``--live``        The loop: capture, read, debounce, write debug JPEGs, print.

Examples::

    python3 examples/plate_ocr_probe.py --synthetic
    python3 examples/plate_ocr_probe.py --shot debug/shot.jpg
    python3 examples/plate_ocr_probe.py --matrix debug/shot.jpg
    python3 examples/plate_ocr_probe.py --live --psm 7 --preprocess none
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

from .camera import (
    CAMERA_AUTO_EXPOSURE_CONTROLS,
    DEFAULT_V4L2_CONTROLS,
    Camera,
    CameraError,
    CameraSettings,
    focus_score,
)
from .debounce import Debouncer
from .debug import DebugWriter, find_font
from .exposure import MODES as EXPOSURE_MODES
from .exposure import ExposureController, ExposureSettings
from .normalize import DEFAULT_PLATE_PATTERN, normalize
from .ocr import DEFAULT_LANG, make_engine
from .reader import (
    DETECTORS,
    PREPROCESS_MODES,
    STRATEGIES,
    SYLLABLE_SOURCES,
    PlateReader,
    ReadSettings,
    estimate_glyph_height,
)

DEFAULT_PLATE_TEXT = "12가3456"
MATRIX_PSMS = (6, 7, 11, 13)
MATRIX_LANGS = ("kor", "kor+eng")


# --- synthetic plate --------------------------------------------------------


def render_plate(text: str = DEFAULT_PLATE_TEXT, width: int = 480) -> np.ndarray:
    """A clean grayscale plate at Korean proportions (335 x 155 mm)."""
    height = round(width * 155 / 335)
    image = Image.new("L", (width, height), 255)
    font = find_font(int(height * 0.55))
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
    draw.text(
        ((width - (right - left)) / 2 - left, (height - (bottom - top)) / 2 - top),
        text,
        font=font,
        fill=0,
    )
    return np.array(image)


# --- modes ------------------------------------------------------------------


def run_synthetic(args: argparse.Namespace) -> int:
    engine = make_engine(args.backend, args.lang)
    print(f"backend={engine.name} lang={engine.lang} text={args.text!r}")
    plate = render_plate(args.text)

    out = Path(args.debug_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out / "synthetic.jpg"), plate)
    print(f"rendered {plate.shape[1]}x{plate.shape[0]} -> {out / 'synthetic.jpg'}")

    failures = 0
    try:
        for psm in MATRIX_PSMS:
            result = engine.recognize(plate, psm)
            candidate = normalize(result.text, args.pattern)
            mark = "OK  " if candidate.accepted else "    "
            print(
                f"{mark}psm {psm:>2}  conf {result.confidence:5.1f}  "
                f"{result.elapsed_ms:6.1f}ms  raw={candidate.cleaned!r}  "
                f"-> {candidate.text or '-'}  ({candidate.reason})"
            )
            failures += 0 if candidate.accepted else 1
    finally:
        engine.close()

    if failures == len(MATRIX_PSMS):
        print("\nFAIL: no page segmentation mode read a perfect synthetic plate.")
        return 1
    print("\nPASS: the OCR chain works. The camera is the only remaining variable.")
    return 0


def run_shot(args: argparse.Namespace) -> int:
    destination = Path(args.shot).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with _open_camera(args) as camera:
        for problem in camera.control_problems:
            print(f"warn: v4l2 {problem}", file=sys.stderr)
        started = time.perf_counter()
        frame = camera.grab()
        elapsed_ms = (time.perf_counter() - started) * 1e3
        print(f"requested {args.width}x{args.height}, got {camera.actual_size}")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(str(destination), frame)
    print(f"capture {elapsed_ms:.0f}ms -> {destination}")
    print(f"focus {focus_score(gray):.0f}   glyph height ~{estimate_glyph_height(gray):.0f}px")
    print("glyph height 30-40px is Tesseract's sweet spot; move the paper or raise --width")
    return 0


def run_matrix(args: argparse.Namespace) -> int:
    frame = cv2.imread(str(Path(args.matrix).expanduser()), cv2.IMREAD_COLOR)
    if frame is None:
        print(f"cannot read image: {args.matrix}", file=sys.stderr)
        return 1
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    print(f"{args.matrix}: {frame.shape[1]}x{frame.shape[0]}  focus {focus_score(gray):.0f}  "
          f"glyph ~{estimate_glyph_height(gray):.0f}px")

    detectors: list[tuple[str, tuple[int, int, int, int] | None]] = [("textband", None)]
    if args.roi:
        detectors.append(("roi", tuple(args.roi)))  # type: ignore[arg-type]

    digit_engine = make_engine(args.backend, args.digit_lang)
    rows = []
    try:
        for lang in MATRIX_LANGS:
            try:
                engine = make_engine(args.backend, lang)
            except RuntimeError as exc:
                print(f"skip lang={lang}: {exc}", file=sys.stderr)
                continue
            try:
                for region_name, roi in detectors:
                    for strategy in STRATEGIES:
                        # Only the single-pass strategy is sensitive to
                        # preprocessing; split binarises for itself.
                        modes = PREPROCESS_MODES if strategy == "line" else ("none",)
                        for preprocess in modes:
                            for psm in MATRIX_PSMS:
                                settings = ReadSettings(
                                    roi=roi,
                                    detector="none" if roi else "textband",
                                    strategy=strategy,
                                    preprocess=preprocess,
                                    psm=psm,
                                    upscale=args.upscale,
                                    pattern=args.pattern,
                                )
                                result = PlateReader(engine, settings, digit_engine).read(frame)
                                best = result.best
                                rows.append(
                                    (
                                        bool(result.accepted),
                                        best.ocr.confidence if best else -1.0,
                                        region_name,
                                        strategy,
                                        preprocess,
                                        psm,
                                        lang,
                                        best.candidate.cleaned if best else "",
                                        best.candidate.text if best else "",
                                        result.elapsed_ms,
                                        best.reason if best else "no attempt",
                                    )
                                )
            finally:
                engine.close()
    finally:
        digit_engine.close()

    rows.sort(key=lambda row: (not row[0], -row[1]))
    header = (
        f"\n{'':4} {'conf':>5} {'region':<9} {'strat':<6} {'prep':<9} {'psm':>3} "
        f"{'lang':<8} {'ms':>7}  raw -> plate"
    )
    print(header)
    print("-" * (len(header) + 30))
    for ok, conf, region, strategy, prep, psm, lang, raw, text, ms, reason in rows[: args.top]:
        mark = "OK  " if ok else "    "
        tail = f"{raw!r} -> {text}" if ok else f"{raw[:40]!r}  ({reason})"
        print(
            f"{mark} {conf:5.1f} {region:<9} {strategy:<6} {prep:<9} {psm:>3} "
            f"{lang:<8} {ms:7.1f}  {tail}"
        )
    if len(rows) > args.top:
        print(f"... {len(rows) - args.top} more rows hidden (--top to show them)")

    winners = [row for row in rows if row[0]]
    if not winners:
        print("\nFAIL: no combination produced a valid plate. Check debug/latest_crop.jpg,")
        print("      the focus score, and the lighting before changing the algorithm.")
        return 1
    _, conf, region, strategy, prep, psm, lang, _, text, ms, _ = winners[0]
    print(
        f"\nPASS: best = region {region}, strategy {strategy}, preprocess {prep}, "
        f"psm {psm}, lang {lang} -> {text} (conf {conf:.0f}, {ms:.0f}ms)"
    )
    return 0


def run_live(args: argparse.Namespace) -> int:
    engine = make_engine(args.backend, args.lang)
    digit_engine = make_engine(args.backend, args.digit_lang)
    settings = _settings_from(args)
    reader = PlateReader(engine, settings, digit_engine)
    debouncer = Debouncer(args.confirm_count, args.history_size, args.repeat_interval)
    writer = DebugWriter(args.debug_dir, ring_size=args.ring_size)
    exposure = ExposureController(
        ExposureSettings(mode=args.exposure_mode, start=args.exposure)
    )
    period = 1.0 / args.rate

    print(
        f"backend={engine.name} lang={engine.lang} digits={digit_engine.lang} "
        f"strategy={args.strategy} psm={args.psm} preprocess={args.preprocess} "
        f"detector={args.detector} rate={args.rate}Hz"
    )
    print(f"debug -> {writer.directory}/latest.jpg   (ctrl-c to stop)")

    camera = None
    frames = 0
    try:
        camera = _open_camera(args)
        for problem in camera.control_problems:
            print(f"warn: v4l2 {problem}", file=sys.stderr)

        while True:
            started = time.monotonic()
            frame = camera.grab()
            result = reader.read(frame)
            frames += 1

            accepted = result.accepted
            published = (
                debouncer.offer(accepted.candidate.text, started)
                if accepted
                else (debouncer.miss(), None)[1]
            )
            if published:
                print(f"\n>>> {published}\n")

            # Meter the region the OCR actually read, so the exposure tracks the
            # plate rather than the room.
            metered = _metered_region(result)
            new_exposure = exposure.update(metered)
            if new_exposure is not None:
                problem = camera.set_exposure(new_exposure)
                if problem:
                    print(f"warn: exposure {problem}", file=sys.stderr)
                elif args.verbose:
                    print(f"     exposure -> {new_exposure} ({exposure.reason})")

            best = result.best
            status = f"#{frames} {1.0 / max(time.monotonic() - started, 1e-6):.1f}Hz"
            writer.write(frame, result, status)
            if best is not None and args.verbose:
                syllable = (
                    f"  syl={best.syllable.syllable!r} margin={best.syllable.margin:.3f}"
                    f" tess={best.tesseract_syllable!r}"
                    if best.syllable
                    else ""
                )
                offset = (
                    f"  off=({result.offset.dx_norm:+.2f},{result.offset.dy_norm:+.2f})"
                    f" {result.offset.bearing_deg:+.1f}deg"
                    if result.offset
                    else ""
                )
                print(
                    f"{status}  exp {exposure.value:<4} focus {result.focus:.0f}  "
                    f"glyph {result.glyph_height:.0f}px  conf {best.ocr.confidence:5.1f}  "
                    f"raw={best.candidate.cleaned!r}  ({best.reason}){syllable}{offset}"
                )

            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    except CameraError as exc:
        print(f"camera error: {exc}", file=sys.stderr)
        return 1
    finally:
        if camera is not None:
            camera.close()
        engine.close()
        digit_engine.close()


# --- wiring -----------------------------------------------------------------


def _metered_region(result) -> np.ndarray:
    """The pixels the exposure loop should meter: the OCR region, else the frame."""
    best = result.best
    if best is None:
        return result.gray
    x, y, w, h = best.region
    return result.gray[y : y + h, x : x + w]


def _settings_from(args: argparse.Namespace) -> ReadSettings:
    return ReadSettings(
        roi=tuple(args.roi) if args.roi else None,  # type: ignore[arg-type]
        detector=args.detector,
        strategy=args.strategy,
        preprocess=args.preprocess,
        upscale=args.upscale,
        psm=args.psm,
        pattern=args.pattern,
        min_confidence=args.min_confidence,
        require_legal_syllable=not args.allow_any_syllable,
        syllable_source=args.syllable_source,
    )


def _open_camera(args: argparse.Namespace) -> Camera:
    if args.no_v4l2:
        controls: tuple[str, ...] = ()
    elif args.exposure_mode == "camera":
        controls = CAMERA_AUTO_EXPOSURE_CONTROLS
    else:
        controls = tuple(
            f"exposure_time_absolute={args.exposure}" if c.startswith("exposure_time_absolute") else c
            for c in DEFAULT_V4L2_CONTROLS
        )
    return Camera(
        CameraSettings(
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            rotate=args.rotate,
            v4l2_controls=controls,
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Korean licence plate OCR bring-up probe (no ROS 2).",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--synthetic", action="store_true", help="OCR a rendered plate, no camera")
    mode.add_argument("--shot", metavar="FILE", help="capture one frame to FILE")
    mode.add_argument("--matrix", metavar="FILE", help="rank every setting over a saved frame")
    mode.add_argument("--live", action="store_true", help="continuous read loop")

    camera = parser.add_argument_group("camera")
    camera.add_argument("--device", default="/dev/video0")
    camera.add_argument("--width", type=int, default=1920)
    camera.add_argument("--height", type=int, default=1080)
    camera.add_argument("--fps", type=int, default=30)
    camera.add_argument("--rotate", type=int, default=0, choices=(0, 90, 180, 270))
    camera.add_argument("--no-v4l2", action="store_true", help="do not touch camera controls")
    camera.add_argument(
        "--exposure-mode", default="adaptive", choices=EXPOSURE_MODES,
        help="adaptive: meter the plate region and track the lighting",
    )
    camera.add_argument("--exposure", type=int, default=60, help="start value, 100 us units")

    ocr = parser.add_argument_group("ocr")
    ocr.add_argument("--backend", default="auto", choices=("auto", "tesserocr", "cli"))
    ocr.add_argument("--lang", default=DEFAULT_LANG, help="language for the Hangul syllable")
    ocr.add_argument(
        "--digit-lang", default="eng", help="language for the digit runs; kor returns nothing"
    )
    ocr.add_argument("--psm", type=int, default=7)
    ocr.add_argument("--strategy", default="split", choices=STRATEGIES)
    ocr.add_argument("--syllable-source", default="template", choices=SYLLABLE_SOURCES)
    ocr.add_argument("--preprocess", default="none", choices=PREPROCESS_MODES)
    ocr.add_argument("--detector", default="textband", choices=DETECTORS)
    ocr.add_argument("--upscale", type=float, default=3.0)
    ocr.add_argument("--pattern", default=DEFAULT_PLATE_PATTERN)
    ocr.add_argument("--min-confidence", type=float, default=0.0)
    ocr.add_argument("--allow-any-syllable", action="store_true")
    ocr.add_argument(
        "--roi", type=int, nargs=4, metavar=("X", "Y", "W", "H"), help="restrict OCR to a box"
    )
    ocr.add_argument("--text", default=DEFAULT_PLATE_TEXT, help="--synthetic plate content")
    ocr.add_argument("--top", type=int, default=20, help="--matrix rows to print")

    live = parser.add_argument_group("live")
    live.add_argument("--rate", type=float, default=2.0, help="Hz")
    live.add_argument("--confirm-count", type=int, default=3)
    live.add_argument("--history-size", type=int, default=5)
    live.add_argument("--repeat-interval", type=float, default=3.0, help="s")
    live.add_argument("--ring-size", type=int, default=30)
    live.add_argument("--debug-dir", default="debug")
    live.add_argument("-v", "--verbose", action="store_true", help="print every attempt")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.synthetic:
            return run_synthetic(args)
        if args.shot:
            return run_shot(args)
        if args.matrix:
            return run_matrix(args)
        return run_live(args)
    except (CameraError, RuntimeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
