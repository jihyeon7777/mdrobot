#!/usr/bin/env python3
"""Write annotated JPEGs to disk — the only way to see the camera over SSH.

RViz over the network was unusably slow, so nothing here streams: a person
opens the files in a remote editor. That constrains the design more than it
looks like it should.

**Writes are atomic.** ``cv2.imwrite`` is not, and an editor's file watcher
fires on the *first* write event, so a plain overwrite is intermittently opened
half-written. Everything goes to ``<name>.tmp`` in the same directory and then
``os.replace``.

**One frame produces every file in one call.** Writing the overview and the
Tesseract input on separate conditions means comparing two different moments
and drawing the wrong conclusion.

**Rejected frames are kept.** Encoding costs ~7 ms for ~100 KB at 720p, so
every attempt goes into a small ring buffer. When the plate is not being read,
the failures are the only evidence of why.

Text is drawn with Pillow, not ``cv2.putText``, which cannot render Hangul —
a plate overlay of ``12???3456`` would defeat the purpose.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .reader import ReadResult

FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
)

_ACCEPTED = (0, 220, 0)
_REJECTED = (0, 200, 255)
_CENTRE = (255, 160, 0)
_UNSAFE = re.compile(r"[^0-9A-Za-z가-힣]")


def find_font(size: int = 20) -> ImageFont.FreeTypeFont:
    """A font that can draw Hangul, or Pillow's bitmap default as a last resort."""
    for path in FONT_CANDIDATES:
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def safe_name(text: str) -> str:
    """Make OCR output safe to put in a path.

    ``plate_regex`` is a parameter, so a caller can widen it to admit ``/`` or
    ``..``; interpolating that straight into a filename is how a debug feature
    becomes a directory traversal.
    """
    return _UNSAFE.sub("_", text)[:32] or "unnamed"


def _write_atomic(path: Path, image: np.ndarray, quality: int) -> None:
    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError(f"failed to encode {path}")
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_bytes(encoded.tobytes())
    os.replace(temporary, path)  # atomic within one filesystem


def annotate(frame: np.ndarray, result: ReadResult, status: str) -> np.ndarray:
    """Draw the regions, the centre offset, and a self-explaining header."""
    canvas = frame.copy() if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    height, width = canvas.shape[:2]
    for attempt in result.attempts:
        x, y, w, h = attempt.region
        colour = _ACCEPTED if attempt.accepted else _REJECTED
        cv2.rectangle(canvas, (x, y), (x + w, y + h), colour, 2)

    # The frame's centre, and a line to the plate's, so the reported offset can
    # be checked against the picture rather than trusted.
    centre = (width // 2, height // 2)
    cv2.drawMarker(canvas, centre, _CENTRE, cv2.MARKER_CROSS, 40, 2)
    if result.offset is not None:
        plate_centre = (int(result.offset.centre_px[0]), int(result.offset.centre_px[1]))
        cv2.line(canvas, centre, plate_centre, _CENTRE, 2)
        cv2.drawMarker(canvas, plate_centre, _CENTRE, cv2.MARKER_TILTED_CROSS, 24, 2)

    best = result.best
    lines = [
        f"{status}  focus {result.focus:.0f}  glyph {result.glyph_height:.0f}px  "
        f"{result.elapsed_ms:.0f}ms",
        f"raw: {best.candidate.cleaned!r}  conf {best.ocr.confidence:.0f}" if best else "raw: -",
    ]
    if best is not None:
        lines.append(
            f"-> {best.candidate.text}  ACCEPTED"
            if best.accepted
            else f"rejected: {best.reason}"
        )
    if result.offset is not None:
        offset = result.offset
        lines.append(
            f"offset {offset.dx_px:+.0f},{offset.dy_px:+.0f}px  "
            f"({offset.dx_norm:+.2f},{offset.dy_norm:+.2f})  "
            f"bearing {offset.bearing_deg:+.1f}deg  width {offset.width_ratio * 100:.0f}%"
        )

    regions = [attempt.region for attempt in result.attempts]
    return _draw_header(canvas, lines, _ACCEPTED if result.accepted else _REJECTED, regions)


def _draw_header(
    canvas: np.ndarray,
    lines: list[str],
    colour: tuple[int, int, int],
    regions: list[tuple[int, int, int, int]],
) -> np.ndarray:
    font = find_font(20)
    line_height = 26
    box_height = line_height * len(lines) + 10
    height, width = canvas.shape[:2]

    # Put the header wherever it hides less of the plate. A plate taped high on
    # a box sits at the top of the frame, exactly where a header would go, and
    # covering the thing you are trying to look at defeats the purpose.
    def hidden(top: int) -> int:
        bottom = top + box_height
        return sum(
            max(0, min(y + h, bottom) - max(y, top)) * w for _, y, w, h in regions
        )

    top = 0 if hidden(0) <= hidden(height - box_height) else height - box_height
    cv2.rectangle(canvas, (0, top), (width, top + box_height), (0, 0, 0), -1)

    # Pillow works in RGB; the frame is BGR.
    image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(image)
    for index, line in enumerate(lines):
        draw.text((8, top + 5 + index * line_height), line, font=font, fill=colour[::-1])
    return cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)


class DebugWriter:
    """Owns a debug directory: ``latest``, a ring of attempts, and the hits."""

    def __init__(
        self,
        directory: str | os.PathLike[str],
        ring_size: int = 30,
        jpeg_quality: int = 80,
        keep_hits: bool = True,
    ) -> None:
        if ring_size < 1:
            raise ValueError(f"ring_size must be >= 1, got {ring_size}")
        self.directory = Path(directory).expanduser()
        self.frames_dir = self.directory / "frames"
        self.hits_dir = self.directory / "hits"
        self.ring_size = ring_size
        self.jpeg_quality = jpeg_quality
        self.keep_hits = keep_hits
        self._index = 0

        self.frames_dir.mkdir(parents=True, exist_ok=True)
        if keep_hits:
            self.hits_dir.mkdir(parents=True, exist_ok=True)
        probe = self.directory / ".writable"
        probe.write_text("")  # fail loudly at start-up, not on the first frame
        probe.unlink()

    def write(self, frame: np.ndarray, result: ReadResult, status: str = "") -> Path:
        """Write every debug artefact for one frame. Returns ``latest.jpg``."""
        overlay = annotate(frame, result, status)
        latest = self.directory / "latest.jpg"
        _write_atomic(latest, overlay, self.jpeg_quality)
        _write_atomic(
            self.frames_dir / f"{self._index % self.ring_size:03d}.jpg",
            overlay,
            self.jpeg_quality,
        )

        best = result.best
        if best is not None:
            _write_atomic(self.directory / "latest_crop.jpg", best.prepared, self.jpeg_quality)
        accepted = result.accepted
        if accepted is not None and self.keep_hits:
            name = f"{self._index:05d}_{safe_name(accepted.candidate.text)}.jpg"
            _write_atomic(self.hits_dir / name, overlay, self.jpeg_quality)

        self._index += 1
        return latest
