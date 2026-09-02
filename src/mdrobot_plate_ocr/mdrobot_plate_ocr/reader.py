#!/usr/bin/env python3
"""Frame in, validated plate out. The pipeline the node and the probe share.

The defaults here are deliberately dull, because on this problem the clever
options lose:

**Find text, not rectangles** (``detector="textband"``). Hunting for the plate's
quadrilateral goes wrong in exactly this scene: white paper against a light wall
gives Canny no closed contour, tape at the corners and a slight bow make
``approxPolyDP`` return five or six vertices instead of four, and sorted by
area a monitor or a door frame beats the plate. A blackhat/Sobel/close pass
looks for *a wide run of vertical strokes* instead, which is what distinguishes
a plate from furniture — measured on this robot's frame it returned exactly one
candidate, enclosing the plate, in 56 ms. ``detector="contour"`` keeps the quad
search for a scene that really wants it; ``"none"`` uses the whole frame or the
``roi``.

**No hand binarisation by default** (``preprocess="none"``). Tesseract 5 runs
its own adaptive thresholding; feeding it a pre-binarised image throws away the
grayscale it would have used, and a global Otsu across a whole frame with mixed
lighting smears the plate into the background. Grayscale plus a clean 2x cubic
upscale is the baseline. The other modes are here so :mod:`probe` can measure
which one actually wins on *this* scene instead of guessing.

**Never downscale.** Tesseract's LSTM wants a glyph height of roughly 30–40 px.
Shrinking a frame to "save time" is what loses the Hangul syllable first — it
carries the most internal structure of the seven characters.

**Two strategies.** ``"line"`` is one Tesseract pass over the region.
``"split"`` cuts the region into characters first and recognises each group
under its own rule — digits with a digit whitelist, the Hangul syllable by
template matching against the 40 legal ones. On this robot's plate the line
strategy tops out at seven of eight characters (it reads ``123?4568`` and turns
``가`` into a digit); split reads all eight. See :mod:`segment` and
:mod:`hangul_template` for why each half works the way it does.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera import focus_score
from .hangul_template import SyllableMatch, match_syllable
from .normalize import DEFAULT_PLATE_PATTERN, PLATE_SYLLABLES, PlateCandidate, normalize
from .ocr import DEFAULT_PSM, DIGITS, OcrEngine, OcrResult
from .segment import (
    Segment,
    band,
    binarize,
    drop_border_components,
    segment_characters,
    trim_to_length,
)

# A whole 1920x1080 frame upscaled 3x is 5760x3240, where the morphology in the
# split path takes over two minutes. Nothing useful reads at that size, so the
# scale is capped instead of letting a misconfigured region hang the node.
MAX_SCALED_PIXELS = 2_000_000

# Tried in order when a digit run reads back the wrong number of characters.
# 6 = uniform block, 13 = raw line (skips Tesseract's own line finding).
DIGIT_RETRY_PSMS = (6, 13)

PREPROCESS_MODES = ("none", "otsu", "adaptive", "sauvola")
DETECTORS = ("none", "textband", "contour")
STRATEGIES = ("line", "split")
SYLLABLE_SOURCES = ("template", "tesseract")

Region = tuple[int, int, int, int]  # x, y, w, h in frame pixels

# Nominal horizontal field of view of the Microdia "Vitade AF". The bearing this
# produces is an estimate, not a calibration — it assumes a pinhole camera and
# ignores lens distortion, which is worst exactly at the frame edges. Use the
# normalised offset for control loops and treat the angle as a readout.
DEFAULT_HFOV_DEG = 70.0


@dataclass(frozen=True)
class Offset:
    """Where the plate sits relative to the centre of the frame.

    Signs follow the image, which is also what a robot wants: ``dx`` positive
    means the plate is to the **right** of centre, ``dy`` positive means it is
    **below** centre.
    """

    dx_px: float
    dy_px: float
    dx_norm: float  # -1 at the left edge, +1 at the right
    dy_norm: float  # -1 at the top edge, +1 at the bottom
    width_ratio: float  # plate width / frame width; a crude range proxy
    bearing_deg: float  # estimated, see DEFAULT_HFOV_DEG
    elevation_deg: float
    centre_px: tuple[float, float]


def offset_from_centre(
    region: Region, frame_shape: tuple[int, ...], hfov_deg: float = DEFAULT_HFOV_DEG
) -> Offset:
    """Measure a region's centre against the frame's centre."""
    height, width = frame_shape[0], frame_shape[1]
    x, y, w, h = region
    centre_x, centre_y = x + w / 2.0, y + h / 2.0
    dx_px, dy_px = centre_x - width / 2.0, centre_y - height / 2.0
    half_width, half_height = width / 2.0, height / 2.0

    # Vertical FOV follows from the horizontal one and the aspect ratio.
    focal_px = half_width / np.tan(np.radians(hfov_deg) / 2.0)
    return Offset(
        dx_px=dx_px,
        dy_px=dy_px,
        dx_norm=dx_px / half_width if half_width else 0.0,
        dy_norm=dy_px / half_height if half_height else 0.0,
        width_ratio=w / width if width else 0.0,
        bearing_deg=float(np.degrees(np.arctan2(dx_px, focal_px))),
        elevation_deg=float(np.degrees(np.arctan2(dy_px, focal_px))),
        centre_px=(centre_x, centre_y),
    )


@dataclass
class ReadSettings:
    """Everything that changes what Tesseract sees."""

    roi: Region | None = None
    detector: str = "textband"
    # Locate the plate and skip reading it. Finding the band is pure OpenCV and
    # costs about 34 ms; the OCR that follows costs seconds on a marginal frame.
    # A control loop steering onto the plate needs the position at rate and
    # never needs the number, so it can have the first without paying for the
    # second.
    detect_only: bool = False
    strategy: str = "split"
    preprocess: str = "none"
    upscale: float = 2.0
    psm: int = DEFAULT_PSM
    pattern: str = DEFAULT_PLATE_PATTERN
    min_confidence: float = 0.0
    require_legal_syllable: bool = True
    max_candidates: int = 3
    # split strategy only
    syllable_source: str = "template"
    expected_lengths: tuple[int, ...] = (7, 8)
    # How far ahead of the runner-up the template match must be. Off by default
    # because the debouncer already rejected every wrong syllable over 219 live
    # frames, and a second filter tuned on one scene is a good way to overfit.
    # Measured here if you want it: correct matches scored a median margin of
    # 0.106 (occasionally as low as 0.006), wrong ones 0.029 to 0.053.
    min_syllable_margin: float = 0.0
    # Only affects the reported bearing, never the recognition.
    hfov_deg: float = DEFAULT_HFOV_DEG
    # detectors. The upper aspect bound is wide because a text band is only as
    # tall as the glyphs, where a plate quad also carries its border and margin:
    # the same plate measured 5.17 as a band and 2.16 as a rectangle.
    min_area_ratio: float = 0.005
    aspect_range: tuple[float, float] = (1.8, 9.0)
    # How wide a plate may be as a fraction of the frame. Without OCR to reject
    # nonsense this is the main thing separating a plate from a shelf edge, a
    # wall line or the side of a box. Measured across thirteen frames:
    #     the plate            0.35 to 0.63
    #     other candidates     0.10  0.14  0.15  0.16
    #     a whole-frame band   1.00
    # 0.20 clears the competition with room on both sides. A plate further away
    # than that is simply not acted on yet, which is the safe way to be wrong:
    # the sequence waits instead of steering at a cardboard box.
    width_ratio_range: tuple[float, float] = (0.20, 0.75)
    # How far the textband detector reaches to join neighbouring strokes into one
    # band. It has to span the gap between the plate's character groups, and that
    # gap grows with the plate's size in frame. Measured through the real
    # detector against the true plate centre (~0 in all three frames):
    #                        width 41        width 121
    #   plate 35% of frame   +0.048          +0.009
    #   plate 40% of frame   +0.045          -0.002
    #   plate 55% of frame   -0.679  <-- caught only a fragment
    #                                        -0.001
    # 121 spans the gap at every distance tried; 41 breaks up the plate as it
    # fills the frame, and a fifth-of-a-frame error in the offset steers the
    # machine off the plate exactly when it is closest to it.
    band_close_width: int = 121
    # A plate is usually brighter than the scene around it, but only usually.
    # Against glass and white paint it measured 1.09, and frame-to-frame wobble
    # put it under a threshold of 1.0 five times out of six — detection came and
    # went with the plate in plain view. Width is what really separates a plate
    # from its competitors, so this only has to catch things far darker than the
    # scene:
    #     plates            1.09  1.13  1.14  2.03  2.15
    #     other candidates  0.52  0.53  0.69  0.88
    min_relative_brightness: float = 0.85

    def __post_init__(self) -> None:
        if self.detector not in DETECTORS:
            raise ValueError(f"detector must be one of {DETECTORS}, got {self.detector!r}")
        if self.strategy not in STRATEGIES:
            raise ValueError(f"strategy must be one of {STRATEGIES}, got {self.strategy!r}")
        if self.syllable_source not in SYLLABLE_SOURCES:
            raise ValueError(
                f"syllable_source must be one of {SYLLABLE_SOURCES}, got {self.syllable_source!r}"
            )
        if self.preprocess not in PREPROCESS_MODES:
            raise ValueError(
                f"preprocess must be one of {PREPROCESS_MODES}, got {self.preprocess!r}"
            )
        if self.upscale <= 0.0:
            raise ValueError(f"upscale must be > 0, got {self.upscale}")
        if not 0 <= self.psm <= 13:
            raise ValueError(f"psm must be 0..13, got {self.psm}")
        if self.max_candidates < 1:
            raise ValueError(f"max_candidates must be >= 1, got {self.max_candidates}")
        if self.roi is not None and (self.roi[2] <= 0 or self.roi[3] <= 0):
            raise ValueError(f"roi must have positive width and height, got {self.roi}")


@dataclass(frozen=True)
class Attempt:
    """One region, one recognition pass."""

    region: Region
    prepared: np.ndarray  # exactly the pixels handed to the recogniser
    ocr: OcrResult
    candidate: PlateCandidate
    accepted: bool
    reason: str
    segments: int = 0  # split strategy: characters found
    syllable: SyllableMatch | None = None  # split strategy: template winner
    tesseract_syllable: str = ""  # what Tesseract made of the same glyph


@dataclass(frozen=True)
class ReadResult:
    """Everything one frame produced, including the failures.

    The rejected attempts are kept on purpose: when the plate is not being read,
    they are the only thing that says why.
    """

    gray: np.ndarray
    focus: float
    glyph_height: float
    attempts: list[Attempt] = field(default_factory=list)
    elapsed_ms: float = 0.0
    offset: Offset | None = None  # of the best attempt's region

    @property
    def accepted(self) -> Attempt | None:
        return next((attempt for attempt in self.attempts if attempt.accepted), None)

    @property
    def best(self) -> Attempt | None:
        """The accepted attempt, else the one that read the most characters."""
        if not self.attempts:
            return None
        return self.accepted or max(self.attempts, key=lambda a: len(a.candidate.cleaned))

    @property
    def text(self) -> str:
        attempt = self.accepted
        return attempt.candidate.text if attempt else ""


def sauvola(gray: np.ndarray, window: int = 25, k: float = 0.2) -> np.ndarray:
    """Sauvola local thresholding, built from box filters.

    Hand-rolled because ``cv2.ximgproc`` is a separate package that is not
    installed here, and this is six lines.
    """
    window |= 1  # box filter needs an odd window
    image = gray.astype(np.float32)
    mean = cv2.boxFilter(image, cv2.CV_32F, (window, window))
    mean_square = cv2.boxFilter(image * image, cv2.CV_32F, (window, window))
    std = np.sqrt(np.maximum(mean_square - mean * mean, 0.0))
    threshold = mean * (1.0 + k * (std / 128.0 - 1.0))
    return np.where(image > threshold, 255, 0).astype(np.uint8)


def prepare(gray: np.ndarray, mode: str, upscale: float) -> np.ndarray:
    """Scale and optionally binarise a grayscale region for Tesseract."""
    if upscale != 1.0:
        gray = cv2.resize(gray, None, fx=upscale, fy=upscale, interpolation=cv2.INTER_CUBIC)
    if mode == "none":
        return gray
    if mode == "otsu":
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    elif mode == "adaptive":
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 10
        )
    elif mode == "sauvola":
        binary = sauvola(gray)
    else:
        raise ValueError(f"unknown preprocess mode {mode!r}")
    # Tesseract expects dark text on light ground. A mostly-dark result means
    # the polarity came out inverted.
    return cv2.bitwise_not(binary) if binary.mean() < 127 else binary


def estimate_glyph_height(gray: np.ndarray) -> float:
    """Median height of text-sized connected components, in source pixels.

    Reported on the debug overlay because it maps straight onto Tesseract's
    30–40 px sweet spot, which makes it far more actionable than a confidence
    number: it says "move the paper closer" or "raise the resolution".
    """
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    count, _, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    frame_height = gray.shape[0]
    heights = [
        float(stats[i, cv2.CC_STAT_HEIGHT])
        for i in range(1, count)
        # The upper bound is generous because this also runs on a tight ROI,
        # where the glyphs legitimately fill most of the crop's height.
        if 0.01 * frame_height < stats[i, cv2.CC_STAT_HEIGHT] < 0.95 * frame_height
        and 0 < stats[i, cv2.CC_STAT_WIDTH] <= 2.0 * stats[i, cv2.CC_STAT_HEIGHT]
        and stats[i, cv2.CC_STAT_AREA] > 20
    ]
    return float(np.median(heights)) if heights else 0.0


def find_regions(gray: np.ndarray, settings: ReadSettings) -> list[Region]:
    """Where to run OCR, best candidate first.

    A detector searches *inside* the ROI when one is set, so the two compose:
    the ROI narrows the scene, the detector finds the plate within it.
    """
    height, width = gray.shape[:2]
    x, y, w, h = settings.roi or (0, 0, width, height)
    x, y = max(0, x), max(0, y)
    base: Region = (x, y, min(w, width - x), min(h, height - y))
    if settings.detector == "none":
        return [base]

    search = gray[base[1] : base[1] + base[3], base[0] : base[0] + base[2]]
    found = (
        _textband_regions(search, settings)
        if settings.detector == "textband"
        else _contour_regions(search, settings)
    )
    absolute = [(base[0] + rx, base[1] + ry, rw, rh) for rx, ry, rw, rh in found]
    if absolute:
        return absolute[: settings.max_candidates]
    # Nothing found. Falling back to the whole ROI means "read the whole thing",
    # which is a reasonable last resort for OCR — but detect_only turns the
    # region straight into a position, and "the plate is the entire frame"
    # steers the machine at the middle of whatever it happens to be facing.
    # Better to report nothing and let the caller wait.
    return [] if settings.detect_only else [base]


def textband_mask(gray: np.ndarray, settings: ReadSettings) -> np.ndarray:
    """The binary mask the textband detector picks its regions out of.

    Separated so a live view can show exactly what the detector is working
    from. When detection comes and goes on a plate that is plainly in frame,
    this is where the answer is.
    """
    rect = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 9))
    blackhat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, rect)
    gradient = np.absolute(cv2.Sobel(blackhat, cv2.CV_32F, 1, 0, ksize=3))
    gradient = cv2.normalize(gradient, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    gradient = cv2.GaussianBlur(gradient, (5, 5), 0)
    closed = cv2.morphologyEx(gradient, cv2.MORPH_CLOSE, rect)
    _, mask = cv2.threshold(closed, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (settings.band_close_width, 11)),
    )
    return cv2.erode(mask, None, iterations=2)


def _textband_regions(gray: np.ndarray, settings: ReadSettings) -> list[Region]:
    """Find wide runs of vertical strokes — text lines, not rectangles."""
    mask = textband_mask(gray, settings)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    low, high = settings.aspect_range
    min_area = settings.min_area_ratio * gray.shape[0] * gray.shape[1]
    frame_width = gray.shape[1]
    narrow, wide = settings.width_ratio_range

    scene_mean = float(gray.mean()) or 1.0

    def plausible(box: Region) -> bool:
        x, y, w, h = box
        if not (w * h >= min_area and w >= 100 and h >= 16
                and low <= w / max(h, 1) <= high
                and narrow <= w / frame_width <= wide):
            return False
        band = gray[y : y + h, x : x + w]
        return band.mean() / scene_mean >= settings.min_relative_brightness

    regions = [
        box for box in (cv2.boundingRect(contour) for contour in contours)
        if plausible(box)
    ]
    regions.sort(key=lambda region: -region[2] * region[3])
    return regions


def _contour_regions(gray: np.ndarray, settings: ReadSettings) -> list[Region]:
    """Opt-in quad search: bilateral filter, Canny, four-vertex convex contours."""
    smoothed = cv2.bilateralFilter(gray, 11, 17, 17)
    edges = cv2.Canny(smoothed, 30, 200)
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    min_area = settings.min_area_ratio * gray.shape[0] * gray.shape[1]
    low, high = settings.aspect_range

    regions: list[Region] = []
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:20]:
        perimeter = cv2.arcLength(contour, True)
        approximation = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
        if len(approximation) != 4 or not cv2.isContourConvex(approximation):
            continue
        x, y, w, h = cv2.boundingRect(approximation)
        if w * h < min_area or not low <= w / max(h, 1) <= high:
            continue
        regions.append((x, y, w, h))
        if len(regions) >= settings.max_candidates:
            break
    return regions


class PlateReader:
    """Runs :class:`ReadSettings` over frames with one or two OCR engines.

    ``digit_engine`` matters more than it looks. The digit runs must be read by
    an **English** engine: measured here, ``-l kor`` with a digit whitelist
    returns *nothing at all* (the Korean LSTM's preferred path is Hangul and
    whitelisting prunes it away), while ``-l eng`` reads the same crop at
    confidence 96. Passing only one engine still works — it just gives up that
    accuracy on ``split``.
    """

    def __init__(
        self,
        engine: OcrEngine,
        settings: ReadSettings | None = None,
        digit_engine: OcrEngine | None = None,
    ) -> None:
        self.engine = engine
        self.digit_engine = digit_engine or engine
        self.settings = settings or ReadSettings()

    def read(self, frame: np.ndarray) -> ReadResult:
        started = cv2.getTickCount()
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        settings = self.settings
        regions = find_regions(gray, settings)

        attempts: list[Attempt] = []
        for region in regions if not settings.detect_only else ():
            x, y, w, h = region
            crop = gray[y : y + h, x : x + w]
            attempt = (
                self._read_split(region, crop)
                if settings.strategy == "split"
                else self._read_line(region, crop)
            )
            attempts.append(attempt)
            if attempt.accepted:
                break

        # Measured over the OCR region, not the whole frame: a frame-wide
        # estimate is dominated by floor texture and reports ~16 px for glyphs
        # that are actually 65 px, which sends you tuning the wrong thing.
        best_region = (attempts[-1].region if attempts else regions[0]) if regions else None
        glyph_gray = gray
        offset = None
        if best_region is not None:
            x, y, w, h = best_region
            glyph_gray = gray[y : y + h, x : x + w]
            offset = offset_from_centre(best_region, gray.shape, settings.hfov_deg)

        elapsed_ms = (cv2.getTickCount() - started) / cv2.getTickFrequency() * 1e3
        return ReadResult(
            gray=gray,
            focus=focus_score(glyph_gray),
            glyph_height=estimate_glyph_height(glyph_gray),
            attempts=attempts,
            elapsed_ms=elapsed_ms,
            offset=offset,
        )

    def _read_line(self, region: Region, crop: np.ndarray) -> Attempt:
        """One Tesseract pass over the whole region."""
        settings = self.settings
        prepared = prepare(crop, settings.preprocess, settings.upscale)
        result = self.engine.recognize(prepared, settings.psm)
        candidate = normalize(result.text, settings.pattern)
        accepted, reason = self._judge(candidate, result)
        return Attempt(region, prepared, result, candidate, accepted, reason)

    def _read_split(self, region: Region, crop: np.ndarray) -> Attempt:
        """Cut into characters, then recognise each group under its own rule."""
        settings = self.settings
        scale = min(
            settings.upscale, (MAX_SCALED_PIXELS / max(crop.shape[0] * crop.shape[1], 1)) ** 0.5
        )
        scaled = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        binary = drop_border_components(binarize(scaled))
        segments = segment_characters(binary)
        if len(segments) > max(settings.expected_lengths):
            segments = trim_to_length(binary, segments, max(settings.expected_lengths))

        empty = OcrResult("", -1.0, 0.0)
        if len(segments) not in settings.expected_lengths:
            reason = f"{len(segments)} characters, expected {settings.expected_lengths}"
            return Attempt(
                region, binary, empty, normalize("", settings.pattern), False, reason,
                segments=len(segments),
            )

        # The layout is fixed: the syllable is always fifth from the right.
        index = len(segments) - 5
        head, syllable_segment, tail = (
            segments[:index],
            segments[index],
            segments[index + 1 :],
        )

        head_result = self._read_digits(scaled, head)
        tail_result = self._read_digits(scaled, tail)
        syllable, match, from_tesseract = self._read_syllable(scaled, binary, syllable_segment)

        text = f"{head_result.text.strip()}{syllable}{tail_result.text.strip()}"
        confidences = [r.confidence for r in (head_result, tail_result) if r.confidence >= 0]
        result = OcrResult(
            text,
            sum(confidences) / len(confidences) if confidences else -1.0,
            head_result.elapsed_ms + tail_result.elapsed_ms,
        )
        candidate = normalize(text, settings.pattern)
        accepted, reason = self._judge(candidate, result)
        return Attempt(
            region, binary, result, candidate, accepted, reason,
            segments=len(segments), syllable=match, tesseract_syllable=from_tesseract,
        )

    def _read_digits(self, scaled: np.ndarray, segments: list[Segment]) -> OcrResult:
        """Read a run of digits, retrying under a different segmentation mode.

        The segmentation already knows how many digits are there, so a run that
        comes back with a different count is known-wrong. Two failures show up
        live: the run coming back empty (~20% of frames) and the band's padding
        pulling in a neighbour so three digits read as four.

        A retry is only *taken* when its digit count matches the segmentation.
        Reading each character separately was tried instead and is a trap — it
        always produces something, so it converts a rejection into a
        plausible-but-wrong plate. Measured over 145 live frames it published
        ``125가4568`` six times for a plate reading ``123가4568``. A wrong plate
        is worse than no plate, so a mismatch here is left to fail validation.
        """
        image = band(scaled, segments[0].start, segments[-1].end)
        best = self.digit_engine.recognize(image, self.settings.psm, DIGITS)
        if len(best.text.strip()) == len(segments):
            return OcrResult(best.text.strip(), best.confidence, best.elapsed_ms)

        elapsed_ms = best.elapsed_ms
        for psm in DIGIT_RETRY_PSMS:
            if psm == self.settings.psm:
                continue
            retry = self.digit_engine.recognize(image, psm, DIGITS)
            elapsed_ms += retry.elapsed_ms
            if len(retry.text.strip()) == len(segments):
                return OcrResult(retry.text.strip(), retry.confidence, elapsed_ms)
        return OcrResult(best.text.strip(), best.confidence, elapsed_ms)

    def _read_syllable(
        self, scaled: np.ndarray, binary: np.ndarray, segment: Segment
    ) -> tuple[str, SyllableMatch | None, str]:
        """Identify the middle glyph. Returns (chosen, template match, Tesseract's).

        Tesseract is asked either way, so the debug topic can show what it would
        have said — that is the evidence for keeping template matching the
        default, and the warning if the plate typeface ever changes.
        """
        glyph = band(scaled, segment.start, segment.end)
        from_tesseract = self.engine.recognize(glyph, psm=10).text.strip()
        match = match_syllable(band(binary, segment.start, segment.end, pad=0))

        if self.settings.syllable_source == "tesseract":
            chosen = from_tesseract
        elif match is not None and match.margin >= self.settings.min_syllable_margin:
            chosen = match.syllable
        elif match is not None:
            chosen = ""  # too close to call; let validation reject the frame
        else:
            chosen = from_tesseract
        # Tesseract's answer is still preferred when it is legal and template
        # matching found nothing to say.
        if not chosen and from_tesseract in PLATE_SYLLABLES:
            chosen = from_tesseract
        return chosen, match, from_tesseract

    def _judge(self, candidate: PlateCandidate, result: OcrResult) -> tuple[bool, str]:
        if not candidate.valid:
            return False, candidate.reason
        if self.settings.require_legal_syllable and not candidate.syllable_legal:
            return False, candidate.reason
        if 0.0 <= result.confidence < self.settings.min_confidence:
            return False, f"confidence {result.confidence:.0f} < {self.settings.min_confidence:.0f}"
        return True, "ok"
