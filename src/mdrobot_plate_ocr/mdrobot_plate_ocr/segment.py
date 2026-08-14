#!/usr/bin/env python3
"""Cut a one-line plate crop into individual characters.

This exists because a single Tesseract pass over ``123가4568`` cannot be given
the constraint that matters. ``tessedit_char_whitelist`` is global, so there is
no way to say "digits here, one Hangul syllable there" — and measured on this
robot's plate, one pass either reads the digits and turns ``가`` into a digit,
or reads ``가`` and corrupts a digit. Splitting first lets each group be
recognised under the rule that actually applies to it.

Three steps, each answering a failure the previous one caused:

1. :func:`binarize` — adaptive threshold with the window scaled to the crop.
   A fixed 31 px block is sensible on a raw crop and useless once the crop is
   upscaled 3x, where it traces stroke edges and leaves the strokes hollow.
2. :func:`drop_border_components` — erase the plate's printed frame. It touches
   every character, so a column projection over an unstripped crop sees one
   continuous run and reports a single character. Morphological line removal
   was tried first and misses it: the plate is photographed at a slight angle,
   so no column holds a long enough vertical run.
3. :func:`segment_characters` — vertical ink projection, *not* connected
   components, because a Hangul syllable is made of disconnected parts. ``가``
   is ``ㄱ`` plus ``ㅏ`` with a visible gap, and components would return two
   characters. Runs closer than ``merge_ratio`` of the crop height are merged:
   measured here the intra-syllable gap was 3 px and the smallest gap between
   characters was 28 px.

:func:`trim_to_length` then uses the one thing known a priori — a plate carries
seven or eight characters — to discard whatever border fragments survived.
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Segment:
    """A character's horizontal extent, in pixels of the crop it came from."""

    start: int
    end: int

    @property
    def width(self) -> int:
        return self.end - self.start


def binarize(gray: np.ndarray) -> np.ndarray:
    """Adaptive threshold with the window scaled to the glyphs, ink white."""
    block = int(gray.shape[0] * 0.9) | 1  # adaptiveThreshold demands an odd block
    return cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, max(block, 15), 15
    )


def drop_border_components(
    binary: np.ndarray, touch: int = 2, span_ratio: float = 0.80
) -> np.ndarray:
    """Erase components that both touch the crop edge and stretch across it.

    That pair of conditions is what separates a printed border from a glyph: no
    character reaches the edge *and* spans most of the crop.
    """
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    height, width = binary.shape[:2]
    kept = np.zeros_like(binary)
    for index in range(1, count):
        x, y, w, h = (int(stats[index, key]) for key in range(4))
        touches = x <= touch or y <= touch or x + w >= width - touch or y + h >= height - touch
        stretches = w >= span_ratio * width or h >= span_ratio * height
        if not (touches and stretches):
            kept[labels == index] = 255
    return kept


def segment_characters(
    binary: np.ndarray,
    ink_ratio: float = 0.02,
    min_width_ratio: float = 0.10,
    merge_ratio: float = 0.10,
) -> list[Segment]:
    """Split a binarised single-line crop into character extents, left to right."""
    height = binary.shape[0]
    inked = (binary > 0).sum(axis=0) > max(1, int(ink_ratio * height))

    runs: list[list[int]] = []
    start: int | None = None
    for index, on in enumerate(inked):
        if on and start is None:
            start = index
        elif not on and start is not None:
            runs.append([start, index])
            start = None
    if start is not None:
        runs.append([start, len(inked)])

    runs = [run for run in runs if run[1] - run[0] >= min_width_ratio * height]
    if not runs:
        return []

    merged = [runs[0]]
    for begin, end in runs[1:]:
        if begin - merged[-1][1] < merge_ratio * height:
            merged[-1][1] = end  # same character, e.g. the two halves of 가
        else:
            merged.append([begin, end])
    return [Segment(begin, end) for begin, end in merged]


def ink_mass(binary: np.ndarray, segment: Segment) -> int:
    """How much ink a segment holds — the tie-breaker when trimming."""
    return int((binary[:, segment.start : segment.end] > 0).sum())


def trim_to_length(
    binary: np.ndarray, segments: list[Segment], target: int
) -> list[Segment]:
    """Drop the faintest end segments until only ``target`` remain.

    A plate has a known character count, so anything extra is a border fragment
    — and fragments are always at one end or the other. Trimming by ink mass
    rather than width is deliberate: the digit ``1`` is the narrowest real
    character on the plate and a width rule discards it.
    """
    trimmed = list(segments)
    while len(trimmed) > target:
        if ink_mass(binary, trimmed[0]) <= ink_mass(binary, trimmed[-1]):
            trimmed.pop(0)
        else:
            trimmed.pop()
    return trimmed


def band(image: np.ndarray, start: int, end: int, pad: int = 12) -> np.ndarray:
    """Slice columns ``start``..``end`` with a margin — Tesseract wants one."""
    return image[:, max(0, start - pad) : min(image.shape[1], end + pad)]
