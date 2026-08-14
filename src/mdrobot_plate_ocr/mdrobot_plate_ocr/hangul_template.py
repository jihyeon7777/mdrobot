#!/usr/bin/env python3
"""Identify the plate's Hangul syllable by matching it against rendered glyphs.

Tesseract's ``kor`` model is trained on document typefaces, and a plate uses a
bold condensed face that looks nothing like them. Measured on this robot's
plate, ``--psm 10`` on the isolated syllable returned ``기`` for a ``가`` — not
merely wrong but not even a syllable any Korean plate carries.

Template matching wins here for a reason that has nothing to do with being
cleverer: the answer set is **40 syllables**, so a classifier that can only
return a legal syllable starts with an enormous advantage over one choosing
from ~2400. Rendering all 40 in several installed fonts and taking the best
normalised cross-correlation recovered ``가`` with a clear margin over the
runner-up.

Both the query and the templates are reduced to the same canonical form — tight
crop to the ink, then a fixed square — so stroke weight and position stop
mattering and only shape is compared.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .normalize import PLATE_SYLLABLES

# Several faces, because the plate's typeface is none of them and the closest
# match differs per syllable. Missing files are skipped.
TEMPLATE_FONTS = (
    "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
    "/usr/share/fonts/truetype/nanum/NanumBarunGothicBold.ttf",
    "/usr/share/fonts/truetype/nanum/NanumSquareB.ttf",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
)

GLYPH_SIZE = 64
_RENDER_SIZE = 160


@dataclass(frozen=True)
class SyllableMatch:
    """The winner, its score, and how far ahead of second place it was."""

    syllable: str
    score: float
    margin: float
    runner_up: str


def canonical(binary: np.ndarray) -> np.ndarray | None:
    """Tight-crop a binary glyph to its ink and scale it to a fixed square."""
    rows, columns = np.nonzero(binary)
    if len(columns) == 0:
        return None
    tight = binary[rows.min() : rows.max() + 1, columns.min() : columns.max() + 1]
    return cv2.resize(tight, (GLYPH_SIZE, GLYPH_SIZE), interpolation=cv2.INTER_AREA)


@lru_cache(maxsize=1)
def _templates() -> tuple[tuple[str, np.ndarray], ...]:
    """Render every legal syllable in every available font. Cached per process."""
    fonts = [path for path in TEMPLATE_FONTS if Path(path).exists()]
    if not fonts:
        raise RuntimeError(f"no template font found; looked for {TEMPLATE_FONTS}")

    rendered: list[tuple[str, np.ndarray]] = []
    for syllable in sorted(PLATE_SYLLABLES):
        for path in fonts:
            font = ImageFont.truetype(path, _RENDER_SIZE * 6 // 10)
            image = Image.new("L", (_RENDER_SIZE, _RENDER_SIZE), 0)
            draw = ImageDraw.Draw(image)
            draw.text(
                (_RENDER_SIZE // 2, _RENDER_SIZE // 2), syllable, font=font, fill=255, anchor="mm"
            )
            glyph = canonical(np.array(image))
            if glyph is not None:
                rendered.append((syllable, glyph.astype(np.float32)))
    return tuple(rendered)


def match_syllable(binary_glyph: np.ndarray) -> SyllableMatch | None:
    """Best of the 40 legal syllables for one binarised glyph, or ``None``."""
    query = canonical(binary_glyph)
    if query is None:
        return None
    query = query.astype(np.float32)

    best: dict[str, float] = {}
    for syllable, template in _templates():
        score = float(cv2.matchTemplate(query, template, cv2.TM_CCOEFF_NORMED)[0][0])
        if score > best.get(syllable, -2.0):
            best[syllable] = score

    ranked = sorted(best.items(), key=lambda item: -item[1])
    if not ranked:
        return None
    (winner, score), (second, runner_score) = ranked[0], ranked[1]
    return SyllableMatch(winner, score, score - runner_score, second)
