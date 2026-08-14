#!/usr/bin/env python3
"""Turn raw OCR output into a validated Korean licence plate string.

Pure standard library — no OpenCV, no Tesseract, no ROS 2. Everything here is
unit-testable without a camera, which is the point of keeping it in its own
module: the CI job that has no OpenCV can still run these tests.

Plate formats covered by the default pattern:

    12가3456    two leading digits  (issued before 2019)
    123가4567   three leading digits (issued from 2019)

Old regional plates (``서울12가3456``) are deliberately *not* covered — this
repository's camera looks at a single modern plate.
"""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass

# The syllables that may legally appear in the middle of a Korean plate. There
# are exactly 40, which makes membership a very cheap false-positive filter:
# tesseract happily returns arbitrary Hangul, and almost none of it is legal.
PLATE_SYLLABLES = frozenset(
    "가나다라마"  # private, passenger
    "거너더러머"
    "버서어저"
    "고노도로모"
    "보소오조"
    "구누두루무"
    "부수우주"
    "바사아자"  # commercial (taxi / bus / freight)
    "배"  # parcel delivery
    "하허호"  # rental
)

DEFAULT_PLATE_PATTERN = r"^\d{2,3}[가-힣]\d{4}$"

# Characters tesseract inserts that carry no information: whitespace, the
# separators a plate graphic may show, and the invisible ones. U+3164 (Hangul
# filler) and U+FFA0 (halfwidth Hangul filler) are emitted by Korean models and
# look exactly like a space in a log — they are the reason a plate that reads
# correctly on screen can fail the regex.
_NOISE = re.compile(r"[\s\-‐-―·‧​-‏ㅤﾠ.,_|]+")

# Latin glyphs tesseract returns for digits. Applied *positionally* — only to
# the slots the pattern says are digits — because applying them everywhere
# would corrupt the syllable slot.
DIGIT_CONFUSIONS = {
    "O": "0",
    "o": "0",
    "D": "0",
    "Q": "0",
    "I": "1",
    "l": "1",
    "i": "1",
    "|": "1",
    "Z": "2",
    "z": "2",
    "E": "3",
    "A": "4",
    "S": "5",
    "s": "5",
    "G": "6",
    "b": "6",
    "T": "7",
    "B": "8",
    "g": "9",
    "q": "9",
}


@dataclass(frozen=True)
class PlateCandidate:
    """The outcome of normalising one OCR result.

    ``valid`` means the text matched the plate pattern. ``syllable_legal`` is a
    second, independent check: the middle character is one of the 40 syllables
    that actually appear on Korean plates. A caller may require both — that
    combination is what makes a false positive rare.
    """

    raw: str  # exactly what the OCR engine returned
    cleaned: str  # NFC-normalised, noise stripped
    text: str  # cleaned + positional digit repair; "" when unusable
    valid: bool
    syllable_legal: bool
    reason: str  # why it was rejected; "ok" when valid

    @property
    def accepted(self) -> bool:
        return self.valid and self.syllable_legal


def clean(raw: str) -> str:
    """NFC-normalise and strip everything that carries no information.

    The NFC step is load-bearing. ``[가-힣]`` matches only precomposed syllables
    (U+AC00–U+D7A3), but a Korean OCR model can return decomposed jamo
    (U+1100 block) that renders identically. Without this the regex fails on
    text that looks perfect in the log.
    """
    return _NOISE.sub("", unicodedata.normalize("NFC", raw))


def repair_digits(text: str) -> str:
    """Map Latin look-alikes back to digits in the slots that must be digits.

    The syllable slot is left untouched. Its index follows from the length: a
    7-character plate is ``DD S DDDD`` (index 2), an 8-character one is
    ``DDD S DDDD`` (index 3). Any other length is returned unchanged — there is
    no slot layout to reason about.
    """
    if len(text) not in (7, 8):
        return text
    syllable_index = len(text) - 5
    return "".join(
        char if i == syllable_index else DIGIT_CONFUSIONS.get(char, char)
        for i, char in enumerate(text)
    )


def normalize(raw: str, pattern: str = DEFAULT_PLATE_PATTERN) -> PlateCandidate:
    """Clean, repair and validate one OCR result.

    Digit repair is a *fallback*, not a first pass: text that already matches is
    returned untouched. Repairing unconditionally would corrupt a correct read
    under any pattern whose slots are not the Korean plate's — ``ABC1234``
    becomes ``48C1234``.
    """
    cleaned = clean(raw)
    if not cleaned:
        return PlateCandidate(raw, cleaned, "", False, False, "empty")

    text = cleaned
    if not re.match(pattern, text):
        text = repair_digits(cleaned)
        if not re.match(pattern, text):
            return PlateCandidate(
                raw, cleaned, text, False, False, f"no match ({len(text)} chars)"
            )

    syllables = [char for char in text if "가" <= char <= "힣"]
    if not syllables:
        # A pattern with no Hangul slot has nothing to validate.
        return PlateCandidate(raw, cleaned, text, True, True, "ok")
    if len(syllables) > 1:
        return PlateCandidate(
            raw, cleaned, text, True, False, f"{len(syllables)} syllables, expected 1"
        )

    legal = syllables[0] in PLATE_SYLLABLES
    reason = "ok" if legal else f"illegal syllable {syllables[0]!r}"
    return PlateCandidate(raw, cleaned, text, True, legal, reason)
