#!/usr/bin/env python3
"""Tesseract behind a two-implementation interface.

Depends on Pillow and NumPy, deliberately *not* on OpenCV — an image arrives as
a 2-D grayscale ``numpy`` array and leaves as PNG bytes or a PIL image, so this
module can be exercised without a camera in the picture.

Two backends:

* :class:`TesserocrEngine` — the ``python3-tesserocr`` binding. Preferred.
  It initialises the engine and loads ``kor.traineddata`` **once**; the CLI
  pays that 150–400 ms on a Raspberry Pi 5 on *every* call, which dominates the
  actual recognition and makes a settings sweep unbearable.
* :class:`CliEngine` — shells out to ``tesseract``. No Python binding required,
  so ``probe.py`` still runs on a machine that only did
  ``apt install tesseract-ocr tesseract-ocr-kor``. The image is piped through
  stdin (``-`` as both input and output name), so there is no temp file.

Neither engine is thread-safe. Keep one per thread.

``--oem 1`` (LSTM only) is not configurable: Ubuntu ships LSTM-only
``.traineddata``, so the legacy engine would simply fail. ``user_defined_dpi``
is always set — without it Tesseract guesses the resolution per image and its
internal scaling, and therefore the output, drifts between frames.
"""

from __future__ import annotations

import io
import shutil
import subprocess
import time
from dataclasses import dataclass
from typing import Protocol

import numpy as np
from PIL import Image

DEFAULT_LANG = "kor"
DEFAULT_PSM = 7
DEFAULT_DPI = 300

# A whitelist is worth using only for a small ASCII set. Constraining the LSTM's
# beam search to a large non-ASCII set (say the 40 legal plate syllables) prunes
# the model's preferred path and commonly returns nothing at all — so the digit
# runs get one and the Hangul syllable never does.
DIGITS = "0123456789"


@dataclass(frozen=True)
class OcrResult:
    """One recognition pass."""

    text: str
    confidence: float  # 0–100; -1.0 when the engine reported none
    elapsed_ms: float


class OcrEngine(Protocol):
    """What :mod:`reader` needs from an OCR implementation."""

    name: str
    lang: str

    def recognize(
        self, image: np.ndarray, psm: int = DEFAULT_PSM, whitelist: str = ""
    ) -> OcrResult: ...

    def close(self) -> None: ...


def _to_pil(image: np.ndarray) -> Image.Image:
    if image.ndim != 2:
        raise ValueError(f"expected a 2-D grayscale array, got shape {image.shape}")
    return Image.fromarray(np.ascontiguousarray(image, dtype=np.uint8), mode="L")


class TesserocrEngine:
    """Persistent ``PyTessBaseAPI``. One engine init for the whole process."""

    name = "tesserocr"

    def __init__(self, lang: str = DEFAULT_LANG, dpi: int = DEFAULT_DPI) -> None:
        import tesserocr  # imported lazily so the CLI backend works without it

        self.lang = lang
        self._api = tesserocr.PyTessBaseAPI(lang=lang, oem=tesserocr.OEM.LSTM_ONLY)
        self._api.SetVariable("user_defined_dpi", str(dpi))
        self._psm: int | None = None
        self._whitelist = ""

    def recognize(
        self, image: np.ndarray, psm: int = DEFAULT_PSM, whitelist: str = ""
    ) -> OcrResult:
        started = time.perf_counter()
        if psm != self._psm:
            # tesserocr.PSM is a constant namespace, not a callable enum, so the
            # plain int is what SetPageSegMode wants.
            self._api.SetPageSegMode(psm)
            self._psm = psm
        if whitelist != self._whitelist:
            self._api.SetVariable("tessedit_char_whitelist", whitelist)
            self._whitelist = whitelist
        self._api.SetImage(_to_pil(image))
        text = self._api.GetUTF8Text()
        confidence = float(self._api.MeanTextConf())
        return OcrResult(text, confidence, (time.perf_counter() - started) * 1e3)

    def close(self) -> None:
        self._api.End()


class CliEngine:
    """``tesseract`` as a subprocess, image in via stdin, TSV out via stdout."""

    name = "cli"

    def __init__(self, lang: str = DEFAULT_LANG, dpi: int = DEFAULT_DPI) -> None:
        if shutil.which("tesseract") is None:
            raise RuntimeError(
                "tesseract not found: sudo apt install tesseract-ocr tesseract-ocr-kor"
            )
        self.lang = lang
        self._dpi = dpi

    def recognize(
        self, image: np.ndarray, psm: int = DEFAULT_PSM, whitelist: str = ""
    ) -> OcrResult:
        buffer = io.BytesIO()
        _to_pil(image).save(buffer, format="PNG")
        command = [
            "tesseract", "-", "-",
            "-l", self.lang,
            "--psm", str(psm),
            "--oem", "1",
            "-c", f"user_defined_dpi={self._dpi}",
        ]  # fmt: skip
        if whitelist:
            command += ["-c", f"tessedit_char_whitelist={whitelist}"]
        command.append("tsv")

        started = time.perf_counter()
        completed = subprocess.run(
            command, input=buffer.getvalue(), capture_output=True, check=False
        )
        elapsed_ms = (time.perf_counter() - started) * 1e3
        if completed.returncode != 0:
            stderr = completed.stderr.decode("utf-8", "replace").strip()
            raise RuntimeError(f"tesseract failed ({completed.returncode}): {stderr}")

        text, confidence = _parse_tsv(completed.stdout.decode("utf-8", "replace"))
        return OcrResult(text, confidence, elapsed_ms)

    def close(self) -> None:
        pass


def _parse_tsv(tsv: str) -> tuple[str, float]:
    """Pull the words and their mean confidence out of Tesseract's TSV output.

    Only rows that carry text matter; Tesseract emits a row per page, block,
    paragraph and line as well, all with ``conf`` of -1 and an empty text field.
    """
    words: list[str] = []
    confidences: list[float] = []
    for line in tsv.splitlines()[1:]:  # row 0 is the header
        fields = line.split("\t")
        if len(fields) < 12:
            continue
        word = fields[11].strip()
        if not word:
            continue
        words.append(word)
        try:
            confidence = float(fields[10])
        except ValueError:
            continue
        if confidence >= 0.0:
            confidences.append(confidence)
    mean = sum(confidences) / len(confidences) if confidences else -1.0
    return " ".join(words), mean


def available_languages() -> list[str]:
    """Languages Tesseract can actually load, via whichever path is available."""
    try:
        import tesserocr

        return sorted(tesserocr.get_languages()[1])
    except ImportError:
        pass
    if shutil.which("tesseract") is None:
        return []
    completed = subprocess.run(
        ["tesseract", "--list-langs"], capture_output=True, check=False
    )
    if completed.returncode != 0:
        return []
    lines = completed.stdout.decode("utf-8", "replace").splitlines()
    return sorted(line.strip() for line in lines[1:] if line.strip())


def make_engine(
    backend: str = "auto", lang: str = DEFAULT_LANG, dpi: int = DEFAULT_DPI
) -> OcrEngine:
    """Build an engine. ``auto`` prefers tesserocr and falls back to the CLI.

    Every language in a ``+``-joined ``lang`` must be installed, so this is also
    where the "empty OCR results forever" failure mode gets turned into a
    message that names the missing apt package.
    """
    installed = available_languages()
    if not installed:
        raise RuntimeError(
            "no Tesseract language data found: "
            "sudo apt install tesseract-ocr tesseract-ocr-kor"
        )
    missing = [part for part in lang.split("+") if part not in installed]
    if missing:
        raise RuntimeError(
            f"language {'+'.join(missing)!r} not installed (have: {', '.join(installed)}); "
            f"try: sudo apt install tesseract-ocr-{missing[0]}"
        )

    if backend == "tesserocr":
        return TesserocrEngine(lang, dpi)
    if backend == "cli":
        return CliEngine(lang, dpi)
    if backend != "auto":
        raise ValueError(f"unknown ocr backend {backend!r}: use auto, tesserocr or cli")
    try:
        return TesserocrEngine(lang, dpi)
    except ImportError:
        return CliEngine(lang, dpi)
