"""Unit tests for the OCR text normaliser. No camera, no Tesseract."""

import unicodedata

import pytest

from mdrobot_plate_ocr.normalize import (
    DEFAULT_PLATE_PATTERN,
    PLATE_SYLLABLES,
    clean,
    normalize,
    repair_digits,
)


def test_plate_syllable_set_has_the_forty_legal_syllables():
    assert len(PLATE_SYLLABLES) == 40
    for syllable in "가허배자무":
        assert syllable in PLATE_SYLLABLES
    # 'ㄱ' style consonants and syllables outside the plate alphabet are out.
    for syllable in "강한택":
        assert syllable not in PLATE_SYLLABLES


class TestClean:
    def test_strips_whitespace_and_separators(self):
        assert clean(" 12 가 3456 \n") == "12가3456"
        assert clean("12-가-3456") == "12가3456"
        assert clean("12·가·3456") == "12가3456"

    def test_strips_invisible_characters(self):
        # U+200B zero width space, U+3164 Hangul filler, U+FFA0 halfwidth filler.
        assert clean("12​가ㅤ3456ﾠ") == "12가3456"

    def test_composes_decomposed_hangul(self):
        decomposed = unicodedata.normalize("NFD", "12가3456")
        assert decomposed != "12가3456"  # guard: the input really is NFD
        assert clean(decomposed) == "12가3456"

    def test_empty_input(self):
        assert clean("") == ""
        assert clean("  \n\t ") == ""


class TestRepairDigits:
    def test_maps_latin_lookalikes_in_digit_slots(self):
        assert repair_digits("I2가34S6") == "12가3456"
        assert repair_digits("OB가1234") == "08가1234"

    def test_leaves_the_syllable_slot_alone(self):
        # 'S' sits in the syllable slot of a 7-char plate and must survive, so
        # the caller can see the real failure instead of a fabricated digit.
        assert repair_digits("12S3456") == "12S3456"
        # Same for the 8-char layout, where the syllable is at index 3.
        assert repair_digits("123S4567") == "123S4567"

    def test_ignores_strings_with_no_plate_layout(self):
        assert repair_digits("OI") == "OI"
        assert repair_digits("") == ""


class TestNormalize:
    @pytest.mark.parametrize("text", ["12가3456", "123가4567", "01하9999"])
    def test_accepts_well_formed_plates(self, text):
        result = normalize(text)
        assert result.accepted
        assert result.text == text
        assert result.reason == "ok"

    def test_accepts_a_plate_that_needed_cleaning_and_repair(self):
        result = normalize("  I2 가 34S6 \n")
        assert result.accepted
        assert result.text == "12가3456"
        assert result.raw == "  I2 가 34S6 \n"
        assert result.cleaned == "I2가34S6"

    def test_rejects_empty(self):
        result = normalize("   ")
        assert not result.accepted
        assert result.reason == "empty"

    @pytest.mark.parametrize("text", ["1가3456", "12가345", "12345678", "가나다라마바사"])
    def test_rejects_wrong_shape(self, text):
        result = normalize(text)
        assert not result.valid
        assert not result.accepted
        assert "no match" in result.reason

    def test_valid_shape_with_an_illegal_syllable_is_reported_not_hidden(self):
        # '강' is a real Hangul syllable and matches the pattern, but no Korean
        # plate carries it. The candidate stays visible for the debug topic.
        result = normalize("12강3456")
        assert result.valid
        assert not result.syllable_legal
        assert not result.accepted
        assert "illegal syllable" in result.reason

    def test_custom_pattern_is_honoured(self):
        result = normalize("ABC1234", pattern=r"^[A-Z]{3}\d{4}$")
        assert result.accepted
        # Digit repair must not fire on text that already matches, or the
        # letters would be mangled into digits.
        assert result.text == "ABC1234"
        assert not normalize("ABC1234", pattern=DEFAULT_PLATE_PATTERN).valid

    def test_repair_does_not_touch_an_already_matching_plate(self):
        # 'O' in a digit slot would normally become '0'; here the raw text
        # already satisfies a pattern that allows it, so it is left alone.
        result = normalize("1O가3456", pattern=r"^.{2}[가-힣]\d{4}$")
        assert result.text == "1O가3456"

    def test_rejects_text_carrying_more_than_one_syllable(self):
        result = normalize("12가나3456", pattern=r"^\d{2}[가-힣]{2}\d{4}$")
        assert result.valid
        assert not result.accepted
        assert "2 syllables" in result.reason
