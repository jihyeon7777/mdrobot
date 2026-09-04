"""The downlink wire format.

This is the line that turns a drill on. The board checks the checksum and
drops anything that does not add up, so the shape of it is worth pinning: a
field count the firmware does not expect fails that check on every line at
once, and the machine stops taking commands with nothing to say why.
"""

import pytest

from mdrobot_rc_bridge.rc_reader import (
    COMMAND_LIMITS,
    COMMAND_NAMES,
    NUM_COMMANDS,
    encode_command,
)


def fields(line: bytes) -> list[int]:
    return [int(v) for v in line.decode().strip().split(",")]


def test_every_command_has_a_declared_range():
    # COMMAND_MIN/MAX are built from this dict BY NAME, so a field without an
    # entry raises at import rather than reaching a drill unclamped.
    assert set(COMMAND_LIMITS) == set(COMMAND_NAMES)


def test_the_checksum_is_the_sum_of_the_fields():
    line = fields(encode_command([10, 1, 1, -1, 1]))
    assert line[:-1] == [10, 1, 1, -1, 1]
    assert line[-1] == sum(line[:-1]) == 12


def test_a_negative_field_still_sums():
    # actuator is the one that goes negative, and it must not be treated as
    # unsigned on its way into the checksum.
    assert fields(encode_command([0, 0, 0, -1, 0]))[-1] == -1


def test_the_field_order_is_the_one_the_board_expects():
    assert COMMAND_NAMES == ("lift", "brake", "drill", "actuator", "solenoid")


@pytest.mark.parametrize("count", [NUM_COMMANDS - 1, NUM_COMMANDS + 1])
def test_the_wrong_number_of_fields_is_refused(count):
    # Sending a count the firmware does not expect would checksum-fail every
    # line and the machine would quietly stop responding. Fail here instead.
    with pytest.raises(ValueError, match=f"{NUM_COMMANDS} values"):
        encode_command([0] * count)


def test_the_terminator_is_configurable():
    # Which terminator the board's parser wants was never established, so it
    # stays a setting rather than an assumption.
    assert encode_command([0] * NUM_COMMANDS, terminator="\n").endswith(b"\n")
    assert encode_command([0] * NUM_COMMANDS).endswith(b"\r\n")
