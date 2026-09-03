"""Shared packet builder.

Lives in conftest so the test modules do not have to import each other: the
test directories carry no __init__.py, so a module named test_protocol here
would collide with the one in src/mdrobot/test.
"""

import struct

import pytest


def make_packet(kind: int, a: int, b: int, c: int, d: int) -> bytes:
    """A well-formed HWT901B packet with a correct checksum."""
    body = bytes([0x55, kind]) + struct.pack("<hhhh", a, b, c, d)
    return body + bytes([sum(body) & 0xFF])


@pytest.fixture
def build():
    return make_packet
