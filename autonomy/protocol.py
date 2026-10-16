"""Pack UDP messages. UDP sends separate messages without a delivery guarantee."""

from dataclasses import dataclass
from enum import IntEnum
import math
import struct

import numpy as np


MAGIC = b"ODUK"
VERSION = 1
HELLO = 1
DEPTH = 2
COMMAND = 3
STATE = 4
MAX_DATAGRAM = 65507
PREFIX = struct.Struct("!4sBBH")
DEPTH_FIELDS = struct.Struct("!IdHH")
DEPTH_HEADER_SIZE = PREFIX.size + DEPTH_FIELDS.size
COMMAND_FIELDS = struct.Struct("!QfffB")
STATE_FIELDS = struct.Struct("!QB9fB")
COMMAND_SIZE = PREFIX.size + COMMAND_FIELDS.size
STATE_HEADER_SIZE = PREFIX.size + STATE_FIELDS.size
MAX_REASON_BYTES = 96
MAX_SEQUENCE = (1 << 64) - 1


class Mode(IntEnum):
    """Identify the source and status of walking speeds."""

    MANUAL = 0
    AUTO = 1
    PAUSED = 2
    FALLEN = 3


@dataclass(frozen=True)
class Command:
    """Store speeds in metres per second and turn rate in radians per second."""

    sequence: int
    forward: float
    sideways: float
    turn: float
    stop: bool = False

    @property
    def speeds(self):
        return self.forward, self.sideways, self.turn


@dataclass(frozen=True)
class State:
    """Store applied speeds, sensor readings, and the last mode change reason."""

    sequence: int
    mode: Mode
    applied: tuple
    gyro: tuple
    accelero: tuple
    reason: str


def newer_sequence(sequence: int, previous: int) -> bool:
    """Compare sequence numbers across one counter wrap."""
    return 0 < ((sequence - previous) & MAX_SEQUENCE) < (1 << 63)


def _check_sequence(sequence):
    if not isinstance(sequence, int) or isinstance(sequence, bool) or not 0 <= sequence <= MAX_SEQUENCE:
        raise ValueError("Sequence number is not valid.")


def _check_vector(values):
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("Use three finite values for each vector.")


def validate_command(command: Command) -> None:
    """Reject invalid command fields before use."""
    if not isinstance(command, Command):
        raise ValueError("Command type is not valid.")
    _check_sequence(command.sequence)
    _check_vector(command.speeds)
    if not isinstance(command.stop, bool):
        raise ValueError("Stop flag is not valid.")


def pack_command(command: Command) -> bytes:
    """Pack one walking command."""
    validate_command(command)
    try:
        fields = COMMAND_FIELDS.pack(command.sequence, *command.speeds, int(command.stop))
    except (OverflowError, struct.error) as error:
        raise ValueError("Command values exceed the message limits.") from error
    return PREFIX.pack(MAGIC, VERSION, COMMAND, COMMAND_SIZE) + fields


def unpack_command(packet: bytes) -> Command:
    """Read and check one walking command."""
    _check_header(packet, COMMAND, COMMAND_SIZE)
    if len(packet) != COMMAND_SIZE:
        raise ValueError("Command length is not valid.")
    sequence, forward, sideways, turn, stop = COMMAND_FIELDS.unpack_from(packet, PREFIX.size)
    if stop not in (0, 1):
        raise ValueError("Stop flag is not valid.")
    command = Command(sequence, forward, sideways, turn, bool(stop))
    validate_command(command)
    return command


def pack_state(state: State) -> bytes:
    """Pack one walking state message."""
    _check_sequence(state.sequence)
    mode = Mode(state.mode)
    for values in (state.applied, state.gyro, state.accelero):
        _check_vector(values)
    reason = state.reason.encode("utf-8")
    if len(reason) > MAX_REASON_BYTES:
        raise ValueError("Mode change reason is too long.")
    try:
        fields = STATE_FIELDS.pack(state.sequence, mode, *state.applied, *state.gyro,
                                   *state.accelero, len(reason))
    except (OverflowError, struct.error) as error:
        raise ValueError("State values exceed the message limits.") from error
    return PREFIX.pack(MAGIC, VERSION, STATE, STATE_HEADER_SIZE) + fields + reason


def unpack_state(packet: bytes) -> State:
    """Read and check one walking state message."""
    _check_header(packet, STATE, STATE_HEADER_SIZE)
    fields = STATE_FIELDS.unpack_from(packet, PREFIX.size)
    length = fields[-1]
    if length > MAX_REASON_BYTES or len(packet) != STATE_HEADER_SIZE + length:
        raise ValueError("State length is not valid.")
    try:
        reason = packet[STATE_HEADER_SIZE:].decode("utf-8")
    except UnicodeDecodeError as error:
        raise ValueError("Mode change reason is not valid text.") from error
    state = State(fields[0], Mode(fields[1]), fields[2:5], fields[5:8], fields[8:11], reason)
    for values in (state.applied, state.gyro, state.accelero):
        _check_vector(values)
    return state


@dataclass(frozen=True)
class DepthFrame:
    """Store the frame number, Pi time in seconds, and depth in millimetres."""

    number: int
    timestamp: float
    depth_mm: np.ndarray


def _check_header(packet: bytes, kind: int, size: int) -> None:
    if len(packet) < size or len(packet) > MAX_DATAGRAM:
        raise ValueError("Message length is not valid.")
    magic, version, actual_kind, header_size = PREFIX.unpack_from(packet)
    if (magic, version, actual_kind, header_size) != (MAGIC, VERSION, kind, size):
        raise ValueError("Message header is not valid.")


def pack_hello() -> bytes:
    """Make a request to receive depth frames."""
    return PREFIX.pack(MAGIC, VERSION, HELLO, PREFIX.size)


def unpack_hello(packet: bytes) -> None:
    """Check a request to receive depth frames."""
    _check_header(packet, HELLO, PREFIX.size)
    if len(packet) != PREFIX.size:
        raise ValueError("Hello length is not valid.")


def pack_depth(number: int, timestamp: float, depth_mm: np.ndarray) -> bytes:
    """Pack one depth frame into one message. Zero means no valid reading."""
    depth = np.asarray(depth_mm)
    if depth.ndim != 2 or depth.dtype.kind != "u" or depth.dtype.itemsize != 2:
        raise ValueError("Use a two-dimensional uint16 depth array.")
    height, width = depth.shape
    if not (0 < width <= 65535 and 0 < height <= 65535):
        raise ValueError("Depth size is not valid.")
    if DEPTH_HEADER_SIZE + depth.size * 2 > MAX_DATAGRAM:
        raise ValueError("Depth frame exceeds one UDP message.")
    if not 0 <= number <= 0xFFFFFFFF:
        raise ValueError("Frame number is not valid.")
    if not math.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Pi time is not valid.")
    header = PREFIX.pack(MAGIC, VERSION, DEPTH, DEPTH_HEADER_SIZE)
    fields = DEPTH_FIELDS.pack(number, timestamp, width, height)
    return header + fields + depth.astype(">u2", copy=False).tobytes(order="C")


def unpack_depth(packet: bytes) -> DepthFrame:
    """Read and check one depth message."""
    _check_header(packet, DEPTH, DEPTH_HEADER_SIZE)
    number, timestamp, width, height = DEPTH_FIELDS.unpack_from(packet, PREFIX.size)
    if not width or not height or len(packet) != DEPTH_HEADER_SIZE + width * height * 2:
        raise ValueError("Depth length does not match its size.")
    if not math.isfinite(timestamp) or timestamp < 0:
        raise ValueError("Pi time is not valid.")
    depth = np.frombuffer(packet, dtype=">u2", offset=DEPTH_HEADER_SIZE)
    return DepthFrame(number, timestamp, depth.astype(np.uint16).reshape(height, width))
