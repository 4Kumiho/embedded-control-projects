"""
@file parser.py
@brief Binary telemetry protocol parser

Decodes binary telemetry packets from firmware into Python structures.

Protocol specification:
========================

Frame format:
    Byte 0:    0x54            (magic byte = 'T')
    Byte 1:    TYPE            (0x01 = telemetry, 0x02 = command)
    Byte 2-3:  LEN (LE2)       (payload length, little-endian)
    Byte 4+:   PAYLOAD         (actual data)
    Byte n:    XOR_CHECKSUM    (XOR of all bytes 0 to n-1)

Example telemetry frame (21 bytes):
    [0x54][0x01][0x10 0x00][float T][float Tset][float err][float cmd][XOR]
     magic  type  length    T(4B)    Tset(4B)    err(4B)    cmd(4B)    check

@author Andrea (ALTEN Training)
@date 2026-03-12
"""

import struct
from enum import IntEnum
from dataclasses import dataclass
from typing import Optional, Tuple

# ============================================================================
#   CONSTANTS
# ============================================================================

class PacketType(IntEnum):
    """Telemetry packet type identifiers"""
    TELEMETRY = 0x01
    COMMAND = 0x02


MAGIC_BYTE = 0x54              # 'T' in ASCII
HEADER_SIZE = 4                # magic(1) + type(1) + length(2)
CHECKSUM_SIZE = 1
TELEMETRY_PAYLOAD_SIZE = 16    # 4 floats × 4 bytes each
MAX_FRAME_SIZE = HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE + CHECKSUM_SIZE  # 21 bytes


# ============================================================================
#   DATA STRUCTURES
# ============================================================================

@dataclass
class TelemetryPacket:
    """
    Represents a single telemetry reading from firmware.

    Attributes:
        temperature (float): Measured room temperature (°C)
        setpoint (float): Desired setpoint temperature (°C)
        error (float): Control error (setpoint - temperature)
        command (float): Heater command output (-100 to +100)
        timestamp (float): When this data was collected (seconds)
    """
    temperature: float
    setpoint: float
    error: float
    command: float
    timestamp: float = 0.0

    def __repr__(self) -> str:
        """Human-readable representation"""
        return (f"TelemetryPacket(T={self.temperature:.2f}°C, "
                f"Tset={self.setpoint:.2f}°C, "
                f"err={self.error:.2f}°C, "
                f"cmd={self.command:.1f}%)")


# ============================================================================
#   CHECKSUM FUNCTIONS
# ============================================================================

def calculate_xor_checksum(data: bytes) -> int:
    """
    Calculate XOR checksum of byte buffer.

    Algorithm:
        result = 0
        for each byte in data:
            result ^= byte
        return result

    Properties:
        - If you XOR the checksum back in: (data XOR checksum) = 0
        - Simple to compute (one byte operation per input byte)
        - Catches single-bit errors
        - Fast (no lookup tables needed)

    Args:
        data: Byte buffer to checksum

    Returns:
        XOR checksum value (0-255)

    Example:
        >>> data = bytes([0x54, 0x01, 0x10, 0x00])
        >>> checksum = calculate_xor_checksum(data)
        >>> # Verify: 0x54 ^ 0x01 ^ 0x10 ^ 0x00 ^ checksum == 0
    """
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def verify_checksum(frame: bytes) -> bool:
    """
    Verify that frame checksum is valid.

    The checksum is the last byte. If valid:
        XOR(all bytes except checksum) == checksum byte

    Args:
        frame: Complete frame including checksum

    Returns:
        True if checksum is valid, False otherwise

    Example:
        >>> frame = bytes([0x54, 0x01, ...data..., checksum_byte])
        >>> if verify_checksum(frame):
        ...     print("Frame is valid")
    """
    if len(frame) < CHECKSUM_SIZE:
        return False

    # Calculate checksum of all bytes except the final checksum byte
    data_part = frame[:-1]
    calculated_checksum = calculate_xor_checksum(data_part)

    # Compare with stored checksum (last byte)
    stored_checksum = frame[-1]

    return calculated_checksum == stored_checksum


# ============================================================================
#   PARSING FUNCTIONS
# ============================================================================

def parse_telemetry_frame(frame: bytes) -> Optional[TelemetryPacket]:
    """
    Parse a complete telemetry frame into a TelemetryPacket.

    Validation steps:
        1. Check minimum length (header + payload + checksum = 21 bytes)
        2. Verify magic byte (0x54)
        3. Verify packet type (0x01 for telemetry)
        4. Verify checksum (XOR integrity check)
        5. Extract floats from payload

    Args:
        frame: Raw bytes received from firmware

    Returns:
        TelemetryPacket if valid, None if parsing fails

    Example:
        >>> received_bytes = b'...'  # 21 bytes from firmware
        >>> packet = parse_telemetry_frame(received_bytes)
        >>> if packet:
        ...     print(f"Temperature: {packet.temperature:.2f}°C")
        ... else:
        ...     print("Invalid packet")
    """
    # Step 1: Validate length
    if len(frame) < MAX_FRAME_SIZE:
        print(f"ERROR: Frame too short ({len(frame)} < {MAX_FRAME_SIZE})")
        return None

    # Step 2: Verify magic byte
    if frame[0] != MAGIC_BYTE:
        print(f"ERROR: Bad magic byte (0x{frame[0]:02x} != 0x{MAGIC_BYTE:02x})")
        return None

    # Step 3: Verify packet type
    packet_type = frame[1]
    if packet_type != PacketType.TELEMETRY:
        print(f"ERROR: Wrong packet type (0x{packet_type:02x}, expected 0x{PacketType.TELEMETRY:02x})")
        return None

    # Step 4: Verify checksum before parsing payload
    #         This is a security best practice: validate first, parse later
    if not verify_checksum(frame):
        print("ERROR: Checksum failed - frame corrupted")
        return None

    # Step 5: Extract floats from payload
    #         Payload starts at byte 4, is 16 bytes (4 floats × 4 bytes each)
    #         Format: '<' = little-endian, '4f' = 4 floats

    try:
        payload = frame[HEADER_SIZE : HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE]

        # Unpack 4 floats in little-endian format
        # struct.unpack returns a tuple: (float, float, float, float)
        temperature, setpoint, error, command = struct.unpack('<4f', payload)

        # Create and return packet
        packet = TelemetryPacket(
            temperature=temperature,
            setpoint=setpoint,
            error=error,
            command=command,
            timestamp=0.0  # Will be filled by caller with current time
        )

        return packet

    except struct.error as e:
        print(f"ERROR: Failed to unpack floats: {e}")
        return None


def parse_stream(data: bytes) -> list[TelemetryPacket]:
    """
    Parse multiple frames from a byte stream.

    Searches for magic bytes (0x54) and tries to extract complete frames.
    Handles partial frames, multiple frames in stream, etc.

    Algorithm:
        1. Scan for magic byte (0x54)
        2. Check if we have full frame (21 bytes)
        3. If yes, try to parse as frame
        4. If valid, add to results
        5. Skip to next potential frame and repeat

    Args:
        data: Continuous byte stream (may contain multiple frames)

    Returns:
        List of successfully parsed TelemetryPacket objects
        (Invalid frames are skipped with warnings)

    Example:
        >>> stream = b'...junk...[frame1]...junk...[frame2]...junk...'
        >>> packets = parse_stream(stream)
        >>> print(f"Parsed {len(packets)} packets")
    """
    packets = []
    pos = 0

    while pos < len(data):
        # Search for next magic byte
        try:
            magic_pos = data.index(MAGIC_BYTE, pos)
        except ValueError:
            # No more magic bytes found
            break

        # Check if we have enough bytes for a complete frame
        remaining = len(data) - magic_pos
        if remaining < MAX_FRAME_SIZE:
            # Not enough bytes for complete frame
            break

        # Extract potential frame
        frame = data[magic_pos : magic_pos + MAX_FRAME_SIZE]

        # Try to parse as telemetry
        packet = parse_telemetry_frame(frame)
        if packet:
            packets.append(packet)

        # Move to next position (skip this frame)
        pos = magic_pos + MAX_FRAME_SIZE

    return packets


# ============================================================================
#   UTILITIES
# ============================================================================

def encode_command(setpoint: float) -> bytes:
    """
    Encode a command packet to send setpoint to firmware.

    Creates a binary frame:
        [0x54][0x02][0x04 0x00][float setpoint][checksum]

    Args:
        setpoint: New setpoint temperature (°C)

    Returns:
        Complete 9-byte frame ready to send to firmware

    Example:
        >>> frame = encode_command(25.0)
        >>> send_to_firmware(frame)  # e.g., via socket or serial
    """
    # Frame structure
    frame = bytearray()

    # Header
    frame.append(MAGIC_BYTE)           # 0x54
    frame.append(PacketType.COMMAND)   # 0x02
    frame.extend(struct.pack('<H', 4)) # Length = 4 bytes (1 float)

    # Payload (1 float)
    frame.extend(struct.pack('<f', setpoint))

    # Checksum
    checksum = calculate_xor_checksum(bytes(frame))
    frame.append(checksum)

    return bytes(frame)


def format_packet(packet: TelemetryPacket) -> str:
    """
    Format a packet for display/logging.

    Args:
        packet: TelemetryPacket to format

    Returns:
        String representation: "T=21.50°C, Tset=25.00°C, err=3.50°C, cmd=45.2%"

    Example:
        >>> packet = parse_telemetry_frame(frame_bytes)
        >>> print(format_packet(packet))
        T=21.50°C, Tset=25.00°C, err=3.50°C, cmd=45.2%
    """
    return (f"T={packet.temperature:7.2f}°C | "
            f"Tset={packet.setpoint:6.2f}°C | "
            f"err={packet.error:6.2f}°C | "
            f"cmd={packet.command:6.1f}%")


# ============================================================================
#   UNIT TESTS (Run with: python -m pytest parser.py)
# ============================================================================

def test_xor_checksum():
    """Test checksum calculation"""
    data = bytes([0x54, 0x01, 0x10, 0x00])
    checksum = calculate_xor_checksum(data)
    # Verify: 0x54 ^ 0x01 ^ 0x10 ^ 0x00 = 0x45
    assert checksum == 0x45, f"Expected 0x45, got 0x{checksum:02x}"


def test_checksum_verify():
    """Test checksum verification"""
    # Valid frame with checksum
    data = bytes([0x54, 0x01, 0x10, 0x00])
    checksum = calculate_xor_checksum(data)
    frame = data + bytes([checksum])

    assert verify_checksum(frame), "Valid checksum should pass"

    # Invalid: corrupted byte
    corrupted = data + bytes([checksum ^ 0xFF])
    assert not verify_checksum(corrupted), "Corrupted checksum should fail"


def test_parse_telemetry():
    """Test parsing a valid telemetry frame"""
    # Construct a valid frame manually
    frame = bytearray()
    frame.append(0x54)                              # Magic
    frame.append(0x01)                              # Type telemetry
    frame.extend(struct.pack('<H', 16))             # Length = 16
    frame.extend(struct.pack('<4f', 21.5, 25.0, 3.5, 50.0))  # Payload
    checksum = calculate_xor_checksum(bytes(frame))
    frame.append(checksum)

    # Parse
    packet = parse_telemetry_frame(bytes(frame))
    assert packet is not None, "Valid frame should parse"
    assert abs(packet.temperature - 21.5) < 0.01
    assert abs(packet.setpoint - 25.0) < 0.01
    assert abs(packet.error - 3.5) < 0.01
    assert abs(packet.command - 50.0) < 0.01


def test_parse_invalid_magic():
    """Test parsing frame with bad magic byte"""
    frame = bytes([0xFF, 0x01] + [0x00] * 19)  # Bad magic
    packet = parse_telemetry_frame(frame)
    assert packet is None, "Bad magic should fail"


def test_encode_command():
    """Test command encoding"""
    frame = encode_command(25.5)
    assert len(frame) == 9, f"Expected 9 bytes, got {len(frame)}"
    assert frame[0] == 0x54, "Bad magic"
    assert frame[1] == 0x02, "Bad type"
    assert verify_checksum(frame), "Bad checksum"


if __name__ == "__main__":
    # Run tests
    print("Running unit tests...")
    test_xor_checksum()
    test_checksum_verify()
    test_parse_telemetry()
    test_parse_invalid_magic()
    test_encode_command()
    print("All tests passed! ✓")
