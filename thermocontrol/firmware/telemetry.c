/**
 * @file telemetry.c
 * @brief Binary telemetry protocol implementation
 *
 * Implements encoding/decoding of binary packets.
 *
 * Key concepts:
 * =============
 * 1. Little-endian encoding for multi-byte values
 * 2. XOR checksum for error detection
 * 3. Strict validation on decode (security/robustness)
 *
 * Byte order (Endianness):
 * =======================
 * Most systems are little-endian (Intel x86, ARM, etc.):
 * - LSB (Least Significant Byte) first
 * - Example: 0x12345678 is stored as [0x78, 0x56, 0x34, 0x12]
 *
 * We use little-endian consistently (matches most modern hardware).
 * On big-endian systems, would need byte-swapping functions.
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#include "telemetry.h"
#include <string.h>     /* memcpy, memset */

/* ============================================================================
   HELPER FUNCTIONS FOR BYTE MANIPULATION
   ============================================================================

   These convert between native types (float, uint16_t) and byte buffers.
   We use inline helpers for clarity.
 */

/**
 * Write a uint16_t in little-endian format to a byte buffer
 *
 * Little-endian: LSB first
 * Example: 0x1234 → [0x34, 0x12]
 */
static inline void write_le16(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFF);         /* LSB */
    buf[1] = (uint8_t)((value >> 8) & 0xFF);  /* MSB */
}

/**
 * Read a uint16_t in little-endian format from byte buffer
 *
 * Reverses write_le16
 */
static inline uint16_t read_le16(const uint8_t *buf)
{
    return ((uint16_t)buf[0]) | (((uint16_t)buf[1]) << 8);
}

/**
 * Write a float (32-bit IEEE 754) to byte buffer
 *
 * Trick: Cast float pointer to uint8_t pointer and memcpy.
 * This works because we're just copying the bit pattern.
 * On little-endian systems (standard), bytes come out in the right order.
 */
static inline void write_float(uint8_t *buf, float value)
{
    memcpy(buf, &value, sizeof(float));
}

/**
 * Read a float from byte buffer
 *
 * Reverses write_float
 */
static inline float read_float(const uint8_t *buf)
{
    float value;
    memcpy(&value, buf, sizeof(float));
    return value;
}

/* ============================================================================
   CHECKSUM CALCULATION
   ============================================================================

   XOR checksum: Simple but effective
   Algorithm:
   1. Start with result = 0
   2. For each byte in frame (except final checksum byte):
        result ^= byte
   3. Final result is the checksum
   4. Append checksum to frame
   5. On receive, recalculate XOR including checksum - result must be 0!

   Why XOR?
   - Fast (bitwise operation)
   - Simple
   - Catches single-bit errors
   - Catches odd-number-of-bit errors in any position

   Why not CRC32?
   - CRC32 is stronger (catches more error patterns)
   - But uses ~1KB lookup table
   - Overkill for our short, reliable protocol
 */

/**
 * Calculate XOR checksum for a frame
 *
 * Iterates through all bytes and XORs them together.
 * If checksum is appended and included, final XOR should be 0
 * (because X ^ X = 0).
 *
 * @param data Buffer to checksum
 * @param length Number of bytes to checksum
 * @return XOR checksum value
 */
static uint8_t calculate_checksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/* ============================================================================
   INITIALIZATION
   ============================================================================ */

/**
 * int telemetry_init(void)
 *
 * Module initialization.
 * Currently just validates that protocol constants are correct.
 * (In production, you'd check buffer sizes, etc.)
 *
 * @return 0 on success, -1 on error
 */
int telemetry_init(void)
{
    /* Validate protocol constants
       These are compile-time checks, but good for documentation */
    if (TELEMETRY_MAX_FRAME_SIZE < TELEMETRY_HEADER_SIZE + TELEMETRY_CHECKSUM_SIZE) {
        return -1;  /* Frame size too small */
    }

    return 0;
}

/* ============================================================================
   ENCODING (Application → Binary Frame)
   ============================================================================

   telemetry_encode converts from human-friendly format to binary.

   Input:  telemetry_packet_t (float values)
   Output: telemetry_frame_t (bytes)

   Process:
   1. Write magic byte
   2. Write type (0x01 for telemetry)
   3. Write payload length (16 bytes)
   4. Write 4 floats
   5. Calculate and append XOR checksum
 */

/**
 * int telemetry_encode(const telemetry_packet_t *packet,
 *                      telemetry_frame_t *frame)
 *
 * Encode a packet into binary frame format.
 *
 * Frame structure:
 *   Offset  Size  Content
 *   0       1     Magic byte (0xTH)
 *   1       1     Type (0x01 = telemetry)
 *   2       2     Payload length (16, little-endian)
 *   4       4     Temperature (float, IEEE 754)
 *   8       4     Setpoint (float)
 *   12      4     Error (float)
 *   16      4     Command (float)
 *   20      1     XOR Checksum
 *
 * @param packet Input packet with data
 * @param frame Output frame (will be filled)
 * @return 0 on success, -1 on error
 */
int telemetry_encode(const telemetry_packet_t *packet,
                     telemetry_frame_t *frame)
{
    /* Input validation */
    if (!packet || !frame) {
        return -1;
    }

    uint8_t *buf = frame->data;
    size_t pos = 0;

    /* ========== HEADER ========== */

    /* Magic byte */
    buf[pos++] = TELEMETRY_MAGIC_BYTE;

    /* Type */
    buf[pos++] = TELEMETRY_TYPE_DATA;

    /* Payload length (16 bytes = 4 floats) */
    write_le16(&buf[pos], TELEMETRY_PAYLOAD_SIZE);
    pos += 2;

    /* ========== PAYLOAD ========== */

    /* Temperature (4 bytes, float) */
    write_float(&buf[pos], packet->temperature);
    pos += 4;

    /* Setpoint (4 bytes, float) */
    write_float(&buf[pos], packet->setpoint);
    pos += 4;

    /* Error (4 bytes, float) */
    write_float(&buf[pos], packet->error);
    pos += 4;

    /* Command (4 bytes, float) */
    write_float(&buf[pos], packet->command);
    pos += 4;

    /* ========== CHECKSUM ========== */

    /* Calculate XOR of all data bytes (not including the checksum itself) */
    uint8_t checksum = calculate_checksum(buf, pos);
    buf[pos++] = checksum;

    /* Set frame length */
    frame->length = pos;

    return 0;
}

/* ============================================================================
   DECODING (Binary Frame → Application)
   ============================================================================

   telemetry_decode reverses encoding.

   Input:  telemetry_frame_t (bytes)
   Output: telemetry_packet_t (float values)

   Includes validation:
   - Minimum length check
   - Magic byte verification
   - Type verification
   - Checksum validation
 */

/**
 * int telemetry_decode(const telemetry_frame_t *frame,
 *                      telemetry_packet_t *packet)
 *
 * Decode a binary frame into a packet.
 *
 * Validation steps:
 * 1. Check frame length (must be at least header + 1 byte)
 * 2. Verify magic byte (0xTH)
 * 3. Verify type (0x01 for telemetry)
 * 4. Read length field and validate
 * 5. Verify checksum matches
 * 6. If all checks pass, extract floats and fill packet
 *
 * @param frame Input frame with raw bytes
 * @param packet Output packet (will be filled)
 * @return 0 on success, -1 on error (bad checksum, format, etc.)
 */
int telemetry_decode(const telemetry_frame_t *frame,
                     telemetry_packet_t *packet)
{
    /* Input validation */
    if (!frame || !packet) {
        return -1;
    }

    const uint8_t *buf = frame->data;
    size_t pos = 0;

    /* Check minimum frame length
       Minimum: magic(1) + type(1) + length(2) + checksum(1) = 5 bytes
       But telemetry has payload, so minimum is TELEMETRY_MAX_FRAME_SIZE
    */
    if (frame->length < TELEMETRY_MAX_FRAME_SIZE) {
        return -1;  /* Frame too short */
    }

    /* ========== HEADER VALIDATION ========== */

    /* Check magic byte */
    if (buf[pos++] != TELEMETRY_MAGIC_BYTE) {
        return -1;  /* Wrong magic - not our protocol */
    }

    /* Check type */
    if (buf[pos++] != TELEMETRY_TYPE_DATA) {
        return -1;  /* Wrong type (expected telemetry) */
    }

    /* Read payload length */
    uint16_t payload_len = read_le16(&buf[pos]);
    pos += 2;

    /* Validate payload length */
    if (payload_len != TELEMETRY_PAYLOAD_SIZE) {
        return -1;  /* Wrong payload size */
    }

    /* ========== CHECKSUM VERIFICATION ========== */

    /* Verify checksum before parsing payload
       This is a security best practice: validate first, parse later.
       Prevents parsing corrupted data.
    */
    if (telemetry_verify_checksum(frame) != 1) {
        return -1;  /* Checksum failed */
    }

    /* ========== PAYLOAD EXTRACTION ========== */

    /* Temperature */
    packet->temperature = read_float(&buf[pos]);
    pos += 4;

    /* Setpoint */
    packet->setpoint = read_float(&buf[pos]);
    pos += 4;

    /* Error */
    packet->error = read_float(&buf[pos]);
    pos += 4;

    /* Command */
    packet->command = read_float(&buf[pos]);
    pos += 4;

    return 0;
}

/* ============================================================================
   CHECKSUM VERIFICATION
   ============================================================================ */

/**
 * int telemetry_verify_checksum(const telemetry_frame_t *frame)
 *
 * Verify that frame checksum is valid.
 *
 * Algorithm:
 * 1. Extract all data bytes (excluding checksum)
 * 2. Calculate XOR of data bytes
 * 3. Get stored checksum (last byte)
 * 4. XOR of (data XOR stored_checksum) should equal 0
 *    Because: (A ^ B) where B = XOR(A) gives 0
 *
 * @param frame Frame to verify
 * @return 1 if valid, 0 if invalid, -1 if frame too short
 */
int telemetry_verify_checksum(const telemetry_frame_t *frame)
{
    if (!frame || frame->length < TELEMETRY_CHECKSUM_SIZE) {
        return -1;
    }

    /* Calculate checksum of all bytes except the final checksum byte */
    size_t data_len = frame->length - TELEMETRY_CHECKSUM_SIZE;
    uint8_t calculated = calculate_checksum(frame->data, data_len);

    /* Get stored checksum (last byte) */
    uint8_t stored = frame->data[frame->length - 1];

    /* If calculated == stored, then:
       calculated ^ stored = 0
       This is the XOR property: A ^ A = 0
    */
    return (calculated == stored) ? 1 : 0;
}

/* ============================================================================
   UTILITIES
   ============================================================================ */

/**
 * const char *telemetry_type_name(uint8_t type)
 *
 * Return human-readable name for packet type.
 * Useful for debugging/logging.
 *
 * @param type Packet type code (0x01, 0x02, etc.)
 * @return Static string describing the type
 */
const char *telemetry_type_name(uint8_t type)
{
    switch (type) {
        case TELEMETRY_TYPE_DATA:
            return "Telemetry";
        case TELEMETRY_TYPE_COMMAND:
            return "Command";
        default:
            return "Unknown";
    }
}
