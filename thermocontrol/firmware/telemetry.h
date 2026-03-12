/**
 * @file telemetry.h
 * @brief Binary telemetry protocol for data transmission
 *
 * Protocol for sending/receiving structured binary data between firmware and GUI.
 *
 * Why binary instead of text?
 * ============================
 * Text (JSON, CSV):
 * - Human-readable (easy debugging)
 * - Large overhead (each number is many bytes)
 * - Variable length (unpredictable timing)
 * - Example: {"temp":21.45,"setpoint":25.0,"error":3.55}  ≈ 50 bytes
 *
 * Binary (our protocol):
 * - Compact (each float is 4 bytes, overhead minimal)
 * - Fixed length (predictable timing)
 * - Fast parsing (just memcpy)
 * - Checksummed (detects transmission errors)
 * - Example: [0xTH][0x01][0x10 0x00][16 bytes payload][checksum]  ≈ 20 bytes
 *
 * Protocol Specification:
 * =======================
 *
 * Frame format:
 *   Byte 0:    0xTH          (magic byte - identifies ThermoControl packets)
 *   Byte 1:    TYPE          (packet type: 0x01=telemetry, 0x02=command)
 *   Byte 2-3:  LEN           (payload length in bytes, little-endian)
 *   Byte 4+:   PAYLOAD       (actual data)
 *   Byte n:    XOR_CHECKSUM  (XOR of all bytes 0 to n-1)
 *
 * Example telemetry frame:
 *   [0xTH][0x01][0x10 0x00][float T][float Tset][float err][float cmd][XOR]
 *   ^^^^^^ magic
 *          ^^^^ type = telemetry
 *              ^^^^^^^^ length = 16 bytes (4 floats)
 *                       ^^^^^^^^^^^^^^^^^^^^^^^^ payload (4 x 4-byte floats)
 *                                               ^^^ checksum
 *
 * Packet types:
 * - 0x01: Telemetry (firmware → GUI) - temperature data
 * - 0x02: Command (GUI → firmware) - new setpoint
 *
 * @author Andrea (ALTEN Training)
 * @date 2026-03-12
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>   /* uint8_t, uint32_t */
#include <stddef.h>   /* size_t */

/**
 * @defgroup Telemetry Telemetry Protocol
 * @{
 */

/* ============================================================================
   PROTOCOL CONSTANTS
   ============================================================================ */

/**
 * @name Protocol Identifiers
 * @{
 */

/** Magic byte to identify ThermoControl packets
    Reduces chance of false packet detection. Any 0xTH byte starts a frame.
 */
#define TELEMETRY_MAGIC_BYTE    0x54  /* 'T' in ASCII */

/** Packet type: Telemetry (firmware → GUI)
    Payload: [float T][float Tset][float error][float cmd]
 */
#define TELEMETRY_TYPE_DATA     0x01

/** Packet type: Command (GUI → firmware)
    Payload: [float setpoint]
 */
#define TELEMETRY_TYPE_COMMAND  0x02

/** @} */

/**
 * @name Frame Structure Constants
 * @{
 */

/** Header length: magic (1) + type (1) + length (2) = 4 bytes */
#define TELEMETRY_HEADER_SIZE   4

/** Checksum length: 1 byte (at end) */
#define TELEMETRY_CHECKSUM_SIZE 1

/** Telemetry payload size: 4 floats = 16 bytes
    Layout: T_read (4B) + T_setpoint (4B) + error (4B) + command (4B)
 */
#define TELEMETRY_PAYLOAD_SIZE  16

/** Maximum frame size: header + payload + checksum = 4 + 16 + 1 = 21 bytes */
#define TELEMETRY_MAX_FRAME_SIZE (TELEMETRY_HEADER_SIZE + TELEMETRY_PAYLOAD_SIZE + TELEMETRY_CHECKSUM_SIZE)

/** @} */

/* ============================================================================
   DATA STRUCTURES
   ============================================================================ */

/**
 * @struct telemetry_packet_t
 * @brief Structured representation of a telemetry packet
 *
 * Used to hold unpacked/parsed data in a convenient form.
 * This is what the application works with (high-level).
 *
 * The binary protocol is a serialized version of this.
 */
typedef struct {
    float temperature;     ///< Measured temperature (°C)
    float setpoint;        ///< Setpoint temperature (°C)
    float error;           ///< Error: setpoint - temperature (°C)
    float command;         ///< Control output to heater (-100 to +100)
    uint32_t timestamp;    ///< When this data was collected (ms)
} telemetry_packet_t;

/**
 * @struct telemetry_frame_t
 * @brief Binary frame buffer
 *
 * Holds the raw binary packet data.
 * This is what gets transmitted over the wire.
 *
 * Usage:
 * 1. Application fills telemetry_packet_t
 * 2. telemetry_encode() packs into telemetry_frame_t
 * 3. Frame bytes are sent to GUI
 * 4. GUI calls telemetry_decode() to unpack back to telemetry_packet_t
 */
typedef struct {
    uint8_t data[TELEMETRY_MAX_FRAME_SIZE];  ///< Raw bytes
    size_t length;                            ///< Number of bytes used
} telemetry_frame_t;

/* ============================================================================
   FUNCTION DECLARATIONS
   ============================================================================ */

/**
 * @brief Initialize telemetry module
 *
 * Prepares the module for use. Currently just validates configuration,
 * but good practice to call once at startup.
 *
 * @return 0 on success, -1 on configuration error
 *
 * @example
 *   if (telemetry_init() != 0) {
 *       printf("Telemetry init failed\n");
 *       return 1;
 *   }
 */
int telemetry_init(void);

/**
 * @brief Encode a telemetry packet into binary frame
 *
 * Takes a high-level telemetry_packet_t and converts it to binary
 * frame format with:
 * - Magic byte
 * - Type field
 * - Length field
 * - Payload (4 floats, little-endian)
 * - XOR checksum
 *
 * Algorithm:
 * 1. Write magic byte (0xTH)
 * 2. Write type (0x01 for telemetry)
 * 3. Write length (16 bytes in little-endian)
 * 4. Write 4 floats (temperature, setpoint, error, command)
 * 5. Calculate XOR of all bytes
 * 6. Append checksum
 * 7. Return frame
 *
 * @param packet Input: telemetry_packet_t with data to encode
 * @param frame Output: telemetry_frame_t that will be filled
 * @return 0 on success, -1 on error
 *
 * @example
 *   telemetry_packet_t pkt;
 *   pkt.temperature = 21.5f;
 *   pkt.setpoint = 25.0f;
 *   pkt.error = 3.5f;
 *   pkt.command = 50.0f;
 *
 *   telemetry_frame_t frame;
 *   if (telemetry_encode(&pkt, &frame) == 0) {
 *       // Send frame.data[0..frame.length-1] to GUI
 *       send_to_gui(frame.data, frame.length);
 *   }
 */
int telemetry_encode(const telemetry_packet_t *packet,
                     telemetry_frame_t *frame);

/**
 * @brief Decode a binary frame into telemetry packet
 *
 * Reverses telemetry_encode(). Takes raw bytes and extracts:
 * - Verifies magic byte and type
 * - Checks frame length
 * - Extracts 4 floats from payload
 * - Verifies XOR checksum (detects transmission errors)
 *
 * Algorithm:
 * 1. Check frame length ≥ minimum required
 * 2. Check magic byte (byte 0 = 0xTH)
 * 3. Check type (byte 1 = 0x01)
 * 4. Read length field (bytes 2-3)
 * 5. Verify payload length matches expected
 * 6. Extract 4 floats from payload
 * 7. Calculate XOR of data bytes
 * 8. Compare with stored checksum (must match!)
 * 9. If all checks pass, fill output packet
 *
 * Error detection:
 * - Returns -1 if magic byte wrong (not our protocol)
 * - Returns -1 if checksum fails (transmission error)
 * - Returns -1 if frame malformed (too short, wrong type)
 *
 * @param frame Input: telemetry_frame_t with raw bytes
 * @param packet Output: telemetry_packet_t to fill with decoded data
 * @return 0 on success, -1 on error (bad checksum or format)
 *
 * @example
 *   telemetry_frame_t frame;
 *   frame.data = received_bytes;  // From GUI
 *   frame.length = num_bytes_received;
 *
 *   telemetry_packet_t pkt;
 *   if (telemetry_decode(&frame, &pkt) == 0) {
 *       // Successfully decoded
 *       printf("T = %.2f°C\n", pkt.temperature);
 *   } else {
 *       printf("Bad packet (checksum error or wrong type)\n");
 *   }
 */
int telemetry_decode(const telemetry_frame_t *frame,
                     telemetry_packet_t *packet);

/**
 * @brief Verify checksum of a frame
 *
 * Computes XOR of all data bytes and compares with stored checksum.
 *
 * XOR checksum:
 * - Start with 0
 * - XOR each byte: result ^= byte
 * - Final result should equal stored checksum byte
 *
 * Good for:
 * - Simple error detection (catches single-bit errors)
 * - Fast computation (O(n) time, O(1) space)
 *
 * Not good for:
 * - Multiple bit errors (might cancel out)
 * - Doesn't detect multi-bit patterns
 * (Use CRC32 for higher reliability, but overkill here)
 *
 * @param frame Frame to check
 * @return 1 if checksum valid, 0 if invalid, -1 if frame too short
 *
 * @example
 *   if (telemetry_verify_checksum(&frame) != 1) {
 *       printf("Checksum error - packet corrupted\n");
 *   }
 */
int telemetry_verify_checksum(const telemetry_frame_t *frame);

/**
 * @brief Get human-readable description of packet type
 *
 * For debugging/logging.
 *
 * @param type Packet type code (0x01, 0x02, etc.)
 * @return Static string (e.g., "Telemetry", "Command", "Unknown")
 *
 * @example
 *   printf("Received packet type: %s\n", telemetry_type_name(0x01));
 *   // Output: "Telemetry"
 */
const char *telemetry_type_name(uint8_t type);

/**
 * @} End of Telemetry group
 */

#endif /* TELEMETRY_H */
