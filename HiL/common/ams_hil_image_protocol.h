/*
 * Atomic ESP32 plant -> AMS ADBMS-image protocol for Classic CAN.
 *
 * Multi-byte values are big-endian. A complete generation is:
 *   START (0x212), all cell/temp triplets (0x210/0x211), COMMIT (0x212).
 * The receiver must not publish staged values until every expected channel is
 * present and the canonical image CRC matches.
 */

#ifndef AMS_HIL_IMAGE_PROTOCOL_H_
#define AMS_HIL_IMAGE_PROTOCOL_H_

#include <stdint.h>

#define AMS_HIL_CAN_ID_MEAS          0x200U
#define AMS_HIL_CAN_ID_TRUTH         0x201U
#define AMS_HIL_CAN_ID_SUMMARY       0x202U
#define AMS_HIL_CAN_ID_CELL_SAMPLE   0x210U
#define AMS_HIL_CAN_ID_TEMP_SAMPLE   0x211U
#define AMS_HIL_CAN_ID_IMAGE_CONTROL 0x212U
#define AMS_HIL_CAN_ID_CTRL          0x300U

#define AMS_HIL_IMAGE_PROTOCOL_VERSION 1U
#define AMS_HIL_IMAGE_CTRL_START       1U
#define AMS_HIL_IMAGE_CTRL_COMMIT      2U
#define AMS_HIL_IMAGE_SAMPLE_STRIDE    3U

#define AMS_HIL_IMAGE_ADDRESS_SEGMENT_SHIFT 5U
#define AMS_HIL_IMAGE_ADDRESS_SEGMENT_MASK  0xE0U
#define AMS_HIL_IMAGE_ADDRESS_INDEX_MASK    0x1FU

/* START: version, opcode, generation, segment count, total cells,
 * total temperatures, expected cell frames, expected temperature frames. */
#define AMS_HIL_IMAGE_START_VERSION_OFFSET       0U
#define AMS_HIL_IMAGE_START_OPCODE_OFFSET        1U
#define AMS_HIL_IMAGE_START_GENERATION_OFFSET    2U
#define AMS_HIL_IMAGE_START_SEGMENTS_OFFSET      3U
#define AMS_HIL_IMAGE_START_CELLS_OFFSET         4U
#define AMS_HIL_IMAGE_START_TEMPERATURES_OFFSET  5U
#define AMS_HIL_IMAGE_START_CELL_FRAMES_OFFSET   6U
#define AMS_HIL_IMAGE_START_TEMP_FRAMES_OFFSET   7U

/* COMMIT: version, opcode, generation, reserved, CRC32 big-endian. */
#define AMS_HIL_IMAGE_COMMIT_VERSION_OFFSET      0U
#define AMS_HIL_IMAGE_COMMIT_OPCODE_OFFSET       1U
#define AMS_HIL_IMAGE_COMMIT_GENERATION_OFFSET   2U
#define AMS_HIL_IMAGE_COMMIT_RESERVED_OFFSET     3U
#define AMS_HIL_IMAGE_COMMIT_CRC_OFFSET          4U

/* DATA: generation, packed segment/index, three big-endian int16 samples. */
#define AMS_HIL_IMAGE_DATA_GENERATION_OFFSET     0U
#define AMS_HIL_IMAGE_DATA_ADDRESS_OFFSET        1U
#define AMS_HIL_IMAGE_DATA_VALUES_OFFSET         2U

static inline uint8_t ams_hil_image_pack_address(uint8_t segment,
                                                 uint8_t first_index)
{
    return (uint8_t)((uint8_t)(segment <<
                               AMS_HIL_IMAGE_ADDRESS_SEGMENT_SHIFT) |
                     (first_index & AMS_HIL_IMAGE_ADDRESS_INDEX_MASK));
}

static inline uint8_t ams_hil_image_address_segment(uint8_t address)
{
    return (uint8_t)((address & AMS_HIL_IMAGE_ADDRESS_SEGMENT_MASK) >>
                     AMS_HIL_IMAGE_ADDRESS_SEGMENT_SHIFT);
}

static inline uint8_t ams_hil_image_address_index(uint8_t address)
{
    return (uint8_t)(address & AMS_HIL_IMAGE_ADDRESS_INDEX_MASK);
}

static inline uint32_t ams_hil_image_crc32_init(void)
{
    return UINT32_C(0xFFFFFFFF);
}

static inline uint32_t ams_hil_image_crc32_update_byte(uint32_t crc,
                                                       uint8_t value)
{
    crc ^= value;
    for(uint8_t bit = 0U; bit < 8U; bit++)
    {
        uint32_t mask = (uint32_t)(0U - (crc & 1U));
        crc = (crc >> 1U) ^ (UINT32_C(0xEDB88320) & mask);
    }
    return crc;
}

static inline uint32_t ams_hil_image_crc32_update_u16_be(uint32_t crc,
                                                         uint16_t value)
{
    crc = ams_hil_image_crc32_update_byte(crc, (uint8_t)(value >> 8U));
    return ams_hil_image_crc32_update_byte(crc, (uint8_t)value);
}

static inline uint32_t ams_hil_image_crc32_finalize(uint32_t crc)
{
    return crc ^ UINT32_C(0xFFFFFFFF);
}

static inline uint32_t ams_hil_image_read_u32_be(const uint8_t data[4])
{
    return ((uint32_t)data[0] << 24U) |
           ((uint32_t)data[1] << 16U) |
           ((uint32_t)data[2] << 8U) |
           (uint32_t)data[3];
}

static inline void ams_hil_image_write_u32_be(uint8_t data[4], uint32_t value)
{
    data[0] = (uint8_t)(value >> 24U);
    data[1] = (uint8_t)(value >> 16U);
    data[2] = (uint8_t)(value >> 8U);
    data[3] = (uint8_t)value;
}

#endif /* AMS_HIL_IMAGE_PROTOCOL_H_ */
