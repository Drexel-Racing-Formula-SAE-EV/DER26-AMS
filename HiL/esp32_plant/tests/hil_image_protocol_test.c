#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../common/ams_hil_image_protocol.h"

#define CHECK(condition)                                                     \
    do                                                                       \
    {                                                                        \
        if(!(condition))                                                     \
        {                                                                    \
            fprintf(stderr, "FAIL %s:%d: %s\n",                             \
                    __FILE__, __LINE__, #condition);                         \
            exit(1);                                                         \
        }                                                                    \
    } while(0)

static void test_address_contract(void)
{
    for(uint8_t segment = 0u; segment < 5u; segment++)
    {
        for(uint8_t first = 0u; first < 24u; first = (uint8_t)(first + 3u))
        {
            const uint8_t address =
                ams_hil_image_pack_address(segment, first);
            CHECK(ams_hil_image_address_segment(address) == segment);
            CHECK(ams_hil_image_address_index(address) == first);
        }
    }
}

static void test_crc_contract(void)
{
    static const uint8_t standard_vector[] = {
        '1', '2', '3', '4', '5', '6', '7', '8', '9'
    };
    uint32_t crc = ams_hil_image_crc32_init();
    for(size_t index = 0u;
        index < (sizeof(standard_vector) / sizeof(standard_vector[0]));
        index++)
    {
        crc = ams_hil_image_crc32_update_byte(crc, standard_vector[index]);
    }
    CHECK(ams_hil_image_crc32_finalize(crc) == UINT32_C(0xCBF43926));

    crc = ams_hil_image_crc32_init();
    crc = ams_hil_image_crc32_update_u16_be(crc, UINT16_C(0x1234));
    crc = ams_hil_image_crc32_update_u16_be(crc, UINT16_C(0xFEDC));
    CHECK(ams_hil_image_crc32_finalize(crc) == UINT32_C(0xFD6C5F88));
}

static void test_big_endian_contract(void)
{
    uint8_t encoded[4] = {0u};
    ams_hil_image_write_u32_be(encoded, UINT32_C(0x1234FEDC));
    CHECK(encoded[0] == 0x12u);
    CHECK(encoded[1] == 0x34u);
    CHECK(encoded[2] == 0xFEu);
    CHECK(encoded[3] == 0xDCu);
    CHECK(ams_hil_image_read_u32_be(encoded) == UINT32_C(0x1234FEDC));
}

int main(void)
{
    test_address_contract();
    test_crc_contract();
    test_big_endian_contract();
    puts("PASS hil_image_protocol_test");
    return 0;
}
