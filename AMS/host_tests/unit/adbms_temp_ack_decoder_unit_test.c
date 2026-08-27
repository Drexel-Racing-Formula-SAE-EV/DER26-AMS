#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "ext_drivers/adbms6830_functions.h"

static unsigned failures;

#define CHECK(expr) do { \
    if(!(expr)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        failures++; \
    } \
} while(0)

int main(void)
{
    const uint8_t mux_4c_ack[RX_DATA] =
        {0x67u, 0x98u, 0x77u, 0x00u, 0x1Fu, 0xFFu, 0u, 0u};
    const uint8_t mux_4f_nack[RX_DATA] =
        {0x6Fu, 0x9Eu, 0x7Fu, 0x00u, 0x1Fu, 0xFFu, 0u, 0u};
    uint8_t packet[RX_DATA] = {0};

    CHECK(adbms6830_comm_address_acknowledged(mux_4c_ack));
    CHECK(adbms6830_comm_data_acknowledged(mux_4c_ack));
    CHECK(adbms6830_comm_write_acknowledged(mux_4c_ack));

    CHECK(!adbms6830_comm_address_acknowledged(mux_4f_nack));
    CHECK(!adbms6830_comm_data_acknowledged(mux_4f_nack));
    CHECK(!adbms6830_comm_write_acknowledged(mux_4f_nack));

    packet[0] = 0x67u;
    packet[2] = 0x07u;
    CHECK(adbms6830_comm_data_acknowledged(packet));
    CHECK(adbms6830_comm_write_acknowledged(packet));

    packet[2] = 0x7Fu;
    CHECK(!adbms6830_comm_data_acknowledged(packet));
    CHECK(!adbms6830_comm_write_acknowledged(packet));

    packet[2] = 0x67u;
    CHECK(!adbms6830_comm_data_acknowledged(packet));
    CHECK(!adbms6830_comm_write_acknowledged(packet));

    CHECK(!adbms6830_comm_address_acknowledged(NULL));
    CHECK(!adbms6830_comm_data_acknowledged(NULL));
    CHECK(!adbms6830_comm_write_acknowledged(NULL));

    if(failures != 0u)
    {
        fprintf(stderr, "ADBMS TEMP ACK DECODER TESTS FAILED: %u\n", failures);
        return 1;
    }

    puts("PASS ADBMS temperature-bus ACK readback decoder");
    return 0;
}
