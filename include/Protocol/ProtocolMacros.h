#ifndef PROTOCOL_MACROS_H
#define PROTOCOL_MACROS_H


#include <cstdint>

#define CRC16 0x8005

/* Begin copy-paste from https://gist.github.com/2666368 */
static uint16_t __gen_crc16(const uint8_t *data, uint16_t size) {
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == nullptr)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> (7 - bits_read)) & 1;

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;
    }

    return out;
}

#define STANDARD_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF } __attribute__((packed));
#define RELIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t crc; } __attribute__((packed));
#define IDENTIFIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t id; } __attribute__((packed));
#define RELIABLE_IDENTIFIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t id; uint16_t crc; }__attribute__((packed));
#define MAKE_RELIABLE(PACKET) (PACKET).crc = __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)
#define IS_RELIABLE(PACKET) (PACKET).crc == __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)

#endif /* PROTOCOL_MACROS_H */
