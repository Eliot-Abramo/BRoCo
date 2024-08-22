#ifndef PROTOCOL_MACROS_H
#define PROTOCOL_MACROS_H


#include <cstdint>

static uint16_t __gen_crc16(const uint8_t *data, uint16_t size) {
    uint8_t x;
    uint16_t crc = 0xFFFF;

    while (size--){
        x = crc >> 8 ^ *data++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

#define STANDARD_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF } __attribute__((packed));
#define RELIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t crc; } __attribute__((packed));
#define IDENTIFIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t id; } __attribute__((packed));
#define RELIABLE_IDENTIFIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t id; uint16_t crc; }__attribute__((packed));
#define MAKE_RELIABLE(PACKET) (PACKET).crc = __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)
#define IS_RELIABLE(PACKET) (PACKET).crc == __gen_crc16((uint8_t*) &(PACKET), sizeof((PACKET)) - 2)

#endif /* PROTOCOL_MACROS_H */
