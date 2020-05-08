/*
 * Protocol20W18.h
 *
 *  Created on: 3 May 2020
 *      Author: Arion
 */

#ifndef PROTOCOL_PROTOCOL20W18_H_
#define PROTOCOL_PROTOCOL20W18_H_

#include <cstdint>

struct TestPacket {
	uint32_t magic = 0xC0FFEE;
	const char data[32] = "This test packet is so hot";
} __attribute__((packed));


#endif /* PROTOCOL_PROTOCOL20W18_H_ */
