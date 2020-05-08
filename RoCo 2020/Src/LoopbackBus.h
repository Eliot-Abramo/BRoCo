/*
 * LoopbackBus.h
 *
 *  Created on: 3 May 2020
 *      Author: Arion
 */

#ifndef LOOPBACKBUS_H_
#define LOOPBACKBUS_H_

#include "Build/Build.h"


#ifdef BUILD_WITH_LOOPBACK_BUS


#include "MessageBus.h"

#define LOOPBACK_BUFFER_LENGTH 5

class LoopbackBus : public MessageBus {
public:
	LoopbackBus(uint64_t latency);

private:
	uint64_t latency;
	uint8_t packet_buffer[LOOPBACK_BUFFER_LENGTH];
	uint8_t buffer_index;

	uint8_t append(uint8_t* buffer, uint32_t length);
	void transmit();
};


#endif /* BUILD_WITH_LOOPBACK_BUS */

#endif /* LOOPBACKBUS_H_ */
