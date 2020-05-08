/*
 * LoopbackBus.cpp
 *
 *  Created on: 3 May 2020
 *      Author: Arion
 */


#include "Build/Build.h"

#ifdef BUILD_WITH_LOOPBACK_BUS


#include "LoopbackBus.h"

#include <thread>

LoopbackBus::LoopbackBus(uint64_t latency) {
	this->latency = latency;
	this->buffer_index = 0;
}

uint8_t LoopbackBus::append(uint8_t* buffer, uint32_t length) {
	uint32_t remaining_length = LOOPBACK_BUFFER_LENGTH - buffer_index;

	if(length > remaining_length) {
		length = remaining_length;
	}

	memcpy(packet_buffer + buffer_index, buffer, length);

	buffer_index += length;

	return length;
}

void LoopbackBus::transmit() {
	std::this_thread::sleep_for(std::chrono::milliseconds(latency));

	receive(0, packet_buffer, buffer_index);

	buffer_index = 0;
}




#endif /* BUILD_WITH_LOOPBACK_BUS */
