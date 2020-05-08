/*
 * IOBus.cpp
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#include "IOBus.h"

IOBus::IOBus(IODriver* driver) {
	this->driver = driver;
	this->buffer_index = 0;

	using namespace std::placeholders;
	driver->receive(std::bind(&IOBus::receive, this, _1, _2, _3));
}

uint8_t IOBus::append(uint8_t* buffer, uint32_t length) {
	uint32_t remaining_length = IO_BUFFER_LENGTH - buffer_index;

	if(length > remaining_length) {
		length = remaining_length;
	}

	memcpy(packet_buffer + buffer_index, buffer, length);

	buffer_index += length;

	return length;
}

void IOBus::transmit() {
	driver->transmit(packet_buffer, buffer_index);
	buffer_index = 0;
}
