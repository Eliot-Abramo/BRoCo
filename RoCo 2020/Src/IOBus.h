/*
 * IOBus.h
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#include "MessageBus.h"
#include "IODriver.h"

#define IO_BUFFER_LENGTH 256

class IOBus : public MessageBus {
public:
	IOBus(IODriver* driver);

private:
	IODriver* driver;
	uint8_t packet_buffer[IO_BUFFER_LENGTH];
	uint8_t buffer_index;

	uint8_t append(uint8_t* buffer, uint32_t length);
	void transmit();
};

