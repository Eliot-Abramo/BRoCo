/*
 * IOBus.h
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#include "MessageBus.h"
#include "IODriver.h"


class IOBus : public MessageBus {
public:
	IOBus(IODriver* driver, uint8_t* buffer, uint32_t length);

private:
	IODriver* driver;
	uint8_t* packet_buffer;
	uint32_t buffer_length;
	uint8_t buffer_index;

	void receive(uint8_t sender_id, uint8_t* buffer, uint32_t length);
	uint8_t append(uint8_t* buffer, uint32_t length);
	void transmit();
};

