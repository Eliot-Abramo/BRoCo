/*
 * IOBus.h
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#ifndef IOBUS_H_
#define IOBUS_H_

#include "MessageBus.h"
#include "IODriver.h"


class IOBus : public MessageBus {
public:
	IOBus(IODriver* driver, uint8_t* buffer, uint32_t length);

protected:
	void transmit();

private:
	IODriver* driver;
	uint8_t* packet_buffer;
	uint32_t buffer_length;
	uint32_t buffer_index;

	void receive(uint8_t sender_id, uint8_t* buffer, uint32_t length);
	uint32_t append(uint8_t* buffer, uint32_t length);
};

#endif /* IOBUS_H_ */
