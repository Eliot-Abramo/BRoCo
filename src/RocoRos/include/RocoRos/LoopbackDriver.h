/*
 * LoopbackDriver.h
 *
 *  Created on: 19 Jul 2022
 *      Author: arion
 */

#ifndef ROCO_SRC_LOOPBACKDRIVER_H_
#define ROCO_SRC_LOOPBACKDRIVER_H_

#include "IODriver.h"

class LoopbackDriver : public IODriver {
public:
	void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

private:
	std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver_func;
};


#endif /* ROCO_SRC_LOOPBACKDRIVER_H_ */
