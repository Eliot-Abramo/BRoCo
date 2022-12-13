/*
 * LoopbackDriver.cpp
 *
 *  Created on: 19 Jul 2022
 *      Author: arion
 */

#include "LoopbackDriver.h"

void LoopbackDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver_func) {
    this->receiver_func = receiver_func;
}

void LoopbackDriver::transmit(uint8_t* buffer, uint32_t length) {
	receiver_func(0, buffer, length);
}
