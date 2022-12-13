/*
 * I2CDriver.cpp
 *
 *  Created on: Jul 13, 2021
 *      Author: arion
 */

#include "I2CDriver.h"
#include "Debug/Debug.h"


#include <cstring>

I2CDriver::I2CDriver(I2CDevice* dev) : dev(dev), receiverFunc(nullptr) {

}

I2CDriver::~I2CDriver() {

}

void I2CDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
	this->receiverFunc = receiver;
}

void I2CDriver::transmit(uint8_t* data, uint32_t length) {
	if(dev->getType() == MASTER) {
		I2CMaster* device = (I2CMaster*) dev;
		uint8_t* slaves = device->getSlaves();
		uint8_t num_slaves = device->getNumSlaves();

		for(uint8_t i = 0; i < num_slaves; i++) {
			uint8_t target = slaves[i];
			transmitI2C(target << 1, data, length);
		}
	} else if(dev->getType() == SLAVE) {
		transmitI2C(0, data, length);
	}
}

void I2CDriver::receiveI2C(uint8_t source, uint8_t* buffer, uint32_t length) {
	receiverFunc(source, buffer, length);
}

//void I2CDriver::tick() {
//	using namespace std::placeholders;
//
//	if(dev->getType() == MASTER) {
//		I2CMaster* device = (I2CMaster*) dev;
//		uint8_t* slaves = device->getSlaves();
//		uint8_t num_slaves = device->getNumSlaves();
//
//		for(uint8_t i = 0; i < num_slaves; i++) {
//			uint8_t source = slaves[i];
//			receiveI2C(source << 1, std::bind(receiverFunc, source, _1, _2));
//		}
//	} else if(dev->getType() == SLAVE) {
//		receiveI2C(0, std::bind(receiverFunc, 0, _1, _2));
//	}
//}
