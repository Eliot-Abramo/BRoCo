/*
 * I2CDriver.h
 *
 *  Created on: Jul 13, 2021
 *      Author: arion
 */

#ifndef LIBRARIES_ROCO_SRC_I2CDRIVER_H_
#define LIBRARIES_ROCO_SRC_I2CDRIVER_H_


#include "IODriver.h"
#include "I2CDevice.h"

#include <cstdint>


#define I2C_BUFFER_SIZE 256


class I2CDriver : public IODriver {

public:
	I2CDriver(I2CDevice* dev);
	virtual ~I2CDriver();
	void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

	virtual void transmitI2C(uint8_t target, uint8_t* buffer, uint32_t length) = 0;
	void receiveI2C(uint8_t source, uint8_t* buffer, uint32_t length);

protected:
	I2CDevice* dev;

private:
	std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiverFunc;
};

#endif
