/*
 * I2CBus.h
 *
 *  Created on: Jul 14, 2021
 *      Author: arion
 */

#ifndef LIBRARIES_ROCO_SRC_I2CBUS_H_
#define LIBRARIES_ROCO_SRC_I2CBUS_H_



#include "I2CDriver.h"
#include "IOBus.h"

#include "Build/Build.h"

#ifdef BUILD_WITH_I2CBUS

#define I2C_FRAME_SIZE (I2C_BUFFER_SIZE - 1) // Minus one, so that the DMA buffer is never full


class I2CBus : public IOBus {
public:
	I2CBus(IODriver* driver); // Constructor is inherited

private:
	uint8_t i2c_frame[I2C_FRAME_SIZE];
};

#endif /* BUILD_WITH_I2CBUS */

#endif /* LIBRARIES_ROCO_SRC_I2CBUS_H_ */
