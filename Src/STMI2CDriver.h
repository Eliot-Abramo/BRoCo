/*
 * STMI2CDriver.h
 *
 *  Created on: Jul 13, 2021
 *      Author: arion
 */

#ifndef LIBRARIES_ROCO_SRC_STMI2CDRIVER_H_
#define LIBRARIES_ROCO_SRC_STMI2CDRIVER_H_

#include "Build/Build.h"

#ifdef BUILD_WITH_STMI2C

#include "I2CDriver.h"
#include "stm32f4xx.h"


class STMI2CDriver : public I2CDriver {
public:
	STMI2CDriver(I2CDevice* dev, I2C_HandleTypeDef* hi2c);
	~STMI2CDriver();
	uint8_t* getBuffer();

protected:
	void transmitI2C(uint8_t target, uint8_t* data, uint32_t length);

private:
	I2C_HandleTypeDef* hi2c;
	uint32_t last_dma_index;
	uint8_t* buffer;

	class STMI2CReceptionThread : Thread {
	public:
		STMI2CReceptionThread(std::function<void ()> tickFunc) : Thread("I2CReceptionThread", 1024), tickFunc(tickFunc) {}
		void loop() { tickFunc(); };

	private:
		std::function<void ()> tickFunc;
	};

	STMI2CReceptionThread* recvThread;
};

#endif /* BUILD_WITH_STMI2C */

#endif /* LIBRARIES_ROCO_SRC_STMI2CDRIVER_H_ */
