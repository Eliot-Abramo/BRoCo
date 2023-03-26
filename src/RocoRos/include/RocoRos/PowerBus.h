/*
 * PowerBus.h
 *
 *  Created on: 23 March 2022
 *      Author: Arion
 */

#ifndef POWERBUS_H_
#define POWERBUS_H_

#include "Build/Build.h"


#ifdef BUILD_WITH_POWER_BUS


#include "IOBus.h"



#define POWER_BUS_FRAME_SIZE 1024


class PowerBus : public IOBus {
public:
	PowerBus(IODriver* driver); // Constructor is inherited

protected:
	virtual bool internal_send(PacketDefinition* def, uint8_t* data);

private:
	uint8_t buffer[POWER_BUS_FRAME_SIZE];
	xSemaphoreHandle semaphore;
};


#endif /* BUILD_WITH_POWER_BUS */

#endif /* POWERBUS_H_ */
