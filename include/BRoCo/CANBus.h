/*
 * NetworkBus.h
 *
 *  Created on: August 2023
 *      Author: Vincent
 */

#ifndef CANBUS_H_
#define CANBUS_H_

#include "Build/Build.h"


#ifdef BUILD_WITH_CAN_BUS

#include "IOBus.h"

#define CAN_FRAME_SIZE 64


class CANBus : public IOBus {
public:
	CANBus(IODriver* driver); // Constructor is inherited

private:
	uint8_t can_frame[CAN_FRAME_SIZE];
};


#endif /* BUILD_WITH_CAN_BUS */

#endif /* CANBUS_H_ */
