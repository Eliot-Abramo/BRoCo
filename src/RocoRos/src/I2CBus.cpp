/*
 * I2CBus.cpp
 *
 *  Created on: Jul 14, 2021
 *      Author: arion
 */

#include "I2CBus.h"


#ifdef BUILD_WITH_I2CBUS
#include "Protocol/Protocol.h"

I2CBus::I2CBus(IODriver* driver) : IOBus(driver, i2c_frame, I2C_FRAME_SIZE) {


	// power supply
	define<Power_SystemPacket>(8);
	define<Power_VoltagePacket>(9);
	define<Power_CurrentPacket>(10);

	define<Reset_PowerSupplyPacket>(13);

	// finite state machine
	define<FsmPacket>(22);

}


#endif /* BUILD_WITH_I2CBUS */
