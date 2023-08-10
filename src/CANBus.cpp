/*
 * NetworkBus.cpp
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_CAN_BUS


#include "CANBus.h"
#include "Protocol/Protocol.h"

CANBus::CANBus(IODriver* driver) : IOBus(driver, can_frame, sizeof(can_frame)) {

	// avionics
	define<DummyPacket>(0);
	define<MassPacket>(1);
    define<FOURINONEPacket>(2);
    define<NPKPacket>(3);
    define<VoltmeterPacket>(4);
    define<IMUPacket>(5);
    define<PotentiometerPacket>(6);
    define<ColorFilterPacket>(7);

	// general packets
	define<DataPacket>(58);
	define<PingPacket>(59);
	define<ErrorPacket>(60);
	define<RequestPacket>(61);
	define<ResponsePacket>(62);
	define<ProgressPacket>(63);
}

#endif /* BUILD_WITH_CAN_BUS */
