/*
 * NetworkBus.cpp
 *
 *  Created on: 8 May 2020
 *      Author: Arion
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_NETWORK_BUS


#include "NetworkBus.h"
#include "Protocol/Protocol.h"

NetworkBus::NetworkBus(IODriver* driver) : IOBus(driver, network_frame, sizeof(network_frame)) {

	// avionics
	define<DummySystem_DummyPacket>(0);
	define<MassPacket>(1);
    define<FOURINONE_Packet>(2);
    define<NPK_Packet>(3);
    define<VoltmeterPacket>(4);
    define<IMU_Packet>(5);
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


#endif /* BUILD_WITH_NETWORK_BUS */
