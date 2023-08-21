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

	// sensors
	define<PingPacket>(58);
    define<FOURINONEPacket>(1);
    define<NPKPacket>(2);
    define<VoltmeterPacket>(3);
	define<MassPacket>(4);
    define<IMUPacket>(5);
	define<PotentiometerPacket>(6);
	define<SpectroPacket>(7);
	define<SpectroResponsePacket>(8);
    define<LaserPacket>(9);
	define<LaserResponsePacket>(10);
	define<ServoPacket>(11);
	define<ServoResponsePacket>(12);
	define<LEDPacket>(13);
	define<LEDResponsePacket>(14);

	// configuration packets
	define<MassConfigRequestPacket>(15);
	define<MassConfigPacket>(16);
	define<MassConfigResponsePacket>(17);
	define<PotentiometerConfigRequestPacket>(18);
	define<PotentiometerConfigPacket>(19);
	define<PotentiometerConfigResponsePacket>(20);
	define<AccelConfigRequestPacket>(21);
	define<AccelConfigPacket>(22);
	define<AccelConfigResponsePacket>(23);
	define<GyroConfigRequestPacket>(24);
	define<GyroConfigPacket>(25);
	define<GyroConfigResponsePacket>(26);
	define<MagConfigRequestPacket>(27);
	define<MagConfigPacket>(28);
	define<MagConfigResponsePacket>(29);
	define<ServoConfigRequestPacket>(30);
	define<ServoConfigPacket>(31);
	define<ServoConfigResponsePacket>(32);

	// general packets
	define<DataPacket>(59);
	define<ErrorPacket>(60);
	define<RequestPacket>(61);
	define<ResponsePacket>(62);
	define<ProgressPacket>(63);
}

#endif /* BUILD_WITH_CAN_BUS */
