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
	define<PingPacket>(1);
    define<FOURINONEPacket>(2);
    define<NPKPacket>(3);
    define<VoltmeterPacket>(4);
	define<MassPacket>(5);
    define<IMUPacket>(6);
    define<MagPacket>(7);
	define<PotentiometerPacket>(8);
	define<SpectroPacket>(9);
	define<SpectroResponsePacket>(10);
    define<LaserPacket>(11);
	define<LaserResponsePacket>(12);
	define<ServoPacket>(13);
	define<ServoResponsePacket>(14);
	define<LEDPacket>(15);
	define<LEDResponsePacket>(16);

	// configuration packets
	define<MassConfigRequestPacket>(31);
	define<MassConfigPacket>(32);
	define<MassConfigResponsePacket>(33);
	define<PotentiometerConfigRequestPacket>(34);
	define<PotentiometerConfigPacket>(35);
	define<PotentiometerConfigResponsePacket>(36);
	define<AccelConfigRequestPacket>(37);
	define<AccelConfigPacket>(38);
	define<AccelConfigResponsePacket>(39);
	define<GyroConfigRequestPacket>(40);
	define<GyroConfigPacket>(41);
	define<GyroConfigResponsePacket>(42);
	define<MagConfigRequestPacket>(43);
	define<MagConfigPacket>(44);
	define<MagConfigResponsePacket>(45);
	define<ServoConfigRequestPacket>(46);
	define<ServoConfigPacket>(47);
	define<ServoConfigResponsePacket>(48);

	// calibration packets
	define<MassCalibPacket>(49);
	define<ImuCalibPacket>(50);

	define<DummyPacket>(51);

	// general packets
	define<DataPacket>(59);
	define<ErrorPacket>(60);
	define<RequestPacket>(61);
	define<ResponsePacket>(62);
	define<ProgressPacket>(63);
}

#endif /* BUILD_WITH_CAN_BUS */
