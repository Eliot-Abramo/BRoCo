/*
 * Protocol23.h
 *
 *  Created on: Feb 26, 2023
 *      Author: Vincent
 */

#ifndef BROCO_SRC_PROTOCOL_PROTOCOL23_H_
#define BROCO_SRC_PROTOCOL_PROTOCOL23_H_



#include "ProtocolMacros.h"

#include <cstdint>

//----------Avionics----------

STANDARD_PACKET(DummySystem_DummyPacket,
  int data;
)

STANDARD_PACKET(MassPacket,
  float mass[4];                    // [g]
)

STANDARD_PACKET(FOURINONE_Packet,
  float temperature;                // [°C]
  float moisture;                    // [%]
  float conductivity;                // [us/cm]
  float pH;                            // [-]
 )

STANDARD_PACKET(NPK_Packet,
  uint16_t nitrogen;                // [mg/kg]
  uint16_t phosphorus;                // [mg/kg]
  uint16_t potassium;                // [mg/kg]
)

STANDARD_PACKET(PotentiometerPacket,
  float angles[4];                     //[deg]
)

STANDARD_PACKET(IMU_Packet,
float acceleration[3];				//[m/s^2]
float angular[3];					//[°/s]
float orientation[4];				//[-]
)

STANDARD_PACKET(VoltmeterPacket,
  float voltage; 					//[V]
)
//STANDARD_PACKET(Avionics_AccelMagPacket,
//  float acceleration[3];			//[m/s^2]
//  float angular[3];					//[°]
//  float magneto[3];					//[mT]
//)
//
//// Handling device + potentiometers (ads1113)
//STANDARD_PACKET(Avionics_ADCPacket,
//  uint8_t port;
//  float voltage;					//[V]
//)
//
//// Science
//STANDARD_PACKET(Science_MassPacket,
//  float mass;						//[g]
//)


//----------General packets----------

STANDARD_PACKET(DataPacket,
	uint32_t data;
)

RELIABLE_PACKET(PingPacket,
	uint64_t time;
)

RELIABLE_PACKET(ErrorPacket,
	uint8_t error_id;
)

RELIABLE_PACKET(RequestPacket,
	uint16_t uid;
	uint8_t action_id;
	uint8_t target_id;
	uint32_t payload;
)

RELIABLE_PACKET(ResponsePacket,
	uint16_t uid;
	uint8_t action_id;
	uint8_t target_id;
	uint32_t payload;
)

RELIABLE_PACKET(ProgressPacket,
	uint16_t uid;
	uint8_t action_id;
	uint8_t target_id;
	uint8_t progress;
)

RELIABLE_PACKET(PayloadPacket,
	uint32_t length;
	uint8_t payload[512];
)

STANDARD_PACKET(FlushPacket,
	uint8_t blank[15];
)


#endif /* BROCO_SRC_PROTOCOL_PROTOCOL23_H_ */
