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

IDENTIFIABLE_PACKET(MassPacket,
  float mass[4];                    // [g]
)

IDENTIFIABLE_PACKET(FOURINONEPacket,
  float temperature;                // [°C]
  float moisture;                    // [%]
  float conductivity;                // [us/cm]
  float pH;                            // [-]
 )

IDENTIFIABLE_PACKET(NPKPacket,
  uint16_t nitrogen;                // [mg/kg]
  uint16_t phosphorus;                // [mg/kg]
  uint16_t potassium;                // [mg/kg]
)

IDENTIFIABLE_PACKET(PotentiometerPacket,
  float angles[4];                     //[deg]
)

IDENTIFIABLE_PACKET(IMUPacket,
float acceleration[3];				//[m/s^2]
float angular[3];					//[°/s]
float orientation[4];				//[-]
)

IDENTIFIABLE_PACKET(VoltmeterPacket,
  float voltage; 					//[V]
)

IDENTIFIABLE_PACKET(SpectroPacket,
  bool measure;
)

IDENTIFIABLE_PACKET(SpectroResponsePacket,
  float data[17];
  bool success;
)

IDENTIFIABLE_PACKET(LaserPacket,
  bool enable;
)

IDENTIFIABLE_PACKET(LaserResponsePacket,
  bool success;
)

IDENTIFIABLE_PACKET(ServoPacket,
  uint8_t channel;
  float angle;
)

IDENTIFIABLE_PACKET(ServoResponsePacket,
  uint8_t channel;
  float angle;
  bool success;
)

IDENTIFIABLE_PACKET(LEDPacket,
  uint8_t state;
)

IDENTIFIABLE_PACKET(LEDResponsePacket,
  uint8_t state;
  bool success;
)


//----------General packets----------

STANDARD_PACKET(DataPacket,
	uint32_t data;
)

IDENTIFIABLE_PACKET(PingPacket,
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
