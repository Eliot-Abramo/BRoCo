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

#include <vector>

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

IDENTIFIABLE_PACKET(MagPacket,
float mag[3];
float mag_raw[3];
)

IDENTIFIABLE_PACKET(VoltmeterPacket,
  float voltage; 					//[V]
)

IDENTIFIABLE_PACKET(SpectroPacket,
  bool measure;
)

IDENTIFIABLE_PACKET(SpectroResponsePacket,
  uint16_t data[18];
  float max_val;
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

//------Configuration packets--------

IDENTIFIABLE_PACKET(MassConfigRequestPacket,
  bool req_offset;
  bool req_scale;
  bool req_alpha;
  bool req_channels_status;
)

IDENTIFIABLE_PACKET(MassConfigPacket,
  float offset[4];
  float scale[4];
  float alpha;
  bool enabled_channels[4];
  bool remote_command;
  bool set_offset;
  bool set_scale;
  bool set_alpha;
  bool set_channels_status;
)

IDENTIFIABLE_PACKET(MassConfigResponsePacket,
  float offset[4];
  float scale[4];
  float alpha;
  bool enabled_channels[4];
  bool remote_command;
  bool set_offset;
  bool set_scale;
  bool set_alpha;
  bool set_channels_status;
  bool success;
)

IDENTIFIABLE_PACKET(PotentiometerConfigRequestPacket,
  bool req_min_voltages;
  bool req_max_voltages;
  bool req_min_angles;
  bool req_max_angles;
  bool req_channels_status;
)

IDENTIFIABLE_PACKET(PotentiometerConfigPacket,
  float min_voltages[4];
  float max_voltages[4];
  float min_angles[4];
  float max_angles[4];
  bool enabled_channels[4];
  bool remote_command;
  bool set_min_voltages;
  bool set_max_voltages;
  bool set_min_angles;
  bool set_max_angles;
  bool set_channels_status;
)

IDENTIFIABLE_PACKET(PotentiometerConfigResponsePacket,
  float min_voltages[4];
  float max_voltages[4];
  float min_angles[4];
  float max_angles[4];
  bool enabled_channels[4];
  bool remote_command;
  bool set_min_voltages;
  bool set_max_voltages;
  bool set_min_angles;
  bool set_max_angles;
  bool set_channels_status;
  bool success;
)

IDENTIFIABLE_PACKET(AccelConfigRequestPacket,
  bool req_bias;
  bool req_transform;
)

IDENTIFIABLE_PACKET(AccelConfigPacket,
  float bias[3];
  float transform[9];
  bool remote_command;
  bool set_bias;
  bool set_transform;
)

IDENTIFIABLE_PACKET(AccelConfigResponsePacket,
  float bias[3];
  float transform[9];
  bool remote_command;
  bool set_bias;
  bool set_transform;
  bool success;
)

IDENTIFIABLE_PACKET(GyroConfigRequestPacket,
  bool req_bias;
)

IDENTIFIABLE_PACKET(GyroConfigPacket,
  float bias[3];
  bool remote_command;
  bool set_bias;
)

IDENTIFIABLE_PACKET(GyroConfigResponsePacket,
  float bias[3];
  bool remote_command;
  bool set_bias;
  bool success;
)

IDENTIFIABLE_PACKET(MagConfigRequestPacket,
  bool req_hard_iron;
  bool req_soft_iron;
)

IDENTIFIABLE_PACKET(MagConfigPacket,
  float hard_iron[3];
  float soft_iron[9];
  bool remote_command;
  bool set_hard_iron;
  bool set_soft_iron;
)

IDENTIFIABLE_PACKET(MagConfigResponsePacket,
  float hard_iron[3];
  float soft_iron[9];
  bool remote_command;
  bool set_hard_iron;
  bool set_soft_iron;
  bool success;
)

IDENTIFIABLE_PACKET(ServoConfigRequestPacket,
  bool req_min_duty;
  bool req_max_duty;
  bool req_min_angles;
  bool req_max_angles;
)

IDENTIFIABLE_PACKET(ServoConfigPacket,
  float min_duty[4];
  float max_duty[4];
  float min_angles[4];
  float max_angles[4];
  bool remote_command;
  bool set_min_duty;
  bool set_max_duty;
  bool set_min_angles;
  bool set_max_angles;
)

IDENTIFIABLE_PACKET(ServoConfigResponsePacket,
  float min_duty[4];
  float max_duty[4];
  float min_angles[4];
  float max_angles[4];
  bool remote_command;
  bool set_min_duty;
  bool set_max_duty;
  bool set_min_angles;
  bool set_max_angles;
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
