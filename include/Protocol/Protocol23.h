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

// Total size: 2 (preamble + packet id) + 2 (id) + 18 * 2 (data) + 4 (max_val) + 2 (success) = 46
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
  uint16_t min_voltages[4];
  uint16_t max_voltages[4];
  uint16_t min_angles[4];
  uint16_t max_angles[4];
  float min_voltages_max_val;
  float max_voltages_max_val;
  float min_angles_max_val;
  float max_angles_max_val;
  bool enabled_channels[4];
  bool remote_command;
  bool set_min_voltages;
  bool set_max_voltages;
  bool set_min_angles;
  bool set_max_angles;
  bool set_channels_status;
)

// Total size: 2 (preamble + packet ID) + 2 (id) + 4*4*2 (uint16) + 4*4 (float) + 4 + 7 (bool) = 63 bytes
IDENTIFIABLE_PACKET(PotentiometerConfigResponsePacket,
  uint16_t min_voltages[4];
  uint16_t max_voltages[4];
  uint16_t min_angles[4];
  uint16_t max_angles[4];
  float min_voltages_max_val;
  float max_voltages_max_val;
  float min_angles_max_val;
  float max_angles_max_val;
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

// Total size: 2 (preamble + packet ID) + 2 (id) + 12*4 (bias + transform) + 4 (bools) = 56 bytes
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

// Total size: 2 (preamble + packet ID) + 2 (id) + 12*4 (hard_iron + soft_iron) + 4 (bools) = 56 bytes
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
  uint16_t min_duty[4];
  uint16_t max_duty[4];
  uint16_t min_angles[4];
  uint16_t max_angles[4];
  float min_duty_max_val;
  float max_duty_max_val;
  float min_angles_max_val;
  float max_angles_max_val;
  bool remote_command;
  bool set_min_duty;
  bool set_max_duty;
  bool set_min_angles;
  bool set_max_angles;
)

// Total size: 2 (preamble + packet ID) + 2 (id) + 4*4*2 (uint16) + 4*4 (float) + 6 (bool) = 58 bytes
IDENTIFIABLE_PACKET(ServoConfigResponsePacket,
  uint16_t min_duty[4];
  uint16_t max_duty[4];
  uint16_t min_angles[4];
  uint16_t max_angles[4];
  float min_duty_max_val;
  float max_duty_max_val;
  float min_angles_max_val;
  float max_angles_max_val;
  bool remote_command;
  bool set_min_duty;
  bool set_max_duty;
  bool set_min_angles;
  bool set_max_angles;
  bool success;
)

// Calibration request packets -----------------

IDENTIFIABLE_PACKET(MassCalibPacket,
	bool tare;
	bool calib_scale;
	float expected_weight;
)

IDENTIFIABLE_PACKET(ImuCalibPacket,
	bool calib_offset_accel;
	bool calib_offset_gyro;
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
