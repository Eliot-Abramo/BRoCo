/*
 * Protocol21W3.h
 *
 *  Created on: 3 May 2020
 *      Author: Aurelio Noca
 */

#ifndef PROTOCOL_PROTOCOL21W03_H_
#define PROTOCOL_PROTOCOL21W03_H_

#include <cstdint>

// General packets
struct PingPacket {
	uint64_t time;
} __attribute__((packed));

struct RequestPacket {
	uint16_t uuid; 
	uint8_t action_id;
	uint8_t target_id;
	uint32_t payload;
} __attribute__((packed));

// calibrer la balance
// reset les sensors
// stop sending data


struct ResponsePacket {
	uint16_t uuid;
	uint8_t action_id;
	uint8_t target_id;
	uint32_t payload;
} __attribute__((packed));

struct ProgressPacket {
	uint32_t uuid;
	uint8_t progress;
} __attribute__((packed));

struct ErrorPacket {
	uint8_t error_id;
} __attribute__((packed));

struct DataPacket {
	uint32_t data;
} __attribute__((packed));

// Avionics
struct Avionics_BaroTempPacket {
  float pressure;
  float temperature;
} __attribute__((packed));

struct Avionics_AccelMagPacket {
  float acceleration[3];
  float angular[3];
  float magneto[3];
} __attribute__((packed));

// // Handling device
// struct Handling_GripperPacket {
//   float voltage;
// } __attribute__((packed));

// Power
struct Power_VoltagePacket {
  float voltages[4];
} __attribute__((packed));

struct Power_CurrentPacket {
  float currents[4];
} __attribute__((packed));

struct Power_SystemPacket {
  float battery_charge;
  uint8_t state;
} __attribute__((packed));

// Science
struct Science_MeasurePacket {
  float mass;
} __attribute__((packed));


// Power supply packets

struct Reset_PowerSupplyPacket {
	bool reset;
} __attribute__((packed));

struct Switch_AvionicsPacket {
	bool on;
} __attribute__((packed));

struct Switch_RamanPacket  {
	bool on;
} __attribute__((packed));

struct Switch_JetsonPacket  {
	bool on;
} __attribute__((packed));

struct Switch_LidarPacket  {
	bool on;
} __attribute__((packed));

struct Switch_EthernetPacket  {
	bool on;
} __attribute__((packed));







struct FsmPacket{
	uint32_t state;
}__attribute__((packed));

struct PotentiometersPacket {
	uint8_t port;
	float voltage;
}__attribute__((packed));

// port sensors: cahr, port, tension
//

// for I2C ports sensors
struct PortSensorsPacket{
	uint8_t port;
	float data;
}





#endif /* PROTOCOL_PROTOCOL21W3_H_ */
