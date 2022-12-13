/*
 * Protocol22W29.h
 *
 *  Created on: 18 July 2021
 *      Author: Arion
 */

#ifndef PROTOCOL_PROTOCOL22W29_H_
#define PROTOCOL_PROTOCOL22W29_H_

#include "ProtocolMacros.h"

#include <cstdint>



//Different action id definitions to be sent:
//Avionics
#define TARE (uint8_t)0x1 //Recalibrate sensor: ads1113, hx711, imu, barometer
#define RESET_SENSOR (uint8_t)0x2
#define STOP_DATA (uint8_t)0x3

//Science
#define START_ANALYSIS (uint8_t)0x4 //Random example

/*
 * POWER SUPPLY DEFINITIONS
 */

#define PS_UID_SUPERVISOR 0x42
#define PS_UID_CTA 0x43
#define PS_UID_CTB 0x44
#define PS_ACTION_GET 0x0
#define PS_ACTION_SET 0x1
#define PS_ACTION_EXE 0x2
#define PS_TARGET_START 0x0
#define PS_TARGET_STOP  0x1
#define PS_TARGET_RESET 0x2
#define PS_TARGET_TEST 0x3
#define PS_TARGET_MISSION_ID 0x4
#define PS_TARGET_DOWNLOAD 0x5
#define PS_TARGET_ACK 0x6
#define PS_TARGET_NACK 0x7
#define PS_TARGET_FLASH_ERASE 0x8
#define PS_CTA  0x0
#define PS_CTB  0x1
#define PS_5V   0x2
#define PS_15V  0x3
#define PS_24V  0x4
#define PS_48V  0x5

//----------Avionics----------

STANDARD_PACKET(Avionics_BaroTempPacket,
  float pressure;					//[Pa]
  float temperature;				//[°C]
)

STANDARD_PACKET(Avionics_AccelMagPacket,
  float acceleration[3];			//[m/s^2]
  float angular[3];					//[°]
  float magneto[3];					//[mT]
)

// Handling device + potentiometers (ads1113)
STANDARD_PACKET(Avionics_ADCPacket,
  uint8_t port;
  float voltage;					//[V]
)

// Science
STANDARD_PACKET(Science_MassPacket,
  float mass;						//[g]
)

//----------Power supply----------

RELIABLE_PACKET(Power_BusInfo,
	uint8_t bus_id; // 0: battery, 1: motors, 2: 5V, 3: 15V, 4: 24V, 5: 48V
	float voltage;
	float current;
	float energy;
	float ripple;
	float temperature;
)

RELIABLE_PACKET(Power_BatteryInfo,
	float charge;
	float estimated_runtime;
)

RELIABLE_PACKET(Power_ControllerHealth,
	float heap;
	float flash;
)

RELIABLE_PACKET(Power_ControllerState,
	uint8_t state; // LSB (48V running)|(24V running)|(15V running)|(5V running)|0|0|sync|fault
)

//----------FSM----------

STANDARD_PACKET(FsmPacket,
	uint32_t state;
)


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


#endif /* PROTOCOL_PROTOCOL22W29_H_ */
