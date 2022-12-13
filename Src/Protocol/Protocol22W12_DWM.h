
/*
 * Protocol20W18.h
 *
 *  Created on: 3 May 2022
 *      Author: Arion
 */

#ifndef PROTOCOL_PROTOCOL_DWM_22W12_H_
#define PROTOCOL_PROTOCOL_DWM_22W12_H_

#include <cstdint>
#include <chrono>

#include "ProtocolMacros.h"

RELIABLE_PACKET(DopplerPacket,
	uint64_t timestamp;
	int32_t cfo;
);

RELIABLE_PACKET(ConnectPacket,
	uint8_t name[32];
);


struct DisconnectPacket {
	;
} __attribute__((packed));


struct RequestPacket {
	uint16_t uuid;
	uint8_t action_id;
	uint8_t target_id;
	uint32_t payload;
} __attribute__((packed));


struct AcknowledgePacket {
	uint16_t uuid;
	uint8_t result;
} __attribute__((packed));


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


struct DataPacket {
	uint32_t data;
} __attribute__((packed));


struct MessagePacket {
	uint8_t message[128];
} __attribute__((packed));


struct ErrorPacket {
	uint8_t error_id;
} __attribute__((packed));


#endif /* PROTOCOL_PROTOCOL_DWM_22W12_H_ */
