/*
 * MessageBus.h
 *
 *  Created on: 09 Feb 2020
 *      Author: Arion
 */

#ifndef MESSAGE_BUS_H_
#define MESSAGE_BUS_H_

#include <cstdint>
#include <cstddef>
#include <typeindex>
#include <functional>


#define PREAMBLE 0x7F

struct PacketDefinition {
	uint8_t id = 0xFF;
	size_t size;
	size_t hash;
};

class MessageBus {
public:
	virtual ~MessageBus() {}

	template<typename T> bool define(uint8_t identifier);
	template<typename T> bool handle(std::function<void(uint8_t, T*)> handler);
	template<typename T> bool forward(MessageBus* bus);
	template<typename T> bool send(T *message);

protected:
	void receive(uint8_t senderID, uint8_t *pointer, uint32_t length);
	virtual uint32_t append(uint8_t* buffer, uint32_t length) = 0; // Must be atomic
	virtual void transmit() = 0;
	virtual bool internal_send(PacketDefinition* def, uint8_t* data);
	virtual PacketDefinition* retrieve(size_t hash);

private:
	static const uint32_t max_packet_size = 1023;
	static const uint32_t max_unique_senders = 4;

	struct ReconstructionBuffer {
		uint8_t current_packet_id;
		uint8_t buffer[max_packet_size];
		size_t index;
	};

	PacketDefinition definitions_by_id[64] = {0};
	PacketDefinition* definitions_by_type[256] = {0}; // Factor 4 to mitigate hash collisions
	ReconstructionBuffer reconstruction_buffers[max_unique_senders];

	std::function<void(uint8_t, void*)> handlers[64];
	MessageBus* forwarders[64];
};


#endif /* MESSAGE_BUS_H_ */
