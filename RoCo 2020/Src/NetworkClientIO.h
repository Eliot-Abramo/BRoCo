/*
 * VirtualIO.h
 *
 *  Created on: 27 Apr 2020
 *      Author: Arion
 */

#ifndef NETWORKIO_H_
#define NETWORKIO_H_

#include "build/Build.h"


#ifdef BUILD_WITH_NETWORK_CLIENT_IO

#include <cstdint>
#include <functional>
#include <thread>

#include <arpa/inet.h>
#include <sys/poll.h>

static const uint16_t PORT_A = 42666; // Ground station
static const uint16_t PORT_B = 42667; // Peripherals

class NetworkClientIO {
public:
	NetworkClientIO(const char* address, uint16_t port);
	~NetworkClientIO();

	int8_t connectClient();
	void disconnect();

	void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

private:
	const char* address_str;
	sockaddr_in address;
	uint32_t socket_id;
	bool connected;
	std::thread reception_thread;
	std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver;

	void receiveThread();
	void closeSocket();
};


#endif /* BUILD_WITH_NETWORK_CLIENT_IO */

#endif /* NETWORKIO_H_ */
