/*
 * VirtualIO.h
 *
 *  Created on: 27 Apr 2020
 *      Author: Arion
 */

#ifndef NETWORKIO_H_
#define NETWORKIO_H_

#include "build/Build.h"


#ifdef BUILD_WITH_NETWORK_IO

#include <cstdint>
#include <functional>
#include <thread>

#include <arpa/inet.h>
#include <sys/poll.h>

static const uint16_t PORT_A = 42666; // Ground station
static const uint16_t PORT_B = 42667; // Peripherals
static const uint32_t MAX_CLIENTS = 64;

class NetworkIO {
public:
	NetworkIO(uint16_t port);
	~NetworkIO();

	int8_t connectServer();
	void disconnect();

	void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

private:
	sockaddr_in address;
	bool connected;
	std::thread reception_thread;
	struct pollfd sockets[MAX_CLIENTS];
	uint32_t num_sockets;
	std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver;

	void receiveThread();
	void closeSockets();
};


#endif /* BUILD_WITH_NETWORK_IO */

#endif /* NETWORKIO_H_ */
