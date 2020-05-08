/*
 * echo_test.cpp
 *
 *  Created on: 27 Apr 2020
 *      Author: Arion
 */


#include <sys/errno.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <thread>

#include "../Src/LoopbackBus.h"
#include "../Src/NetworkClientIO.h"
#include "../Src/NetworkIO.h"
#include "../Src/NetworkServerIO.h"
#include "../Src/Protocol/Protocol20W18.h"

struct TestPacket;

void handle_input(uint8_t sender_id, uint8_t* buffer, uint32_t length) {
	std::cout << std::endl << "---------- Frame begin from sender ID " << (uint32_t) sender_id << " ----------" << std::endl << std::endl << " ";

	for(int32_t i = 0; i < length; i++) {
		std::cout << std::setfill('0') << std::setw(2) << std::hex << (uint32_t) buffer[i] << " ";

		if(i % 16 == 15 || i == length - 1) {
			std::cout << std::endl;
		}

		if(i % 4 == 3) {
			std::cout << " ";
		}
	}

	std::cout << std::dec << std::endl << "---------- Frame end ----------" << std::endl << std::endl;
}

void handle_packet(TestPacket* packet) {
	std::cout << std::hex << packet->magic << std::endl;
	std::cout << packet->data << std::endl;
}

int main() {
	std::cout << "Starting main test..." << std::endl;

	NetworkServerIO* io = new NetworkServerIO(PORT_B);

	io->receive(&handle_input);

	int32_t result = io->connectServer();

	if(result < 0) {
		std::cout << "Network Server IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network server IO" << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	NetworkClientIO* client_io = new NetworkClientIO("127.0.0.1", PORT_B);

	result = client_io->connectClient();

	if(result < 0) {
		std::cout << "Network Client IO connection failed with error code " << result << std::endl;
		std::cout << std::strerror(errno) << std::endl;
	} else {
		std::cout << "Connected to network client IO" << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	LoopbackBus* bus = new LoopbackBus(100);

	bus->define<TestPacket>(0);
	bus->handle(handle_packet);

	TestPacket p;

	bus->send(&p);

	std::this_thread::sleep_for(std::chrono::seconds(1));



	std::cout << "Test finished" << std::endl;

	while(true);
}
