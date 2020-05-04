/*
 * echo_test.cpp
 *
 *  Created on: 27 Apr 2020
 *      Author: Arion
 */


#include <iostream>
#include <iomanip>

#include "../Src/NetworkIO.h"

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

int main() {
	std::cout << "Starting main test..." << std::endl;

	NetworkIO* io = new NetworkIO(PORT_B);

	io->receive(&handle_input);

	uint32_t result = io->connectServer();

	if(result < 0) {
		std::cout << "External IO connection failed with error code " << result << std::endl;
	} else {
		std::cout << "Connected to external IO" << std::endl;
	}

	while(true);
}
