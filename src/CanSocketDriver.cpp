/*
 * NetworkClientIO.cpp
 *
 *  Created on: 7 June 2023
 *      Author: YassineBakkali
 */

#include "CanSocketDriver.h"


#ifdef BUILD_WITH_CAN_SOCKET_DRIVER


#include <iostream>
#include <unistd.h>
#include <fcntl.h>

CanSocketDriver::CanSocketDriver(const char* ifname) {
    struct sockaddr_can addr;
    struct ifreq ifr;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) == -1) {
		perror("Error while opening socket");
		this->connected = false;
	}

	int canfd_enabled = 1;
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_enabled, sizeof(int));
	strcpy(ifr.ifr_name, ifname);
	ioctl(s, SIOCGIFINDEX, &ifr);
	
	addr.can_family  = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
		perror("Error in socket bind");
		this->connected = false;
	} else {
		printf("CAN connected.");
        this->connected = true;

    }
}

void CanSocketDriver::start_reception() {
	this->reception_thread = std::thread(&CanSocketDriver::receiveThread, this);
}


/*
 * Releases IO resources
 */
CanSocketDriver::~CanSocketDriver() {
	closeDevice();
    this->connected = false;
}

/*
 * Closes all used IO resources
 */
void CanSocketDriver::closeDevice() {
    if(connected) {
        close(fd);
    }
}

/*
 * Reception thread
 *
 * Processes input from the remote connections and passes it to the reception handler.
 * Handles closing connections.
 */
void CanSocketDriver::receiveThread() {
    int n_bytes;

    std::cout << "Connected to network" << std::endl;

	while(connected) {
		// New data from endpoint
		// while((result = read(s, &rxframe, sizeof(struct canfd_frame))) > 0) {
        //     receiver(0, rxframe.data, result);
		// }
		n_bytes = read(s, &rxframe, CANFD_MTU);
		if (n_bytes != CANFD_MTU && n_bytes != CAN_MTU)
			std::cout << "Invalid CAN(FD) frame" << std::endl;
		else
			receiver(0, rxframe.data, rxframe.len);
        std::this_thread::yield();
	}

	std::cout << "Disconnected from network" << std::endl;

	closeDevice();
}

/*
 * Sets the receiver callback function
 */
void CanSocketDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
	this->receiver = receiver;
}

void CanSocketDriver::TxFrameConfig(uint32_t can_id)
{
    txframe.can_id  = can_id;
}

/*
 * Transmits the given data to the server
 */
void CanSocketDriver::transmit(uint8_t* buffer, uint32_t length) {
	if(connected) {
		int32_t result;
        txframe.len = length;
        for(int i = 0; i < length; ++i){
	        txframe.data[i] = buffer[i];
        }
        result = write(s, &txframe, CANFD_MTU);
	}
}

#endif /* BUILD_WITH_UDEV_DRIVER */
