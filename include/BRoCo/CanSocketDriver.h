/*
 * CanSocketDriver.h
 *
 *  Created on: 7 June 2023
 *      Author: YassineBakkali
 */

#ifndef CANSOCKETDRIVER_H_
#define CANSOCKETDRIVER_H_

#include "Build/Build.h"

#ifdef BUILD_WITH_CAN_SOCKET_DRIVER

#include "IODriver.h"

#include <cstdint>
#include <functional>
#include <thread>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>


class CanSocketDriver : public IODriver {
public:
	CanSocketDriver(const char* ifname);
	~CanSocketDriver();
	void start_reception();
	void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);
    void TxFrameConfig(uint32_t can_id);


    bool isConnected() { return connected; }
	int getFD() { return fd; }

private:
    int s;
    int fd;
    bool connected;
    struct canfd_frame txframe;
    struct canfd_frame rxframe;

	std::thread reception_thread;
	std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver;

	void receiveThread();
	void closeDevice();
};


#endif /* BUILD_WITH_CAN_SOCKET_DRIVER */

#endif /* CANSOCKETDRIVER_H_ */
