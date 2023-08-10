/*
 * NetworkBus.h
 *
 *  Created on: 23 March 2022
 *      Author: Arion
 */

#ifndef RADIOBUS_H_
#define RADIOBUS_H_

#include "Build/Build.h"


#ifdef BUILD_WITH_RADIO_BUS


#include "IOBus.h"

#define RADIO_BUS_FRAME_SIZE 256


class RadioBus : public IOBus {
public:
	RadioBus(IODriver* driver); // Constructor is inherited

private:
	uint8_t buffer[RADIO_BUS_FRAME_SIZE];
};


#endif /* BUILD_WITH_RADIO_BUS */

#endif /* RADIOBUS_H_ */
