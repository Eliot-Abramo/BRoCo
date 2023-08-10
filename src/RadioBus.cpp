/*
 * NetworkBus.cpp
 *
 *  Created on: 23 March 2022
 *      Author: Arion
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_RADIO_BUS


#include "RadioBus.h"
#include "Protocol/Protocol.h"

RadioBus::RadioBus(IODriver* driver) : IOBus(driver, buffer, RADIO_BUS_FRAME_SIZE) {
    define<ConnectPacket>(1);
    define<DopplerPacket>(2);
}


#endif /* BUILD_WITH_RADIO_BUS */
