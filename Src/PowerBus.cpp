/*
 * PowerBus.cpp
 *
 *  Created on: 23 March 2022
 *      Author: Arion
 */

#include "Build/Build.h"


#ifdef BUILD_WITH_POWER_BUS


#include "PowerBus.h"
#include "Protocol/Protocol.h"


PowerBus::PowerBus(IODriver* driver) : IOBus(driver, buffer, POWER_BUS_FRAME_SIZE) {
    define<PingPacket>(1);
    define<RequestPacket>(2);
    define<ResponsePacket>(3);
    define<ProgressPacket>(4);
    define<PayloadPacket>(5);
    define<Power_BusInfo>(6);
    define<Power_BatteryInfo>(7);
    define<Power_ControllerHealth>(8);
    define<Power_ControllerState>(9);

    define<FlushPacket>(62);
    define<ErrorPacket>(63);

    semaphore = xSemaphoreCreateMutex();
}

bool PowerBus::internal_send(PacketDefinition* def, uint8_t* data) {
	xSemaphoreTake(semaphore, portMAX_DELAY);
	bool ret = IOBus::internal_send(def, data);
	xSemaphoreGive(semaphore);
	return ret;
}

#endif /* BUILD_WITH_POWER_BUS */
