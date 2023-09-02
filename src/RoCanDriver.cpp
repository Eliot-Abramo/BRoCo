/*
 * RoCanDriver.cpp
 *
 *  Created on: Jun 4, 2023
 *      Author: YassineBakkali
 */

#include "RoCanDriver.h"

#ifdef BUILD_WITH_FDCAN
#include "Debug.h"
#include "Operators.h"


#include <cstring>
#include <inttypes.h>
#include "stdio.h"
#include <algorithm>


static ROCANDriver* instance;
static xSemaphoreHandle semaphore;

ROCANDriver::ROCANDriver(FDCAN_HandleTypeDef* fdcan, uint32_t can_id): Thread("ROCANDriver", osPriorityRealtime, 4096), fdcan(fdcan), can_id(can_id) {
	instance = this;
	LOG_INFO("Driver created for FDCAN%d", getSenderID(fdcan));
	FDCANDriver_list.push_back(this);
	this->RxData = (uint8_t*) pvPortMalloc(RX_ELEMENT_NUMBER*RX_ELEMENT_SIZE);

    if((RxData == nullptr)){
        LOG_ERROR("Unable to allocate Rx buffer for FDCAN%d", getSenderID(fdcan));
    }

    semaphore = xSemaphoreCreateCounting(16, 0);

    if(semaphore == nullptr) {
    	LOG_ERROR("Unable to allocate semaphore for FDCAN%d", getSenderID(fdcan));
    }
    setTickDelay(0);
}

ROCANDriver::~ROCANDriver() {
    vPortFree(RxData);
    FDCANDriver_list.erase(std::remove(FDCANDriver_list.begin(), FDCANDriver_list.end(), this), FDCANDriver_list.end());
}

void ROCANDriver::init() {

	/* Configure Rx filter */
	if(fdcan == &hfdcan1)
		MX_FDCAN1_Init();
	else if (fdcan == &hfdcan2)
		MX_FDCAN2_Init();
	/* Start the FDCan line */
	filterConfig(can_id);
	TxHeaderConfig();
	TxHeaderConfigID(0);
	start();
}

void ROCANDriver::filterConfig(uint32_t id){

	// Node ID (can_id)
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = id;
	sFilterConfig.FilterID2 = 0x7FF;

	if(HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK)
		LOG_ERROR("Unable to configure CAN RX filters index 0 for FDCAN%d", getSenderID(fdcan));

	// General ID (0x7FF)
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 1;
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x7FF;
	sFilterConfig.FilterID2 = 0x7FF;

	if(HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK)
		LOG_ERROR("Unable to configure CAN RX filters index 1 for FDCAN%d", getSenderID(fdcan));

	HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ConfigTxDelayCompensation(fdcan, fdcan->Init.DataPrescaler * fdcan->Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(fdcan);

}

uint32_t ROCANDriver::get_can_id() {
	return can_id;
}

void ROCANDriver::TxHeaderConfig(){
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void ROCANDriver::TxHeaderConfigID(uint32_t id){
	TxHeader.Identifier = id;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	if(id < 0x800) {
		TxHeader.IdType = FDCAN_STANDARD_ID;
	}
}

void ROCANDriver::TxHeaderConfigLength(uint32_t length){
	TxHeader.DataLength = len2dlc(length);
}

void ROCANDriver::start(){
	if(HAL_FDCAN_Start(fdcan) != HAL_OK)
		LOG_ERROR("Couldn't start FDCAN%d module", getSenderID(fdcan));
}

uint32_t ROCANDriver::len2dlc(uint32_t length, bool return_raw) {

	if (return_raw) {
		if (length <= 8)
			return length;
		else if (length <= 12)
			return 12;
		else if (length <= 16)
			return 16;
		else if (length <= 20)
			return 20;
		else if (length <= 24)
			return 24;
		else if (length <= 32)
			return 32;
		else if (length <= 48)
			return 48;
		else if (length <= 64)
			return 64;
		else
			return 0;
	}
	else {
		// Standard lengths
		switch (length) {
		case 0:
			return FDCAN_DLC_BYTES_0;
		case 1:
			return FDCAN_DLC_BYTES_1;
		case 2:
			return FDCAN_DLC_BYTES_2;
		case 3:
			return FDCAN_DLC_BYTES_3;
		case 4:
			return FDCAN_DLC_BYTES_4;
		case 5:
			return FDCAN_DLC_BYTES_5;
		case 6:
			return FDCAN_DLC_BYTES_6;
		case 7:
			return FDCAN_DLC_BYTES_7;
		case 8:
			return FDCAN_DLC_BYTES_8;
		}

		// Extended lengths
		if (length <= 12)
			return FDCAN_DLC_BYTES_12;
		else if (length <= 16)
			return FDCAN_DLC_BYTES_16;
		else if (length <= 20)
			return FDCAN_DLC_BYTES_20;
		else if (length <= 24)
			return FDCAN_DLC_BYTES_24;
		else if (length <= 32)
			return FDCAN_DLC_BYTES_32;
		else if (length <= 48)
			return FDCAN_DLC_BYTES_48;
		else if (length <= 64)
			return FDCAN_DLC_BYTES_64;
		else
			return 0;
	}
}

uint32_t ROCANDriver::dlc2len(uint32_t dlc) {
	switch (dlc) {
	case FDCAN_DLC_BYTES_0:
		return 0;
	case FDCAN_DLC_BYTES_1:
		return 1;
	case FDCAN_DLC_BYTES_2:
		return 2;
	case FDCAN_DLC_BYTES_3:
		return 3;
	case FDCAN_DLC_BYTES_4:
		return 4;
	case FDCAN_DLC_BYTES_5:
		return 5;
	case FDCAN_DLC_BYTES_6:
		return 6;
	case FDCAN_DLC_BYTES_7:
		return 7;
	case FDCAN_DLC_BYTES_8:
		return 8;
	case FDCAN_DLC_BYTES_12:
		return 12;
	case FDCAN_DLC_BYTES_16:
		return 16;
	case FDCAN_DLC_BYTES_20:
		return 20;
	case FDCAN_DLC_BYTES_24:
		return 24;
	case FDCAN_DLC_BYTES_32:
		return 32;
	case FDCAN_DLC_BYTES_48:
		return 48;
	case FDCAN_DLC_BYTES_64:
		return 64;
	}
	return 0;
}


void ROCANDriver::loop() {
	if(xSemaphoreTake(semaphore, portMAX_DELAY)) {
		uint8_t sender = getSenderID(fdcan);
		if((FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0){
			uint32_t fill_level = HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0);
			while (fill_level > 0) {
				HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
				uint32_t length = dlc2len(RxHeader.DataLength);
				receiveFDCan(sender, RxData, length);
				fill_level = HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0);
			}
		}
	}
}

void ROCANDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
    this->receiver_func = receiver;
}

void ROCANDriver::transmit(uint8_t* buffer, uint32_t length) {
	// Extra condition because HAL_FDCAN_AddMessageToTxFifoQ introduces delay when not HAL_OK
	// Important to keep trying to send messages to decrease error count to escape Error Passive
	// or Bus Off state
//	if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0) {
//		// First check status of CAN bus
//		uint32_t dlc_len = len2dlc(length, true);
//		for (uint32_t i = 0; i < dlc_len; ++i) {
//			buffer[i] = 0;
//		}
//		if (dlc_len > 20) {
//			for (uint32_t i = dlc_len-10; i < dlc_len; ++i) {
//						buffer[i] = 1;
//					}
//		}
//		TxHeaderConfigLength(length);
//		// Normally always HAL_OK if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0
//		if(HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, buffer) != HAL_OK) {
//			LOG_ERROR("Transmission failed on FDCAN%d", getSenderID(fdcan));
//		}
//	}

	if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0) {
		// First check status of CAN bus
		uint32_t dlc_len = len2dlc(length, true);
		for (uint32_t i = length; i < dlc_len; ++i) {
			buffer[i] = 0;
		}
		TxHeaderConfigLength(length);
		// Normally always HAL_OK if (HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0
		if(HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, buffer) != HAL_OK) {
			LOG_ERROR("Transmission failed on FDCAN%d", getSenderID(fdcan));
		}
	}
}


/**
 * @brief Getters to the reference of the buffer
 *
 * @return uint8_t* the reference to the buffer
 */
uint8_t* ROCANDriver::getRxBuffer() {
	return this->RxData;
}

ROCANDriver* ROCANDriver::getInstance(FDCAN_HandleTypeDef* fdcan) {
	for (auto & driver : FDCANDriver_list) {
		if (driver->getFDCan() == fdcan)
			return driver;
	}
	return nullptr;
}

/**
 * @brief Function handling the call to the user-defined callback routine
 *
 * @param sender_id the ID of the MCU
 * @param buffer the buffer to provide to the user-defined callback function
 * @param length the size of the data in the buffer to provide
 */
void ROCANDriver::receiveFDCan(uint8_t sender_id, uint8_t* buffer, uint32_t length) {
	this->receiver_func(sender_id, buffer, length);
}

FDCAN_HandleTypeDef* ROCANDriver::getFDCan() {
	return this->fdcan;
}

// Fifo0 Rx Callback
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	ROCANDriver* driver = ROCANDriver::getInstance(hfdcan);
	xSemaphoreGiveFromISR(driver->getSemaphore(), nullptr);
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if(hfdcan == &hfdcan1){
		MX_FDCAN1_Init();
		ROCANDriver* driver = ROCANDriver::getInstance(hfdcan);
		while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
		driver->filterConfig(driver->get_can_id());
		driver->start();
	} else if (hfdcan == &hfdcan2) {
		MX_FDCAN2_Init();
		ROCANDriver* driver = ROCANDriver::getInstance(hfdcan);
		while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
		driver->filterConfig(driver->get_can_id());
		driver->start();
	}
}

/**
 * @brief Get the sender id from the FDCAN port ID
 *
 * @param fdcan the FDCAN port to get
 * @return uint8_t the sender_id
 */
uint8_t ROCANDriver::getSenderID(FDCAN_HandleTypeDef* fdcan) {
    for(int i = 0; i < NB_CAN_PORTS; ++i){
        if(this->mapper[i] == fdcan->Instance){
            return i+1;
        }
    }
    return 0;
}


xSemaphoreHandle ROCANDriver::getSemaphore() {
	return semaphore;
}

std::vector<ROCANDriver*> ROCANDriver::FDCANDriver_list;

#endif

