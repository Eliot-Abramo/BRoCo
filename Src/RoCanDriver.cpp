/*
 * RoCanDriver.cpp
 *
 *  Created on: Jun 4, 2023
 *      Author: YassineBakkali
 */


#include "RoCanDriver.h"
#include "Debug/Debug.h"
#include "Lang/Operators.h"


#include <cstring>
#include <inttypes.h>
#include "stdio.h"
#include <algorithm>

#ifdef BUILD_WITH_FDCAN


static ROCANDriver* instance;
static xSemaphoreHandle semaphore;

uint8_t dlc2len[]={0,1,2,3,4,5,6,7,8,12,16,20,24,32,48,64};

uint8_t can_dlc2len(uint32_t RxHeader_DataLength)
{
	return dlc2len[RxHeader_DataLength>>16];
}

ROCANDriver::ROCANDriver(FDCAN_HandleTypeDef* fdcan, uint32_t can_id): Thread("ROCANDriver", osPriorityRealtime), fdcan(fdcan), can_id(can_id){
	instance = this;
	this->RxData = (uint8_t*) pvPortMalloc(RX_BUFFER_SIZE);
	this->TxData = (uint8_t*) pvPortMalloc(TX_BUFFER_SIZE);

    if((TxData == nullptr) || (RxData == nullptr)){
        printf("[RoCo] [ROCANDriver] Unable to allocate Rx/Tx buffers for MCU#%" PRIu32 "\r\n", getSenderID(fdcan));
    }

    semaphore = xSemaphoreCreateCounting(16, 0);

    if(semaphore == nullptr) {
        printf("[RoCo] [ROCANDriver] Unable to allocate semaphore for MCU#%" PRIu32 "\r\n", getSenderID(fdcan));
    }
    setTickDelay(0);
}

ROCANDriver::~ROCANDriver() {
    vPortFree(TxData);
    vPortFree(RxData);
    FDCANDriver_list.erase(std::remove(FDCANDriver_list.begin(), FDCANDriver_list.end(), this), FDCANDriver_list.end());
}

void ROCANDriver::init() {

	/* Configure Rx filter */
	if(fdcan == &hfdcan1){
			MX_FDCAN1_Init();
	}
	filterConfig();
	/* Start the FDCan line */
	start();
}

void ROCANDriver::filterConfig(){

	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = 0x0;
	sFilterConfig.FilterID2 = 0x0;

	if(HAL_FDCAN_ConfigFilter(fdcan, &sFilterConfig) != HAL_OK)
		printf("[RoCo] [ROCANDriverFilterConf] Unable to configure CAN RX filters");


	HAL_FDCAN_ConfigGlobalFilter(fdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_ConfigTxDelayCompensation(fdcan, fdcan->Init.DataPrescaler * fdcan->Init.DataTimeSeg1, 0);
	HAL_FDCAN_EnableTxDelayCompensation(fdcan);

}

void ROCANDriver::TxHeaderConfig(uint32_t can_id, uint32_t DataLength){
	TxHeader.Identifier = can_id;
	TxHeader.IdType = FDCAN_EXTENDED_ID;
	if(can_id < 0x800) {
		TxHeader.IdType = FDCAN_STANDARD_ID;
	}
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_64;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = FDCAN_FD_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
}

void ROCANDriver::start(){
	if(HAL_FDCAN_Start(fdcan) != HAL_OK)
		printf("[RoCo] [ROCANDriverStart] Couldn't start FDCAN module");

}


void ROCANDriver::loop() {
	if(xSemaphoreTake(semaphore, portMAX_DELAY)) {
//		uint32_t end_dma_index = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);
		uint8_t sender = getSenderID(fdcan);
		if((FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0){
			HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &RxHeader, RxData);
			uint8_t buffer2[RxData[0]];
			for(int i = 0; i < (RxData[0]); ++i)
				buffer2[i] = RxData[i+1];
			receiveFDCan(sender, buffer2, RxData[0]);
		}
	}
}

void ROCANDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) {
    this->receiver_func = receiver;
}

void ROCANDriver::transmit(uint8_t* buffer, uint32_t length) {
	//Example of TxHeader configuration

	static uint32_t true_length = length;
	uint8_t buffer2[true_length+1];
	TxHeaderConfig(0x123, true_length);
	buffer2[0] = true_length;
	for(int i = 0; i < true_length; ++i)
		buffer2[i+1] = buffer[i];
	if(HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, buffer2) != HAL_OK) {
        printf("[RoCo] [ROCANDriverTransmit] Transmission failed for MCU#%" PRIu32 "\r\n", getSenderID(fdcan));
//		memcpy(&fdcan1_send_fail.TxHeader[fdcan_send_fail.index], &TxHeader, sizeof(FDCAN_TxHeaderTypeDef));
//		memcpy(&fdcan1_send_fail.TxHeader[TX_BUFFER_SIZE*fdcan_send_fail.index], tx_data, can_dlc2len(DataLength));
//		fdcan1_send_fail.index = (fdcan1_send_fail.index + 1) & 0x0F;
//		fdcan1_send_fail.flag = 1;
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

uint8_t* ROCANDriver::getTxBuffer() {
	return this->TxData;
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

// RxCallback of FDCAN1
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	ROCANDriver* driver = instance->getInstance(hfdcan);
	xSemaphoreGiveFromISR(driver->getSemaphore(), nullptr);
}
// RxCallback of FDCAN2
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint16_t Size) {
	ROCANDriver* driver = instance->getInstance(hfdcan);
	xSemaphoreGiveFromISR(driver->getSemaphore(), nullptr);
}


void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if(hfdcan == &hfdcan1){
		MX_FDCAN1_Init();
		ROCANDriver* driver = instance->getInstance(hfdcan);
		while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
		instance->filterConfig();
		instance->start();
	} else {
//		MX_FDCAN2_Init();
		ROCANDriver* driver = instance->getInstance(hfdcan);
		while(xSemaphoreTakeFromISR(driver->getSemaphore(), nullptr)); // Clear semaphore
		instance->filterConfig();
		instance->start();
	}
}

ROCANDriver* ROCANDriver::getInstance(FDCAN_HandleTypeDef* fdcan) {
	for (auto & driver : FDCANDriver_list) {
		if (driver->getFDCan() == fdcan)
			return driver;
	}
}

/**
 * @brief Get the sender id from the FDCAN port ID
 *
 * @param huart the FDCAN port to get
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

