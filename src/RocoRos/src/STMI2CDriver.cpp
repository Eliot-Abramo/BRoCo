/*
 * STMI2CDriver.c
 *
 *  Created on: Jul 13, 2021
 *      Author: arion
 */

#include "STMI2CDriver.h"
#include "Debug/Console.h"
#include "Lang/Operators.h"

#include <cstring>


#ifdef BUILD_WITH_STMI2C

#define GET_ADDR(hi2c) ((uint8_t) hi2c->Init.OwnAddress1)
#define GET_PEER(hi2c) active_peers[GET_ADDR(hi2c)]

static STMI2CDriver* driver_instances[256];
static uint8_t active_peers[256];


void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	static uint8_t temp[I2C_BUFFER_SIZE];

	STMI2CDriver* instance = driver_instances[GET_ADDR(hi2c)];

	if(instance != nullptr) {
		if(GET_PEER(hi2c) == 0) {
			uint32_t header = *((uint32_t*) (hi2c->pBuffPtr - hi2c->XferSize));
			uint8_t sender_id = header & 0xFF;
			uint32_t length = (header >> 8) & 0xFFFFFF;

			if(HAL_I2C_Slave_Receive_IT(hi2c, instance->getBuffer(), length) != HAL_OK) {
				console.printf("[RoCo] [STMI2CDriver] Payload reception failed\r\n");
			}

			GET_PEER(hi2c) = sender_id;
		} else {
			GET_PEER(hi2c) = 0;

			uint32_t length = hi2c->XferSize;
			std::memcpy(temp, instance->getBuffer(), length);

			if(HAL_I2C_Slave_Receive_IT(hi2c, instance->getBuffer(), 4) != HAL_OK) {
				console.printf("[RoCo] [STMI2CDriver] Header reception failed\r\n");
			}

			instance->receiveI2C(GET_PEER(hi2c), temp, length);
		}
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	STMI2CDriver* instance = driver_instances[GET_ADDR(hi2c)];

	if(instance != nullptr) {
		if(GET_PEER(hi2c) != 0) {
			uint32_t header = *((uint32_t*) (hi2c->pBuffPtr - hi2c->XferSize));
			uint32_t length = (header >> 8) & 0xFFFFFF;

			if(HAL_I2C_Master_Transmit_IT(hi2c, GET_PEER(hi2c), instance->getBuffer(), length) != HAL_OK) {
				console.printf("[RoCo] [STMI2CDriver] Payload transmission failed\r\n");
			}

			GET_PEER(hi2c) = 0;
		} else {
			HAL_I2C_DeInit(hi2c);
			HAL_I2C_Init(hi2c);

			if(HAL_I2C_Slave_Receive_IT(hi2c, instance->getBuffer(), 4) != HAL_OK) {
				console.printf("[RoCo] [STMI2CDriver] Header reception failed\r\n");
			}
		}
	}
}


STMI2CDriver::STMI2CDriver(I2CDevice* dev, I2C_HandleTypeDef* hi2c) : I2CDriver(dev), hi2c(hi2c), last_dma_index(0), recvThread(nullptr) {
	driver_instances[GET_ADDR(hi2c)] = this;

	this->buffer = (uint8_t*) pvPortMalloc(I2C_BUFFER_SIZE);

	if(buffer == nullptr) {
		console.printf("[RoCo] [STMI2CDriver] Unable to allocate DMA buffer\r\n");
	} else if(dev->getType() == MASTER){
		//HAL_StatusTypeDef status = HAL_I2C_Slave_Receive_IT(hi2c, dma_buffer, 1);
	} else if(dev->getType() == SLAVE) {
	}

	if(HAL_I2C_Slave_Receive_IT(hi2c, this->buffer, 4) != HAL_OK) {
		console.printf("[RoCo] [STMI2CDriver] Header reception failed\r\n");
	}
}

STMI2CDriver::~STMI2CDriver() {
	driver_instances[(uint8_t) hi2c->Init.OwnAddress1] = nullptr;
	vPortFree(buffer);
	delete this->recvThread;
}

void STMI2CDriver::transmitI2C(uint8_t target, uint8_t* data, uint32_t length) {
	uint8_t sender_id = GET_ADDR(hi2c);
	uint32_t header = ((length & 0xFFFFFF) << 8) | sender_id;

	if(dev->getType() == MASTER) {
		uint32_t count = 65536;
		while(GET_PEER(hi2c) != 0 && --count > 0) { // Busy reading
			portYIELD();
		}

		if(count == 0) {
			console.printf("[RoCo] [STMI2CDriver] Forcing transmission\r\n");
		}

		// Enter master mode
		HAL_I2C_DeInit(hi2c);
		HAL_I2C_Init(hi2c);

		std::memcpy(buffer, data, length);
		GET_PEER(hi2c) = target;

		if(HAL_I2C_Master_Transmit_IT(hi2c, target, (uint8_t*) &header, 4) != HAL_OK) {   // Send packet ID
			console.printf("[RoCo] [STMI2CDriver] Header transmission failed\r\n");
		}

	} else if(dev->getType() == SLAVE) {
		if(HAL_I2C_Slave_Transmit_DMA(hi2c, data, length) != HAL_OK) {
			console.printf("[RoCo] [STMI2CDriver] Payload transmission failed\r\n");
		}
	} else {
		console.printf("[RoCo] [STMI2CDriver] Device type invalid\r\n");
	}

}

uint8_t* STMI2CDriver::getBuffer() {
	return buffer;
}

//void STMI2CDriver::receiveI2C(uint8_t source, std::function<void (uint8_t* buffer, uint32_t length)> callback) {
//
//	if(dev->getType() == MASTER) { // Unfortunately cannot use DMA in master mode since multiple slaves must be managed
//
//		/*osDelay(10 / portTICK_PERIOD_MS);
//		uint32_t end_dma_index = I2C_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hi2c->hdmarx);
//		console.printf("[RoCo] %d\r\n", end_dma_index - last_dma_index);
//		HAL_I2C_Master_Abort_IT(hi2c, 0);
//
//
//
//		this->last_dma_index = end_dma_index;*/
//
//		//callback(dma_buffer, 1 - hi2c->XferCount);
//	} else {
//		if(dma_buffer != nullptr && hi2c->SlaveRxCpltCallback == &noRXCallback) {
//			uint32_t end_dma_index = I2C_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(hi2c->hdmarx);
//
//			if(end_dma_index - last_dma_index > 0) {
//				console.printf("[RoCo] %d bytes received\r\n", end_dma_index - last_dma_index);
//				callback(&dma_buffer[last_dma_index], end_dma_index - last_dma_index);
//			}
//
//
//			this->last_dma_index = end_dma_index;
//		}
//	}
//}

//void HAL_I2C_MasterRxCpltCallback()

#endif /* BUILD_WITH_STMI2C */
