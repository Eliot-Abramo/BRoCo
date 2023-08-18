/*
 * RoCanDriver.h
 *
 *  Created on: Jun 4, 2023
 *      Author: YassineBakkali
 */

#ifndef BROCO_SRC_ROCANDRIVER_H_
#define BROCO_SRC_ROCANDRIVER_H_

#include "Build/Build.h"
#include "IODriver.h"


#ifdef BUILD_WITH_FDCAN

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

#include <Thread.h>
#include "fdcan.h"
#include <vector>

#define RX_ELEMENT_SIZE		  64 				  // RANGE : 0...64
#define RX_ELEMENT_NUMBER	  32
#define NB_CAN_PORTS           2                  // CHANGE ONLY IF NEEDED

class ROCANDriver: public IODriver,  public Thread{
    public:
		ROCANDriver(FDCAN_HandleTypeDef* fdcan, uint32_t can_id); // Constructor
        virtual ~ROCANDriver(); // Destructor
        uint8_t* getRxBuffer();
        uint8_t* getTxBuffer();

        xSemaphoreHandle getSemaphore();
        uint8_t getSenderID(FDCAN_HandleTypeDef* fdcan);

        void init();
        void loop();

        void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver) override;
        void transmit(uint8_t* buffer, uint32_t length);

        void receiveFDCan(uint8_t sender_id, uint8_t* buffer, uint32_t length);
        FDCAN_HandleTypeDef* getFDCan();
        static ROCANDriver* getInstance(FDCAN_HandleTypeDef* fdcan);
        void TxHeaderConfig();
        void TxHeaderConfigID(uint32_t can_id);
        void TxHeaderConfigLength(uint32_t length);
        void filterConfig(uint32_t can_id);
        uint32_t get_can_id();
        void start();
        uint32_t len2dlc(uint32_t length, bool return_raw = false);
        uint32_t dlc2len(uint32_t dlc);

        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t* RxData;
        FDCAN_TxHeaderTypeDef TxHeader;

    private:
        static std::vector<ROCANDriver*> FDCANDriver_list;
        FDCAN_HandleTypeDef* fdcan;

        FDCAN_GlobalTypeDef* mapper[NB_CAN_PORTS] = {FDCAN1, FDCAN2};

        FDCAN_FilterTypeDef sFilterConfig;

        uint32_t can_id = 0x7FF;

        std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver_func; // User-defined callback function

};

#endif

#endif /* BROCO_SRC_ROCANDRIVER_H_ */
