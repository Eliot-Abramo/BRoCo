#ifndef ZEPHYR_UART_DRIVER
#define ZEPHYR_UART_DRIVER

#include "Build/Build.h"

#ifdef BUILD_WITH_ZEPHYR_UART_DRIVER


#include "IODriver.h"

#include <Zephyr.h>
#include <device.h>
#include <functional>


#define ZEPHYR_UART_DRIVER_BUFFER_SIZE 256

class ZephyrUARTDriver : public IODriver {
public:
    ZephyrUARTDriver(const struct device *dev);

    void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

    std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver_func;
    uint8_t tx_buffer[ZEPHYR_UART_DRIVER_BUFFER_SIZE];
    uint32_t tx_index = 0;

private:
    const struct device *dev;
    uint8_t rx_buffer[ZEPHYR_UART_DRIVER_BUFFER_SIZE];
};

#endif /* BUILD_WITH_ZEPHYR_UART_DRIVER */

#endif /* ZEPHYR_UART_DRIVER */
