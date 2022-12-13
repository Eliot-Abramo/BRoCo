#ifndef ESPRESSIF_UART_DRIVER
#define ESPRESSIF_UART_DRIVER

#include "Build/Build.h"

#ifdef BUILD_WITH_ESP_UART_DRIVER


#include "IODriver.h"
#include "../../System/Thread.h"

#include <driver/uart.h>
#include <functional>


#define ESPRESSIF_UART_DRIVER_BUFFER_SIZE 256

class EspressifUARTDriver : public IODriver, public Thread {
public:
	EspressifUARTDriver(const uart_port_t dev);

    void receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver);
	void transmit(uint8_t* buffer, uint32_t length);

    std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> receiver_func;

private:
    const uart_port_t dev;
	uint8_t rx_buffer[ESPRESSIF_UART_DRIVER_BUFFER_SIZE];

    void tick();
};

#endif /* BUILD_WITH_ESPRESSIF_UART_DRIVER */

#endif /* ESPRESSIF_UART_DRIVER */
