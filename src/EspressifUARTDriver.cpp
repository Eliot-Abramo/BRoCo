#include "EspressifUARTDriver.h"


#ifdef BUILD_WITH_ESP_UART_DRIVER

#include <cstring>

EspressifUARTDriver::EspressifUARTDriver(const uart_port_t dev) : Thread("uart_driver"), dev(dev) {

}


void EspressifUARTDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver_func) {
    this->receiver_func = receiver_func;
}

void EspressifUARTDriver::tick() {
	int length = 0;
	ESP_ERROR_CHECK(uart_get_buffered_data_len(dev, (size_t*) &length));
	length = uart_read_bytes(dev, rx_buffer, length, 100);
	receiver_func((uint8_t) dev, rx_buffer, length);
}

void EspressifUARTDriver::transmit(uint8_t* buffer, uint32_t length) {
	uart_write_bytes(dev, buffer, length);
	//ESP_ERROR_CHECK(uart_wait_tx_done(dev, 100));
}


#endif /* BUILD_WITH_ESP_UART_DRIVER */
