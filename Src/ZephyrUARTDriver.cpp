#include "ZephyrUARTDriver.h"


#ifdef BUILD_WITH_ZEPHYR_UART_DRIVER

#include <cstring>
#include <drivers/uart.h>

void __internal_callback(const struct device *dev, struct uart_event *evt, void *user_data);

ZephyrUARTDriver::ZephyrUARTDriver(const struct device *dev) : dev(dev) {
    uart_callback_set(dev, &__internal_callback, this);
    uart_rx_enable(dev, rx_buffer, ZEPHYR_UART_DRIVER_BUFFER_SIZE, SYS_FOREVER_US);
}

void __internal_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
    ZephyrUARTDriver* driver = (ZephyrUARTDriver*) user_data;

    if(evt->type == uart_event_type::UART_RX_RDY) {
        driver->receiver_func(0, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
    } else if(evt->type == uart_event_type::UART_TX_DONE) {
        /*uint32_t current_tx_index = (evt->data.tx.len + (uint32_t) evt->data.tx.buf - (uint32_t) driver->tx_buffer) % ZEPHYR_UART_DRIVER_BUFFER_SIZE;

        if(current_tx_index != driver->tx_index) {
            uart_tx(dev, driver->tx_buffer + current_tx_index, driver->tx_index - current_tx_index, SYS_FOREVER_US);
        }*/
    }
}

void ZephyrUARTDriver::receive(const std::function<void (uint8_t sender_id, uint8_t* buffer, uint32_t length)> &receiver_func) {
    this->receiver_func = receiver_func;
}

/*#define SIZE ZEPHYR_UART_DRIVER_BUFFER_SIZE
void ZephyrUARTDriver::transmit(uint8_t* buffer, uint32_t length) {
    if(tx_index + length >= SIZE) {
        memcpy(tx_buffer + tx_index, buffer, SIZE - tx_index);
        memcpy(tx_buffer, buffer + SIZE - tx_index, length - (SIZE - tx_index));
        uart_tx(dev, tx_buffer + tx_index, SIZE - tx_index, SYS_FOREVER_US);
    } else {
        memcpy(tx_buffer + tx_index, buffer, length);
        uart_tx(dev, tx_buffer + tx_index, length, SYS_FOREVER_US);
    }

    tx_index = (tx_index + length) % SIZE;
}
#undef SIZE*/

// Polling implementation
void ZephyrUARTDriver::transmit(uint8_t* buffer, uint32_t length) {
    while(uart_tx(dev, buffer, length, SYS_FOREVER_US) != 0) {
        k_yield();
    }
}


#endif /* BUILD_WITH_ZEPHYR_UART_DRIVER */
