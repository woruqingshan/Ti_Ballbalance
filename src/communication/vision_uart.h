#ifndef VISION_UART_H
#define VISION_UART_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void vision_uart_init(void);
bool vision_uart_read(uint8_t *byte);
void vision_uart_write(const uint8_t *data, size_t len);
uint32_t vision_uart_rx_overflow(void);
#endif
