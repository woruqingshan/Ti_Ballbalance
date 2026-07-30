#ifndef UART_RING_H
#define UART_RING_H
#include <stdbool.h>
#include <stdint.h>
#define UART_RING_CAPACITY 256U
typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow;
    uint8_t data[UART_RING_CAPACITY];
} UartRing;
void uart_ring_init(UartRing *r);
bool uart_ring_push_isr(UartRing *r, uint8_t byte);
bool uart_ring_pop(UartRing *r, uint8_t *byte);
#endif
