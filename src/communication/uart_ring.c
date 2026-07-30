#include "uart_ring.h"
void uart_ring_init(UartRing *r) { r->head = r->tail = 0U; r->overflow = 0U; }
bool uart_ring_push_isr(UartRing *r, uint8_t b)
{
    uint16_t next = (uint16_t)((r->head + 1U) % UART_RING_CAPACITY);
    if (next == r->tail) { r->overflow++; return false; }
    r->data[r->head] = b; r->head = next; return true;
}
bool uart_ring_pop(UartRing *r, uint8_t *b)
{
    if (r->tail == r->head) return false;
    *b = r->data[r->tail]; r->tail = (uint16_t)((r->tail + 1U) % UART_RING_CAPACITY); return true;
}
