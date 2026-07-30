#include "vision_uart.h"
#include "uart_ring.h"
#include "ti_msp_dl_config.h"
static UartRing g_rx;
void vision_uart_init(void)
{
    uart_ring_init(&g_rx);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(VISION_UART_INST_INT_IRQN);
}
bool vision_uart_read(uint8_t *byte) { return uart_ring_pop(&g_rx, byte); }
void vision_uart_write(const uint8_t *data, size_t len)
{
    size_t i;
    for (i = 0; i < len; ++i) DL_UART_Main_transmitDataBlocking(VISION_UART_INST, data[i]);
}
uint32_t vision_uart_rx_overflow(void) { return g_rx.overflow; }
void VISION_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(VISION_UART_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            (void) uart_ring_push_isr(&g_rx,
                (uint8_t) DL_UART_Main_receiveDataBlocking(VISION_UART_INST));
            break;
        default: break;
    }
}
