#ifndef PI_TI_UART0_DIAG_TEST_H
#define PI_TI_UART0_DIAG_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * UART0-only hardware diagnostic.
 *
 * This mode deliberately avoids VisionLink, CRC, UART1 and Emm V5.0. It uses
 * the same proven ASCII console path as the phase-3/4 motor tests. PA10 sends
 * a banner immediately and then every 500 ms. Bytes received on PA11 are
 * echoed back so TX and RX can be checked with a CH340 before testing the
 * binary protocol.
 */
static void pi_ti_uart0_diag_test_run(void)
{
    uint32_t last_report_ms = bsp_time_ms();

    test_console_write_line("");
    test_console_write_line("=== PI-TI UART0 DIAG MODE 14 ===");
    test_console_write_line("UART0 PA10 TX / PA11 RX, 115200 8N1");
    test_console_write_line("Send any byte: TI echoes it on PA10.");
    test_console_write_line("TI_UART0_DIAG_READY");

    for (;;) {
        uint8_t byte;
        uint32_t now_ms = bsp_time_ms();

        while (vision_uart_read(&byte)) {
            vision_uart_write(&byte, 1U);
        }

        if ((uint32_t) (now_ms - last_report_ms) >=
            APP_PI_UART0_DIAG_PERIOD_MS) {
            last_report_ms = now_ms;
            test_console_write("TI_UART0_DIAG_ALIVE ms=");
            test_console_write_u32(now_ms);
            test_console_newline();
        }

        __WFI();
    }
}

#endif
