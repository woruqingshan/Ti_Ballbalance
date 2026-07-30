#ifndef PI_TI_PROTOCOL_DIAG_TEST_H
#define PI_TI_PROTOCOL_DIAG_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "protocol/pi_ti_messages.h"
#include "protocol/protocol.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/*
 * Finite UART0/protocol isolation modes.
 *
 * Mode 15: one-shot TX only. It sends a compile-time known-good binary frame,
 * then creates the same frame through protocol_encode(), sends it once and
 * halts. No RX processing, heartbeat loop, UART1 or motor access.
 *
 * Mode 16: one-shot request/response using direct UART FIFO polling with the
 * UART0 NVIC interrupt disabled. It waits up to 5 seconds for one valid frame,
 * replies once with COMMAND_ACK and halts.
 *
 * Mode 17: the same one-shot request/response through the existing RX ISR and
 * ring buffer. Comparing mode 16 and 17 isolates ISR/ring-buffer faults.
 */

static void pi_ti_diag_halt(void)
{
    for (;;) {
        __WFI();
    }
}

static size_t pi_ti_diag_build_heartbeat(uint8_t *frame,
                                        size_t capacity,
                                        uint32_t sequence,
                                        uint32_t timestamp_ms)
{
    uint8_t payload[8];
    protocol_put_u32_le(&payload[0], APP_MODE);
    protocol_put_u32_le(&payload[4], timestamp_ms);
    return protocol_encode(MSG_HEARTBEAT,
                           sequence,
                           timestamp_ms,
                           payload,
                           sizeof(payload),
                           frame,
                           capacity);
}

static void pi_ti_protocol_tx_diag_run(void)
{
    /* Mode=15, sequence=0, timestamp=0, CRC16-CCITT-FALSE=0x0B6F. */
    static const uint8_t known_good_frame[24] = {
        0xA5U, 0x5AU, 0x01U, 0x83U, 0x08U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x0FU, 0x00U, 0x00U, 0x00U,
        0x00U, 0x00U, 0x00U, 0x00U, 0x6FU, 0x0BU
    };
    uint8_t encoded[PROTOCOL_MAX_FRAME];
    size_t encoded_len;

    test_console_write_line("P15_BEGIN");

    vision_uart_write(known_good_frame, sizeof(known_good_frame));
    test_console_newline();
    test_console_write_line("P15_RAW_SENT");

    test_console_write_line("P15_BEFORE_ENCODE");
    encoded_len = pi_ti_diag_build_heartbeat(encoded, sizeof(encoded), 0U, 0U);
    if (encoded_len != sizeof(known_good_frame)) {
        test_console_write_line("P15_ENCODE_LENGTH_FAIL");
        pi_ti_diag_halt();
    }
    if (memcmp(encoded, known_good_frame, sizeof(known_good_frame)) != 0) {
        test_console_write_line("P15_ENCODE_BYTES_FAIL");
        vision_uart_write(encoded, encoded_len);
        test_console_newline();
        pi_ti_diag_halt();
    }

    test_console_write_line("P15_ENCODE_MATCH");
    vision_uart_write(encoded, encoded_len);
    test_console_newline();
    test_console_write_line("P15_DONE");
    pi_ti_diag_halt();
}

static void pi_ti_diag_send_ack_for_frame(const ProtocolFrame *received,
                                          uint32_t now_ms)
{
    uint8_t ack_payload[PI_TI_ACK_PAYLOAD_SIZE];
    uint8_t tx_frame[PROTOCOL_MAX_FRAME];
    uint8_t command = 0U;
    uint8_t result = PI_ACK_BAD_COMMAND;
    uint32_t token = 0U;
    int32_t detail = (int32_t) received->type;
    size_t tx_len;

    if ((received->type == MSG_CONTROL_COMMAND) &&
        (received->payload_len == 8U)) {
        command = received->payload[0];
        token = protocol_get_u32_le(&received->payload[4]);
        result = PI_ACK_OK;
        detail = PI_TI_PROTOCOL_PROFILE_VERSION;
    }

    pi_ti_pack_ack(ack_payload, command, result, detail, token);
    tx_len = protocol_encode(MSG_COMMAND_ACK,
                             0U,
                             now_ms,
                             ack_payload,
                             sizeof(ack_payload),
                             tx_frame,
                             sizeof(tx_frame));
    if (tx_len != 0U) {
        vision_uart_write(tx_frame, tx_len);
    }
}

static bool pi_ti_diag_wait_polling(ProtocolFrame *received,
                                    uint32_t timeout_ms)
{
    ProtocolParser parser;
    uint32_t started_ms = bsp_time_ms();
    uint8_t ignored;

    /* Stop the existing RX ISR; this mode reads the hardware FIFO directly. */
    NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);

    /* Discard bytes already copied into the ring before IRQ disable. */
    while (vision_uart_read(&ignored)) {
    }
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        (void) DL_UART_Main_receiveData(VISION_UART_INST);
    }

    protocol_parser_init(&parser);
    while ((uint32_t) (bsp_time_ms() - started_ms) < timeout_ms) {
        if (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
            uint8_t byte = (uint8_t) DL_UART_Main_receiveData(VISION_UART_INST);
            if (protocol_parser_push(&parser, byte, received)) {
                return true;
            }
        }
    }
    return false;
}

static bool pi_ti_diag_wait_irq(ProtocolFrame *received,
                                uint32_t timeout_ms)
{
    ProtocolParser parser;
    uint32_t started_ms = bsp_time_ms();
    uint8_t ignored;

    while (vision_uart_read(&ignored)) {
    }
    protocol_parser_init(&parser);

    while ((uint32_t) (bsp_time_ms() - started_ms) < timeout_ms) {
        uint8_t byte;
        while (vision_uart_read(&byte)) {
            if (protocol_parser_push(&parser, byte, received)) {
                return true;
            }
        }
        __WFI();
    }
    return false;
}

static void pi_ti_protocol_rx_poll_diag_run(void)
{
    ProtocolFrame received;
    uint32_t now_ms;

    test_console_write_line("P16_POLL_READY_SEND_ONE_HELLO");
    if (!pi_ti_diag_wait_polling(&received, APP_PI_PROTOCOL_DIAG_TIMEOUT_MS)) {
        test_console_write_line("P16_TIMEOUT");
        pi_ti_diag_halt();
    }

    test_console_write_line("P16_FRAME_OK");
    now_ms = bsp_time_ms();
    pi_ti_diag_send_ack_for_frame(&received, now_ms);
    test_console_newline();
    test_console_write_line("P16_REPLY_SENT");
    pi_ti_diag_halt();
}

static void pi_ti_protocol_rx_irq_diag_run(void)
{
    ProtocolFrame received;
    uint32_t now_ms;

    test_console_write_line("P17_IRQ_READY_SEND_ONE_HELLO");
    if (!pi_ti_diag_wait_irq(&received, APP_PI_PROTOCOL_DIAG_TIMEOUT_MS)) {
        test_console_write_line("P17_TIMEOUT");
        pi_ti_diag_halt();
    }

    test_console_write_line("P17_FRAME_OK");
    now_ms = bsp_time_ms();
    pi_ti_diag_send_ack_for_frame(&received, now_ms);
    test_console_newline();
    test_console_write_line("P17_REPLY_SENT");
    pi_ti_diag_halt();
}

#endif
