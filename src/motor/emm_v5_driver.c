#include "emm_v5_driver.h"
#include "emm_v5_protocol.h"
#include "ti_msp_dl_config.h"
#include <stddef.h>

#define EMM_ADDRESS                       EMM_V5_DEFAULT_ADDRESS
#define EMM_CHECK                         EMM_V5_CHECK_BYTE
#define EMM_POSITION_TIMEOUT_MS           (100U)

static EmmV5Stats g_stats;
static EmmV5PositionParser g_position_parser;
static bool g_position_request_pending;
static bool g_position_ready;
static uint32_t g_position_request_started_ms;
static EmmV5Position g_latest_position;

static void send_bytes(const uint8_t *data, size_t length)
{
    size_t i;
    for (i = 0U; i < length; ++i) {
        DL_UART_Main_transmitDataBlocking(EMM_UART_INST, data[i]);
    }
}

static void flush_rx(void)
{
    while (!DL_UART_Main_isRXFIFOEmpty(EMM_UART_INST)) {
        (void) DL_UART_Main_receiveData(EMM_UART_INST);
    }
}

void emm_v5_init(void)
{
    g_stats.sent = 0U;
    g_stats.rejected = 0U;
    g_stats.position_requests = 0U;
    g_stats.position_responses = 0U;
    g_stats.position_timeouts = 0U;
    g_stats.position_bad_frames = 0U;
    g_position_request_pending = false;
    g_position_ready = false;
    g_position_request_started_ms = 0U;
    g_latest_position.raw_units = 0;
    g_latest_position.received_timestamp_ms = 0U;
    emm_v5_position_parser_init(&g_position_parser);
}

void emm_v5_poll(uint32_t now_ms)
{
    int32_t position_units;

    while (!DL_UART_Main_isRXFIFOEmpty(EMM_UART_INST)) {
        uint8_t byte = (uint8_t) DL_UART_Main_receiveData(EMM_UART_INST);
        if (emm_v5_position_parser_push(&g_position_parser,
                                        EMM_ADDRESS,
                                        byte,
                                        &position_units)) {
            g_latest_position.raw_units = position_units;
            g_latest_position.received_timestamp_ms = now_ms;
            g_position_ready = true;
            g_position_request_pending = false;
            g_stats.position_responses++;
        }
    }

    g_stats.position_bad_frames = g_position_parser.bad_frames;
    if (g_position_request_pending &&
        ((uint32_t) (now_ms - g_position_request_started_ms) >=
         EMM_POSITION_TIMEOUT_MS)) {
        g_position_request_pending = false;
        g_stats.position_timeouts++;
        emm_v5_position_parser_init(&g_position_parser);
    }
}

bool emm_v5_enable(bool enable)
{
    const uint8_t frame[6] = {
        EMM_ADDRESS, 0xF3U, 0xABU, enable ? 0x01U : 0x00U, 0x00U, EMM_CHECK
    };
    send_bytes(frame, sizeof(frame));
    g_stats.sent++;
    return true;
}

bool emm_v5_clear_current_position(void)
{
    const uint8_t frame[4] = {
        EMM_ADDRESS, 0x0AU, 0x6DU, EMM_CHECK
    };
    send_bytes(frame, sizeof(frame));
    g_stats.sent++;
    return true;
}

bool emm_v5_move_relative(int32_t pulses,
                          uint16_t rpm,
                          uint8_t accel)
{
    uint32_t magnitude;
    uint8_t frame[13];

    if (pulses == 0) {
        return true;
    }

    magnitude = (uint32_t) ((pulses < 0) ? -pulses : pulses);
    frame[0] = EMM_ADDRESS;
    frame[1] = 0xFDU;
    frame[2] = (pulses < 0) ? 0x01U : 0x00U;
    frame[3] = (uint8_t) (rpm >> 8);
    frame[4] = (uint8_t) rpm;
    frame[5] = accel;
    frame[6] = (uint8_t) (magnitude >> 24);
    frame[7] = (uint8_t) (magnitude >> 16);
    frame[8] = (uint8_t) (magnitude >> 8);
    frame[9] = (uint8_t) magnitude;

    /* Emm V5.0: 0x00 = relative position, 0x01 = absolute position. */
    frame[10] = 0x00U;
    frame[11] = 0x00U; /* no multi-axis synchronization */
    frame[12] = EMM_CHECK;
    send_bytes(frame, sizeof(frame));
    g_stats.sent++;
    return true;
}

bool emm_v5_request_current_position(uint32_t now_ms)
{
    uint8_t command[3];
    size_t command_length;

    if (g_position_request_pending) {
        g_stats.rejected++;
        return false;
    }

    flush_rx();
    emm_v5_position_parser_init(&g_position_parser);
    command_length = emm_v5_build_read_current_position(
        EMM_ADDRESS, command, sizeof(command));
    if (command_length == 0U) {
        g_stats.rejected++;
        return false;
    }

    send_bytes(command, command_length);
    g_stats.sent++;
    g_stats.position_requests++;
    g_position_request_pending = true;
    g_position_request_started_ms = now_ms;
    return true;
}

bool emm_v5_take_current_position(EmmV5Position *position_out)
{
    if ((position_out == NULL) || !g_position_ready) {
        return false;
    }

    *position_out = g_latest_position;
    g_position_ready = false;
    return true;
}

bool emm_v5_position_request_pending(void)
{
    return g_position_request_pending;
}

const EmmV5Stats *emm_v5_stats(void)
{
    return &g_stats;
}
