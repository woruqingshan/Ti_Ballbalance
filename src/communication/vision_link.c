#include "vision_link.h"
#include "vision_uart.h"
#include "protocol/protocol.h"
#include <string.h>

static uint32_t g_tx_sequence;

void vision_link_init(VisionLink *v)
{
    memset(v, 0, sizeof(*v));
    protocol_parser_init(&v->parser);
    g_tx_sequence = 0U;
}

static void handle_ball_state(VisionLink *v,
                              const ProtocolFrame *f,
                              uint32_t now_ms)
{
    const uint8_t *p = f->payload;
    uint8_t flags;

    if (f->payload_len != 16U) {
        return;
    }
    if (v->packets_ok && f->sequence != v->last_sequence + 1U) {
        v->sequence_gaps++;
    }
    v->last_sequence = f->sequence;
    v->packets_ok++;
    v->latest.frame_id = protocol_get_u32_le(&p[0]);
    v->latest.position_cm = (float) protocol_get_i16_le(&p[4]) * 0.01f;
    v->latest.velocity_cm_s = (float) protocol_get_i16_le(&p[6]) * 0.01f;
    v->latest.confidence = (float) protocol_get_u16_le(&p[8]) / 1000.0f;
    v->latest.vision_latency_ms = protocol_get_u16_le(&p[10]);
    flags = p[12];
    v->latest.found = (flags & 0x01U) != 0U;
    v->latest.predicted = (flags & 0x02U) != 0U;
    v->latest.valid = (flags & 0x04U) != 0U;
    v->latest.invalid_reason = p[13];
    v->latest.source_timestamp_ms = f->timestamp_ms;
    v->latest.received_timestamp_ms = now_ms;
    v->has_state = true;
}

void vision_link_poll(VisionLink *v, uint32_t now_ms)
{
    uint8_t b;
    ProtocolFrame f;

    while (vision_uart_read(&b)) {
        if (!protocol_parser_push(&v->parser, b, &f)) {
            continue;
        }

        if (v->has_rx_sequence && f.sequence != v->rx_last_sequence + 1U) {
            v->rx_sequence_gaps++;
        }
        v->rx_last_sequence = f.sequence;
        v->has_rx_sequence = true;
        v->rx_frames++;
        v->last_rx_timestamp_ms = now_ms;

        if (f.type == MSG_BALL_STATE) {
            handle_ball_state(v, &f, now_ms);
        } else if (f.type == MSG_CONTROL_COMMAND && f.payload_len == 8U) {
            v->command = f.payload[0];
            v->command_argument = f.payload[1];
            v->command_value = protocol_get_i16_le(&f.payload[2]);
            v->command_token = protocol_get_u32_le(&f.payload[4]);
            v->has_command = true;
        } else if (f.type == MSG_PI_STATUS) {
            v->pi_status_packets++;
        }
    }
}

bool vision_link_get_latest(const VisionLink *v, BallState *out)
{
    if (!v->has_state) {
        return false;
    }
    *out = v->latest;
    return true;
}

bool vision_link_take_command_ex(VisionLink *v,
                                 uint8_t *command,
                                 uint8_t *argument,
                                 int16_t *value,
                                 uint32_t *token)
{
    if (!v->has_command) {
        return false;
    }
    *command = v->command;
    *argument = v->command_argument;
    *value = v->command_value;
    *token = v->command_token;
    v->has_command = false;
    return true;
}

bool vision_link_take_command(VisionLink *v,
                              uint8_t *command,
                              uint8_t *argument,
                              uint32_t *token)
{
    int16_t ignored_value;
    return vision_link_take_command_ex(v, command, argument,
                                       &ignored_value, token);
}

void vision_link_send_frame(uint8_t type,
                            const uint8_t *payload,
                            uint16_t len,
                            uint32_t now_ms)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    size_t n = protocol_encode(type, g_tx_sequence++, now_ms,
                               payload, len, frame, sizeof(frame));
    if (n) {
        vision_uart_write(frame, n);
    }
}
