#ifndef BALL_OBSERVATION_RX_DIAG_H
#define BALL_OBSERVATION_RX_DIAG_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/ball_observation_protocol.h"
#include "communication/vision_uart.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if APP_BALL_OBS_FRAME_SIZE != BALL_OBSERVATION_FRAME_SIZE
#error APP_BALL_OBS_FRAME_SIZE must match BALL_OBSERVATION_FRAME_SIZE
#endif

typedef struct {
    uint32_t rx_bytes;
    uint32_t completed_frames;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t flag_errors;
    uint32_t confidence_errors;
    uint32_t sequence_gaps;
    uint32_t duplicate_frames;
    uint32_t ring_overflows;

    uint32_t valid_frames;
    uint32_t invalid_frames;
    uint32_t found_frames;
    uint32_t predicted_frames;
    uint32_t emergency_frames;

    uint32_t latest_version;
    uint32_t last_protocol_frame_rx_ms;
    uint32_t latest_received_ms;

    int16_t latest_position_centi_cm;
    int16_t latest_velocity_centi_cm_s;
    uint16_t latest_confidence_milli;
    uint16_t latest_vision_age_ms;
    uint8_t latest_sequence;
    uint8_t latest_flags;
    uint8_t has_sequence;
    uint8_t has_observation;
} BallObservationRxStats;

/* Inspect this symbol in Keil Watch. T1 deliberately sends no UART0 TX data. */
volatile BallObservationRxStats g_ball_observation_rx_stats;

static void ball_observation_rx_diag_init_stats(void)
{
    memset((void *) &g_ball_observation_rx_stats,
           0,
           sizeof(g_ball_observation_rx_stats));
}

static void ball_observation_rx_diag_accept(
    const BallObservationPacket *packet,
    uint32_t now_ms)
{
    uint8_t delta;

    g_ball_observation_rx_stats.frames_ok++;
    g_ball_observation_rx_stats.last_protocol_frame_rx_ms = now_ms;

    if (ball_observation_packet_valid(packet)) {
        g_ball_observation_rx_stats.valid_frames++;
    } else {
        g_ball_observation_rx_stats.invalid_frames++;
    }
    if (ball_observation_packet_found(packet)) {
        g_ball_observation_rx_stats.found_frames++;
    }
    if (ball_observation_packet_predicted(packet)) {
        g_ball_observation_rx_stats.predicted_frames++;
    }
    if (ball_observation_packet_emergency(packet)) {
        g_ball_observation_rx_stats.emergency_frames++;
    }

    if (g_ball_observation_rx_stats.has_sequence != 0U) {
        delta = (uint8_t) (packet->sequence -
                           g_ball_observation_rx_stats.latest_sequence);
        if (delta == 0U) {
            /*
             * A duplicate proves that a protocol frame arrived, but it must not
             * refresh the latest observation version or observation age.
             */
            g_ball_observation_rx_stats.duplicate_frames++;
            return;
        }
        if (delta > 1U) {
            g_ball_observation_rx_stats.sequence_gaps +=
                (uint32_t) delta - 1U;
        }
    }

    g_ball_observation_rx_stats.has_sequence = 1U;
    g_ball_observation_rx_stats.has_observation = 1U;
    g_ball_observation_rx_stats.latest_sequence = packet->sequence;
    g_ball_observation_rx_stats.latest_flags = packet->flags;
    g_ball_observation_rx_stats.latest_position_centi_cm =
        packet->position_centi_cm;
    g_ball_observation_rx_stats.latest_velocity_centi_cm_s =
        packet->velocity_centi_cm_s;
    g_ball_observation_rx_stats.latest_confidence_milli =
        packet->confidence_milli;
    g_ball_observation_rx_stats.latest_vision_age_ms =
        packet->vision_age_ms;
    g_ball_observation_rx_stats.latest_received_ms = now_ms;
    g_ball_observation_rx_stats.latest_version++;
}

static void ball_observation_rx_diag_note_result(
    BallObservationParseResult result,
    const BallObservationPacket *packet,
    uint32_t now_ms)
{
    if (result == BALL_OBSERVATION_PARSE_INCOMPLETE) {
        return;
    }

    g_ball_observation_rx_stats.completed_frames++;
    switch (result) {
    case BALL_OBSERVATION_PARSE_ACCEPTED:
        ball_observation_rx_diag_accept(packet, now_ms);
        break;
    case BALL_OBSERVATION_PARSE_BAD_CRC:
        g_ball_observation_rx_stats.crc_errors++;
        break;
    case BALL_OBSERVATION_PARSE_BAD_FLAGS:
        g_ball_observation_rx_stats.flag_errors++;
        break;
    case BALL_OBSERVATION_PARSE_BAD_CONFIDENCE:
        g_ball_observation_rx_stats.confidence_errors++;
        break;
    default:
        break;
    }
}

static void ball_observation_rx_diag_run(void)
{
    BallObservationParser parser;
    BallObservationPacket packet;

    ball_observation_parser_init(&parser);
    ball_observation_rx_diag_init_stats();

    for (;;) {
        uint32_t processed = 0U;
        uint8_t byte;

        while ((processed < APP_BALL_OBS_RX_BUDGET_BYTES) &&
               vision_uart_read(&byte)) {
            BallObservationParseResult result;
            uint32_t now_ms = bsp_time_ms();

            g_ball_observation_rx_stats.rx_bytes++;
            result = ball_observation_parser_push(&parser, byte, &packet);
            ball_observation_rx_diag_note_result(result, &packet, now_ms);
            processed++;
        }

        g_ball_observation_rx_stats.ring_overflows =
            vision_uart_rx_overflow();

        /* Keep draining a finite burst even when the sender stops immediately. */
        if (processed < APP_BALL_OBS_RX_BUDGET_BYTES) {
            __WFI();
        }
    }
}

#endif
