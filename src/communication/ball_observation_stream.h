#ifndef BALL_OBSERVATION_STREAM_H
#define BALL_OBSERVATION_STREAM_H

#include "communication/ball_observation_protocol.h"
#include "communication/vision_uart.h"
#include "bsp/bsp_time.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

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

typedef struct {
    BallObservationParser parser;
    BallObservationPacket latest;
    BallObservationRxStats stats;
} BallObservationStream;

static void ball_observation_stream_init(BallObservationStream *stream)
{
    if (stream == 0) {
        return;
    }
    memset(stream, 0, sizeof(*stream));
    ball_observation_parser_init(&stream->parser);
}

static void ball_observation_stream_accept(BallObservationStream *stream,
                                           const BallObservationPacket *packet,
                                           uint32_t now_ms)
{
    uint8_t delta;

    stream->stats.frames_ok++;
    stream->stats.last_protocol_frame_rx_ms = now_ms;

    if (ball_observation_packet_valid(packet)) {
        stream->stats.valid_frames++;
    } else {
        stream->stats.invalid_frames++;
    }
    if (ball_observation_packet_found(packet)) {
        stream->stats.found_frames++;
    }
    if (ball_observation_packet_predicted(packet)) {
        stream->stats.predicted_frames++;
    }
    if (ball_observation_packet_emergency(packet)) {
        stream->stats.emergency_frames++;
    }

    if (stream->stats.has_sequence != 0U) {
        delta = (uint8_t) (packet->sequence - stream->stats.latest_sequence);
        if (delta == 0U) {
            /*
             * A duplicate proves that a protocol frame arrived, but it must not
             * refresh observation version or age. The sender contract is
             * latest-only with no retransmission of historical observations.
             */
            stream->stats.duplicate_frames++;
            return;
        }
        if (delta > 1U) {
            stream->stats.sequence_gaps += (uint32_t) delta - 1U;
        }
    }

    stream->latest = *packet;
    stream->stats.has_sequence = 1U;
    stream->stats.has_observation = 1U;
    stream->stats.latest_sequence = packet->sequence;
    stream->stats.latest_flags = packet->flags;
    stream->stats.latest_position_centi_cm = packet->position_centi_cm;
    stream->stats.latest_velocity_centi_cm_s = packet->velocity_centi_cm_s;
    stream->stats.latest_confidence_milli = packet->confidence_milli;
    stream->stats.latest_vision_age_ms = packet->vision_age_ms;
    stream->stats.latest_received_ms = now_ms;
    stream->stats.latest_version++;
}

static void ball_observation_stream_note_result(
    BallObservationStream *stream,
    BallObservationParseResult result,
    const BallObservationPacket *packet,
    uint32_t now_ms)
{
    if (result == BALL_OBSERVATION_PARSE_INCOMPLETE) {
        return;
    }

    stream->stats.completed_frames++;
    switch (result) {
    case BALL_OBSERVATION_PARSE_ACCEPTED:
        ball_observation_stream_accept(stream, packet, now_ms);
        break;
    case BALL_OBSERVATION_PARSE_BAD_CRC:
        stream->stats.crc_errors++;
        break;
    case BALL_OBSERVATION_PARSE_BAD_FLAGS:
        stream->stats.flag_errors++;
        break;
    case BALL_OBSERVATION_PARSE_BAD_CONFIDENCE:
        stream->stats.confidence_errors++;
        break;
    default:
        break;
    }
}

static uint32_t ball_observation_stream_poll(BallObservationStream *stream,
                                             uint32_t byte_budget)
{
    BallObservationPacket packet;
    uint32_t processed = 0U;
    uint8_t byte;

    if (stream == 0) {
        return 0U;
    }

    while ((processed < byte_budget) && vision_uart_read(&byte)) {
        BallObservationParseResult result;
        uint32_t now_ms = bsp_time_ms();

        stream->stats.rx_bytes++;
        result = ball_observation_parser_push(&stream->parser, byte, &packet);
        ball_observation_stream_note_result(stream, result, &packet, now_ms);
        processed++;
    }

    stream->stats.ring_overflows = vision_uart_rx_overflow();
    return processed;
}

static const BallObservationPacket *ball_observation_stream_latest(
    const BallObservationStream *stream)
{
    if ((stream == 0) || (stream->stats.has_observation == 0U)) {
        return 0;
    }
    return &stream->latest;
}

#endif
