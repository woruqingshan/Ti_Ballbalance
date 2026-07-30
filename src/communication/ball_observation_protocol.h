#ifndef BALL_OBSERVATION_PROTOCOL_H
#define BALL_OBSERVATION_PROTOCOL_H

#include "protocol/crc16.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BALL_OBSERVATION_FRAME_SIZE              14U
#define BALL_OBSERVATION_BODY_OFFSET               2U
#define BALL_OBSERVATION_BODY_SIZE                10U
#define BALL_OBSERVATION_SOF_0                  0xA5U
#define BALL_OBSERVATION_SOF_1                  0x5AU

#define BALL_OBSERVATION_FLAG_FOUND              0x01U
#define BALL_OBSERVATION_FLAG_VALID              0x02U
#define BALL_OBSERVATION_FLAG_PREDICTED          0x04U
#define BALL_OBSERVATION_FLAG_EMERGENCY          0x08U
#define BALL_OBSERVATION_KNOWN_FLAGS             \
    (BALL_OBSERVATION_FLAG_FOUND |               \
     BALL_OBSERVATION_FLAG_VALID |               \
     BALL_OBSERVATION_FLAG_PREDICTED |           \
     BALL_OBSERVATION_FLAG_EMERGENCY)

#define BALL_OBSERVATION_CONFIDENCE_MAX          1000U

typedef enum {
    BALL_OBSERVATION_PARSE_INCOMPLETE = 0,
    BALL_OBSERVATION_PARSE_ACCEPTED,
    BALL_OBSERVATION_PARSE_BAD_CRC,
    BALL_OBSERVATION_PARSE_BAD_FLAGS,
    BALL_OBSERVATION_PARSE_BAD_CONFIDENCE
} BallObservationParseResult;

typedef struct {
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t index;
} BallObservationParser;

typedef struct {
    uint8_t sequence;
    uint8_t flags;
    int16_t position_centi_cm;
    int16_t velocity_centi_cm_s;
    uint16_t confidence_milli;
    uint16_t vision_age_ms;
} BallObservationPacket;

static uint16_t ball_observation_read_u16_le(const uint8_t *data)
{
    return (uint16_t) data[0] | ((uint16_t) data[1] << 8U);
}

static int16_t ball_observation_read_i16_le(const uint8_t *data)
{
    return (int16_t) ball_observation_read_u16_le(data);
}

static void ball_observation_parser_init(BallObservationParser *parser)
{
    if (parser != 0) {
        parser->index = 0U;
    }
}

static void ball_observation_parser_wait_for_sof(
    BallObservationParser *parser,
    uint8_t byte)
{
    if (byte == BALL_OBSERVATION_SOF_0) {
        parser->frame[0] = byte;
        parser->index = 1U;
    } else {
        parser->index = 0U;
    }
}

/*
 * Retain a possible SOF suffix after a malformed full frame. This allows the
 * parser to recover immediately when a dropped/noisy byte shifted the next
 * frame header into the current 14-byte window.
 */
static void ball_observation_parser_resync_full_frame(
    BallObservationParser *parser)
{
    uint8_t start;

    for (start = 1U; start < BALL_OBSERVATION_FRAME_SIZE; ++start) {
        if (parser->frame[start] != BALL_OBSERVATION_SOF_0) {
            continue;
        }

        if (start == (BALL_OBSERVATION_FRAME_SIZE - 1U)) {
            parser->frame[0] = BALL_OBSERVATION_SOF_0;
            parser->index = 1U;
            return;
        }

        if (parser->frame[start + 1U] == BALL_OBSERVATION_SOF_1) {
            uint8_t retained = (uint8_t) (BALL_OBSERVATION_FRAME_SIZE - start);
            memmove(parser->frame, &parser->frame[start], retained);
            parser->index = retained;
            return;
        }
    }

    parser->index = 0U;
}

static BallObservationParseResult ball_observation_decode_frame(
    const uint8_t frame[BALL_OBSERVATION_FRAME_SIZE],
    BallObservationPacket *packet)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint8_t flags;
    uint16_t confidence;

    received_crc = ball_observation_read_u16_le(&frame[12]);
    calculated_crc = crc16_ccitt_false(
        &frame[BALL_OBSERVATION_BODY_OFFSET],
        BALL_OBSERVATION_BODY_SIZE);
    if (received_crc != calculated_crc) {
        return BALL_OBSERVATION_PARSE_BAD_CRC;
    }

    flags = frame[3];
    if ((flags & (uint8_t) ~BALL_OBSERVATION_KNOWN_FLAGS) != 0U) {
        return BALL_OBSERVATION_PARSE_BAD_FLAGS;
    }

    confidence = ball_observation_read_u16_le(&frame[8]);
    if (confidence > BALL_OBSERVATION_CONFIDENCE_MAX) {
        return BALL_OBSERVATION_PARSE_BAD_CONFIDENCE;
    }

    if (packet != 0) {
        packet->sequence = frame[2];
        packet->flags = flags;
        packet->position_centi_cm = ball_observation_read_i16_le(&frame[4]);
        packet->velocity_centi_cm_s = ball_observation_read_i16_le(&frame[6]);
        packet->confidence_milli = confidence;
        packet->vision_age_ms = ball_observation_read_u16_le(&frame[10]);
    }

    return BALL_OBSERVATION_PARSE_ACCEPTED;
}

static BallObservationParseResult ball_observation_parser_push(
    BallObservationParser *parser,
    uint8_t byte,
    BallObservationPacket *packet)
{
    BallObservationParseResult result;

    if (parser == 0) {
        return BALL_OBSERVATION_PARSE_INCOMPLETE;
    }

    if (parser->index == 0U) {
        ball_observation_parser_wait_for_sof(parser, byte);
        return BALL_OBSERVATION_PARSE_INCOMPLETE;
    }

    if (parser->index == 1U) {
        if (byte == BALL_OBSERVATION_SOF_1) {
            parser->frame[1] = byte;
            parser->index = 2U;
        } else {
            ball_observation_parser_wait_for_sof(parser, byte);
        }
        return BALL_OBSERVATION_PARSE_INCOMPLETE;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index < BALL_OBSERVATION_FRAME_SIZE) {
        return BALL_OBSERVATION_PARSE_INCOMPLETE;
    }

    result = ball_observation_decode_frame(parser->frame, packet);
    if (result == BALL_OBSERVATION_PARSE_ACCEPTED) {
        parser->index = 0U;
    } else {
        ball_observation_parser_resync_full_frame(parser);
    }
    return result;
}

static bool ball_observation_packet_found(const BallObservationPacket *packet)
{
    return packet != 0 &&
           (packet->flags & BALL_OBSERVATION_FLAG_FOUND) != 0U;
}

static bool ball_observation_packet_valid(const BallObservationPacket *packet)
{
    return packet != 0 &&
           (packet->flags & BALL_OBSERVATION_FLAG_VALID) != 0U;
}

static bool ball_observation_packet_predicted(const BallObservationPacket *packet)
{
    return packet != 0 &&
           (packet->flags & BALL_OBSERVATION_FLAG_PREDICTED) != 0U;
}

static bool ball_observation_packet_emergency(const BallObservationPacket *packet)
{
    return packet != 0 &&
           (packet->flags & BALL_OBSERVATION_FLAG_EMERGENCY) != 0U;
}

#endif
