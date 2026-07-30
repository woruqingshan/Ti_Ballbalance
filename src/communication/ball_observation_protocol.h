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
#define BALL_OBSERVATION_STATUS_MASK             0x0FU
#define BALL_OBSERVATION_REASON_MASK             0xF0U
#define BALL_OBSERVATION_REASON_SHIFT               4U

#define BALL_OBSERVATION_CONFIDENCE_MAX          1000U

typedef enum {
    BALL_OBSERVATION_INVALID_NONE = 0,
    BALL_OBSERVATION_INVALID_NOT_FOUND = 1,
    BALL_OBSERVATION_INVALID_LOW_CONFIDENCE = 2,
    BALL_OBSERVATION_INVALID_OUT_OF_ROI = 3,
    BALL_OBSERVATION_INVALID_POSITION_JUMP = 4,
    BALL_OBSERVATION_INVALID_STALE = 5,
    BALL_OBSERVATION_INVALID_OUT_OF_RANGE = 6,
    BALL_OBSERVATION_INVALID_SOURCE_ERROR = 7,
    BALL_OBSERVATION_INVALID_PIPE_AXIS = 8,
    BALL_OBSERVATION_INVALID_VELOCITY = 9,
    BALL_OBSERVATION_INVALID_REACQUIRING = 10,
    BALL_OBSERVATION_INVALID_SHUTDOWN = 11
} BallObservationInvalidReason;

typedef enum {
    BALL_OBSERVATION_PARSE_INCOMPLETE = 0,
    BALL_OBSERVATION_PARSE_ACCEPTED,
    BALL_OBSERVATION_PARSE_BAD_CRC,
    /* Kept for source compatibility; Fixed-14 v1 defines all flag bits. */
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
    uint16_t confidence;

    received_crc = ball_observation_read_u16_le(&frame[12]);
    calculated_crc = crc16_ccitt_false(
        &frame[BALL_OBSERVATION_BODY_OFFSET],
        BALL_OBSERVATION_BODY_SIZE);
    if (received_crc != calculated_crc) {
        return BALL_OBSERVATION_PARSE_BAD_CRC;
    }

    /*
     * Fixed-14 v1 uses bits 0..3 as status and bits 4..7 as invalid_reason.
     * Therefore 0x14 and 0xB8 are valid protocol values and must not be
     * rejected as unknown flags.
     */
    confidence = ball_observation_read_u16_le(&frame[8]);
    if (confidence > BALL_OBSERVATION_CONFIDENCE_MAX) {
        return BALL_OBSERVATION_PARSE_BAD_CONFIDENCE;
    }

    if (packet != 0) {
        packet->sequence = frame[2];
        packet->flags = frame[3];
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

static uint8_t ball_observation_packet_invalid_reason(
    const BallObservationPacket *packet)
{
    if (packet == 0) {
        return (uint8_t) BALL_OBSERVATION_INVALID_NONE;
    }
    return (uint8_t) ((packet->flags & BALL_OBSERVATION_REASON_MASK) >>
                      BALL_OBSERVATION_REASON_SHIFT);
}

#endif
