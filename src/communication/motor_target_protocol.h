#ifndef MOTOR_TARGET_PROTOCOL_H
#define MOTOR_TARGET_PROTOCOL_H

#include "protocol/crc16.h"
#include <stdbool.h>
#include <stdint.h>

#define MOTOR_TARGET_FRAME_SIZE        8U
#define MOTOR_TARGET_SOF_0             0xA5U
#define MOTOR_TARGET_SOF_1             0x5AU
#define MOTOR_TARGET_FLAG_VALID        0x01U
#define MOTOR_TARGET_FLAG_DISABLE      0x02U
#define MOTOR_TARGET_KNOWN_FLAGS       \
    (MOTOR_TARGET_FLAG_VALID | MOTOR_TARGET_FLAG_DISABLE)

typedef enum {
    MOTOR_TARGET_PARSE_INCOMPLETE = 0,
    MOTOR_TARGET_PARSE_ACCEPTED,
    MOTOR_TARGET_PARSE_BAD_CRC,
    MOTOR_TARGET_PARSE_BAD_FLAGS,
    MOTOR_TARGET_PARSE_BAD_RANGE
} MotorTargetParseResult;

typedef struct {
    uint8_t frame[MOTOR_TARGET_FRAME_SIZE];
    uint8_t index;
} MotorTargetParser;

typedef struct {
    uint8_t sequence;
    uint8_t flags;
    int32_t target_offset_mdeg;
} MotorTargetPacket;

static void motor_target_parser_init(MotorTargetParser *parser)
{
    if (parser != 0) {
        parser->index = 0U;
    }
}

static void motor_target_parser_resync(MotorTargetParser *parser,
                                       uint8_t byte)
{
    if (byte == MOTOR_TARGET_SOF_0) {
        parser->frame[0] = byte;
        parser->index = 1U;
    } else {
        parser->index = 0U;
    }
}

static MotorTargetParseResult motor_target_decode_frame(
    const uint8_t frame[MOTOR_TARGET_FRAME_SIZE],
    int32_t minimum_offset_mdeg,
    int32_t maximum_offset_mdeg,
    MotorTargetPacket *packet)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t raw_target;
    uint8_t flags;
    int32_t target_mdeg;

    received_crc = (uint16_t) frame[6] |
                   ((uint16_t) frame[7] << 8U);
    calculated_crc = crc16_ccitt_false(&frame[2], 4U);
    if (received_crc != calculated_crc) {
        return MOTOR_TARGET_PARSE_BAD_CRC;
    }

    flags = frame[3];
    if ((flags & (uint8_t) ~MOTOR_TARGET_KNOWN_FLAGS) != 0U) {
        return MOTOR_TARGET_PARSE_BAD_FLAGS;
    }

    raw_target = (uint16_t) frame[4] |
                 ((uint16_t) frame[5] << 8U);
    target_mdeg = (int32_t) ((int16_t) raw_target);
    if (((flags & MOTOR_TARGET_FLAG_VALID) != 0U) &&
        ((flags & MOTOR_TARGET_FLAG_DISABLE) == 0U) &&
        ((target_mdeg < minimum_offset_mdeg) ||
         (target_mdeg > maximum_offset_mdeg))) {
        return MOTOR_TARGET_PARSE_BAD_RANGE;
    }

    if (packet != 0) {
        packet->sequence = frame[2];
        packet->flags = flags;
        packet->target_offset_mdeg = target_mdeg;
    }
    return MOTOR_TARGET_PARSE_ACCEPTED;
}

static MotorTargetParseResult motor_target_parser_push(
    MotorTargetParser *parser,
    uint8_t byte,
    int32_t minimum_offset_mdeg,
    int32_t maximum_offset_mdeg,
    MotorTargetPacket *packet)
{
    MotorTargetParseResult result;

    if (parser == 0) {
        return MOTOR_TARGET_PARSE_INCOMPLETE;
    }

    if (parser->index == 0U) {
        motor_target_parser_resync(parser, byte);
        return MOTOR_TARGET_PARSE_INCOMPLETE;
    }

    if (parser->index == 1U) {
        if (byte == MOTOR_TARGET_SOF_1) {
            parser->frame[1] = byte;
            parser->index = 2U;
        } else {
            motor_target_parser_resync(parser, byte);
        }
        return MOTOR_TARGET_PARSE_INCOMPLETE;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index < MOTOR_TARGET_FRAME_SIZE) {
        return MOTOR_TARGET_PARSE_INCOMPLETE;
    }

    result = motor_target_decode_frame(
        parser->frame,
        minimum_offset_mdeg,
        maximum_offset_mdeg,
        packet);
    parser->index = 0U;
    return result;
}

#endif
