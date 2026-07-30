#include "communication/ball_observation_protocol.h"
#include "protocol/crc16.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t) (value & 0xFFU);
    dst[1] = (uint8_t) (value >> 8U);
}

static void build_frame(uint8_t frame[BALL_OBSERVATION_FRAME_SIZE],
                        uint8_t sequence,
                        uint8_t flags,
                        int16_t position,
                        int16_t velocity,
                        uint16_t confidence,
                        uint16_t age_ms)
{
    uint16_t crc;

    memset(frame, 0, BALL_OBSERVATION_FRAME_SIZE);
    frame[0] = BALL_OBSERVATION_SOF_0;
    frame[1] = BALL_OBSERVATION_SOF_1;
    frame[2] = sequence;
    frame[3] = flags;
    put_u16_le(&frame[4], (uint16_t) position);
    put_u16_le(&frame[6], (uint16_t) velocity);
    put_u16_le(&frame[8], confidence);
    put_u16_le(&frame[10], age_ms);
    crc = crc16_ccitt_false(&frame[2], BALL_OBSERVATION_BODY_SIZE);
    put_u16_le(&frame[12], crc);
}

static BallObservationParseResult feed_frame(
    BallObservationParser *parser,
    const uint8_t *data,
    size_t length,
    BallObservationPacket *packet)
{
    BallObservationParseResult result = BALL_OBSERVATION_PARSE_INCOMPLETE;
    size_t i;

    for (i = 0; i < length; ++i) {
        BallObservationParseResult current =
            ball_observation_parser_push(parser, data[i], packet);
        if (current != BALL_OBSERVATION_PARSE_INCOMPLETE) {
            result = current;
        }
    }
    return result;
}

static void test_nominal_signed_fields(void)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservationPacket packet;
    BallObservationParseResult result;

    build_frame(frame,
                42U,
                BALL_OBSERVATION_FLAG_FOUND |
                    BALL_OBSERVATION_FLAG_VALID,
                325,
                -840,
                950U,
                27U);

    ball_observation_parser_init(&parser);
    result = feed_frame(&parser, frame, sizeof(frame), &packet);

    assert(result == BALL_OBSERVATION_PARSE_ACCEPTED);
    assert(packet.sequence == 42U);
    assert(packet.position_centi_cm == 325);
    assert(packet.velocity_centi_cm_s == -840);
    assert(packet.confidence_milli == 950U);
    assert(packet.vision_age_ms == 27U);
    assert(ball_observation_packet_found(&packet));
    assert(ball_observation_packet_valid(&packet));
    assert(!ball_observation_packet_predicted(&packet));
    assert(!ball_observation_packet_emergency(&packet));
}

static void test_split_and_sticky_frames(void)
{
    uint8_t first[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t second[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t stream[BALL_OBSERVATION_FRAME_SIZE * 2U];
    BallObservationParser parser;
    BallObservationPacket packet;
    size_t i;
    unsigned accepted = 0U;

    build_frame(first, 254U, BALL_OBSERVATION_FLAG_VALID, -500, 120, 800U, 10U);
    build_frame(second, 255U, BALL_OBSERVATION_FLAG_VALID, 500, -120, 810U, 11U);
    memcpy(stream, first, sizeof(first));
    memcpy(stream + sizeof(first), second, sizeof(second));

    ball_observation_parser_init(&parser);
    for (i = 0; i < sizeof(stream); ++i) {
        BallObservationParseResult result =
            ball_observation_parser_push(&parser, stream[i], &packet);
        if (result == BALL_OBSERVATION_PARSE_ACCEPTED) {
            accepted++;
        }
    }

    assert(accepted == 2U);
    assert(packet.sequence == 255U);
    assert(packet.position_centi_cm == 500);
    assert(packet.velocity_centi_cm_s == -120);
}

static void test_crc_flags_and_confidence_errors(void)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservationPacket packet;

    build_frame(frame, 1U, BALL_OBSERVATION_FLAG_VALID, 0, 0, 900U, 20U);
    frame[5] ^= 0x01U;
    ball_observation_parser_init(&parser);
    assert(feed_frame(&parser, frame, sizeof(frame), &packet) ==
           BALL_OBSERVATION_PARSE_BAD_CRC);

    build_frame(frame, 2U, 0x80U, 0, 0, 900U, 20U);
    ball_observation_parser_init(&parser);
    assert(feed_frame(&parser, frame, sizeof(frame), &packet) ==
           BALL_OBSERVATION_PARSE_BAD_FLAGS);

    build_frame(frame, 3U, BALL_OBSERVATION_FLAG_VALID, 0, 0, 1001U, 20U);
    ball_observation_parser_init(&parser);
    assert(feed_frame(&parser, frame, sizeof(frame), &packet) ==
           BALL_OBSERVATION_PARSE_BAD_CONFIDENCE);
}

static void test_noise_and_resynchronization(void)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    uint8_t stream[3U + BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservationPacket packet;
    BallObservationParseResult result;

    build_frame(frame,
                9U,
                BALL_OBSERVATION_FLAG_FOUND |
                    BALL_OBSERVATION_FLAG_VALID |
                    BALL_OBSERVATION_FLAG_PREDICTED,
                -1300,
                327,
                350U,
                120U);

    stream[0] = 0x00U;
    stream[1] = BALL_OBSERVATION_SOF_0;
    stream[2] = 0x11U;
    memcpy(&stream[3], frame, sizeof(frame));

    ball_observation_parser_init(&parser);
    result = feed_frame(&parser, stream, sizeof(stream), &packet);
    assert(result == BALL_OBSERVATION_PARSE_ACCEPTED);
    assert(packet.sequence == 9U);
    assert(packet.position_centi_cm == -1300);
    assert(packet.velocity_centi_cm_s == 327);
    assert(ball_observation_packet_predicted(&packet));
}

int main(void)
{
    test_nominal_signed_fields();
    test_split_and_sticky_frames();
    test_crc_flags_and_confidence_errors();
    test_noise_and_resynchronization();

    puts("BallObservation protocol host tests: PASS");
    return 0;
}
