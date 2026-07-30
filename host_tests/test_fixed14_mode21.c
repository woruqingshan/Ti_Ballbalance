#include "communication/ball_observation_protocol.h"
#include "control/ball_observation_gate.h"
#include "control/ball_position_controller.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const uint8_t GOLDEN_VALID[BALL_OBSERVATION_FRAME_SIZE] = {
    0xA5, 0x5A, 0x2A, 0x03, 0xF4, 0x01, 0x06,
    0xFF, 0x84, 0x03, 0x2D, 0x00, 0x5D, 0x72
};
static const uint8_t GOLDEN_INVALID_PREDICTED[BALL_OBSERVATION_FRAME_SIZE] = {
    0xA5, 0x5A, 0xFF, 0x14, 0x0C, 0xFE, 0xD2,
    0x04, 0x00, 0x00, 0x78, 0x00, 0x89, 0x68
};
static const uint8_t GOLDEN_EMERGENCY[BALL_OBSERVATION_FRAME_SIZE] = {
    0xA5, 0x5A, 0x00, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0xFF, 0xFF, 0x79, 0x9E
};

static BallObservationParseResult feed(
    const uint8_t frame[BALL_OBSERVATION_FRAME_SIZE],
    BallObservationPacket *packet)
{
    BallObservationParser parser;
    BallObservationParseResult result = BALL_OBSERVATION_PARSE_INCOMPLETE;
    size_t i;

    ball_observation_parser_init(&parser);
    for (i = 0; i < BALL_OBSERVATION_FRAME_SIZE; ++i) {
        result = ball_observation_parser_push(&parser, frame[i], packet);
    }
    return result;
}

static void test_golden_vectors(void)
{
    BallObservationPacket packet;

    assert(feed(GOLDEN_VALID, &packet) == BALL_OBSERVATION_PARSE_ACCEPTED);
    assert(packet.sequence == 42U);
    assert(packet.flags == 0x03U);
    assert(packet.position_centi_cm == 500);
    assert(packet.velocity_centi_cm_s == -250);
    assert(packet.confidence_milli == 900U);
    assert(packet.vision_age_ms == 45U);
    assert(ball_observation_packet_found(&packet));
    assert(ball_observation_packet_valid(&packet));
    assert(!ball_observation_packet_predicted(&packet));
    assert(!ball_observation_packet_emergency(&packet));
    assert(ball_observation_packet_invalid_reason(&packet) == 0U);

    assert(feed(GOLDEN_INVALID_PREDICTED, &packet) ==
           BALL_OBSERVATION_PARSE_ACCEPTED);
    assert(packet.sequence == 255U);
    assert(packet.flags == 0x14U);
    assert(packet.position_centi_cm == -500);
    assert(packet.velocity_centi_cm_s == 1234);
    assert(!ball_observation_packet_valid(&packet));
    assert(ball_observation_packet_predicted(&packet));
    assert(ball_observation_packet_invalid_reason(&packet) == 1U);

    assert(feed(GOLDEN_EMERGENCY, &packet) ==
           BALL_OBSERVATION_PARSE_ACCEPTED);
    assert(packet.flags == 0xB8U);
    assert(packet.vision_age_ms == 65535U);
    assert(ball_observation_packet_emergency(&packet));
    assert(ball_observation_packet_invalid_reason(&packet) == 11U);
}

static BallObservationGateConfig gate_config(void)
{
    BallObservationGateConfig config;
    config.minimum_position_centi_cm = -1300;
    config.maximum_position_centi_cm = 1300;
    config.maximum_abs_velocity_centi_cm_s = 5000U;
    config.minimum_confidence_milli = 350U;
    config.maximum_total_age_ms = 120U;
    config.allow_predicted = false;
    return config;
}

static void test_gate_source_reason(void)
{
    BallObservationPacket packet;
    BallObservationGateConfig config = gate_config();
    BallObservationGateResult gate;

    assert(feed(GOLDEN_INVALID_PREDICTED, &packet) ==
           BALL_OBSERVATION_PARSE_ACCEPTED);
    gate = ball_observation_gate_evaluate(&packet, true, 1000U, 1010U, &config);
    assert(!gate.control_valid);
    assert(gate.reason == BALL_OBS_GATE_FLAG_INVALID);
    assert(gate.source_invalid_reason == 1U);

    assert(feed(GOLDEN_EMERGENCY, &packet) ==
           BALL_OBSERVATION_PARSE_ACCEPTED);
    gate = ball_observation_gate_evaluate(&packet, true, 1000U, 1000U, &config);
    assert(!gate.control_valid);
    assert(gate.reason == BALL_OBS_GATE_EMERGENCY);
    assert(gate.source_invalid_reason == 11U);

    assert(feed(GOLDEN_VALID, &packet) == BALL_OBSERVATION_PARSE_ACCEPTED);
    gate = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(gate.control_valid);
    assert(gate.total_age_ms == 85U);
    assert(gate.source_invalid_reason == 0U);

    gate = ball_observation_gate_evaluate(&packet, true, 1000U, 1080U, &config);
    assert(!gate.control_valid);
    assert(gate.reason == BALL_OBS_GATE_STALE);
}

static void test_controller_sign_and_limits(void)
{
    BallPositionController controller;
    BallPositionControllerConfig config;
    int32_t output;

    config.target_position_centi_cm = 0;
    config.position_kp_mdeg_per_cm = 60;
    config.velocity_kd_mdeg_per_cm_s = 15;
    config.maximum_offset_mdeg = 200;
    config.maximum_slew_mdeg_per_s = 800;
    config.motor_sign = 1;

    ball_position_controller_init(&controller);
    output = ball_position_controller_update(
        &controller, &config, 500, 0, 1000U, 25U);
    assert(controller.error_centi_cm == -500);
    assert(controller.raw_output_mdeg == -300);
    assert(controller.amplitude_limited_mdeg == -200);
    assert(output == -20);

    output = ball_position_controller_update(
        &controller, &config, -500, 0, 1025U, 25U);
    assert(controller.error_centi_cm == 500);
    assert(controller.raw_output_mdeg == 300);
    assert(controller.amplitude_limited_mdeg == 200);
    assert(output == 0);

    ball_position_controller_reset(&controller);
    assert(controller.target_offset_mdeg == 0);
}

static void test_crc_rejection(void)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationPacket packet;
    memcpy(frame, GOLDEN_VALID, sizeof(frame));
    frame[4] ^= 0x01U;
    assert(feed(frame, &packet) == BALL_OBSERVATION_PARSE_BAD_CRC);
}

int main(void)
{
    test_golden_vectors();
    test_gate_source_reason();
    test_controller_sign_and_limits();
    test_crc_rejection();
    puts("Fixed-14 compatibility and Mode21 core host tests: PASS");
    return 0;
}
