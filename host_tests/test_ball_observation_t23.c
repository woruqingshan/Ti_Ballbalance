#include "communication/ball_observation_protocol.h"
#include "control/ball_observation_gate.h"
#include "control/ball_position_controller.h"
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

static BallObservationPacket decode(uint8_t sequence,
                                    uint8_t flags,
                                    int16_t position,
                                    int16_t velocity,
                                    uint16_t confidence,
                                    uint16_t age_ms)
{
    uint8_t frame[BALL_OBSERVATION_FRAME_SIZE];
    BallObservationParser parser;
    BallObservationPacket packet;
    BallObservationParseResult result = BALL_OBSERVATION_PARSE_INCOMPLETE;
    size_t i;

    build_frame(frame, sequence, flags, position, velocity, confidence, age_ms);
    ball_observation_parser_init(&parser);
    for (i = 0; i < sizeof(frame); ++i) {
        result = ball_observation_parser_push(&parser, frame[i], &packet);
    }
    assert(result == BALL_OBSERVATION_PARSE_ACCEPTED);
    return packet;
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

static void test_gate(void)
{
    BallObservationGateConfig config = gate_config();
    BallObservationPacket packet = decode(
        1U,
        BALL_OBSERVATION_FLAG_FOUND | BALL_OBSERVATION_FLAG_VALID,
        500,
        120,
        950U,
        25U);
    BallObservationGateResult result;

    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(result.control_valid);
    assert(result.reason == BALL_OBS_GATE_VALID);
    assert(result.total_age_ms == 65U);

    packet.flags |= BALL_OBSERVATION_FLAG_PREDICTED;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(!result.control_valid);
    assert(result.reason == BALL_OBS_GATE_PREDICTED_DISALLOWED);

    packet.flags = BALL_OBSERVATION_FLAG_FOUND | BALL_OBSERVATION_FLAG_VALID;
    packet.confidence_milli = 349U;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(result.reason == BALL_OBS_GATE_LOW_CONFIDENCE);

    packet.confidence_milli = 950U;
    packet.position_centi_cm = 1301;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(result.reason == BALL_OBS_GATE_POSITION_RANGE);

    packet.position_centi_cm = 0;
    packet.velocity_centi_cm_s = -5001;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1040U, &config);
    assert(result.reason == BALL_OBS_GATE_VELOCITY_RANGE);

    packet.velocity_centi_cm_s = 0;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1100U, &config);
    assert(result.reason == BALL_OBS_GATE_STALE);

    packet.flags = BALL_OBSERVATION_FLAG_EMERGENCY;
    result = ball_observation_gate_evaluate(&packet, true, 1000U, 1000U, &config);
    assert(result.reason == BALL_OBS_GATE_EMERGENCY);
}

static BallPositionControllerConfig controller_config(void)
{
    BallPositionControllerConfig config;
    config.target_position_centi_cm = 0;
    config.position_kp_mdeg_per_cm = 60;
    config.velocity_kd_mdeg_per_cm_s = 15;
    config.maximum_offset_mdeg = 200;
    config.maximum_slew_mdeg_per_s = 800;
    config.motor_sign = 1;
    return config;
}

static void test_controller(void)
{
    BallPositionController controller;
    BallPositionControllerConfig config = controller_config();
    int32_t output;

    ball_position_controller_init(&controller);
    output = ball_position_controller_update(
        &controller, &config, 500, 0, 1000U, 25U);
    assert(controller.error_centi_cm == -500);
    assert(controller.raw_output_mdeg == -300);
    assert(controller.amplitude_limited_mdeg == -200);
    assert(output == -20);

    output = ball_position_controller_update(
        &controller, &config, 500, 0, 1025U, 25U);
    assert(output == -40);

    ball_position_controller_reset(&controller);
    output = ball_position_controller_update(
        &controller, &config, -500, 0, 2000U, 25U);
    assert(controller.raw_output_mdeg == 300);
    assert(output == 20);

    ball_position_controller_reset(&controller);
    output = ball_position_controller_update(
        &controller, &config, 0, 100, 3000U, 25U);
    assert(controller.raw_output_mdeg == -15);
    assert(output == -15);

    config.motor_sign = -1;
    ball_position_controller_reset(&controller);
    output = ball_position_controller_update(
        &controller, &config, 500, 0, 4000U, 25U);
    assert(controller.raw_output_mdeg == 300);
    assert(output == 20);
}

int main(void)
{
    test_gate();
    test_controller();
    puts("BallObservation T2/T3 host tests: PASS");
    return 0;
}
