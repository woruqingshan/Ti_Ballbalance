#ifndef BALL_OBSERVATION_GATE_H
#define BALL_OBSERVATION_GATE_H

#include "communication/ball_observation_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BALL_OBS_GATE_VALID = 0,
    BALL_OBS_GATE_NO_OBSERVATION = 1,
    BALL_OBS_GATE_EMERGENCY = 2,
    BALL_OBS_GATE_FLAG_INVALID = 3,
    BALL_OBS_GATE_NOT_FOUND = 4,
    BALL_OBS_GATE_PREDICTED_DISALLOWED = 5,
    BALL_OBS_GATE_LOW_CONFIDENCE = 6,
    BALL_OBS_GATE_POSITION_RANGE = 7,
    BALL_OBS_GATE_VELOCITY_RANGE = 8,
    BALL_OBS_GATE_STALE = 9
} BallObservationGateReason;

typedef struct {
    int16_t minimum_position_centi_cm;
    int16_t maximum_position_centi_cm;
    uint16_t maximum_abs_velocity_centi_cm_s;
    uint16_t minimum_confidence_milli;
    uint32_t maximum_total_age_ms;
    bool allow_predicted;
} BallObservationGateConfig;

typedef struct {
    uint32_t total_age_ms;
    BallObservationGateReason reason;
    uint8_t source_invalid_reason;
    bool control_valid;
} BallObservationGateResult;

static uint32_t ball_observation_total_age_ms(
    const BallObservationPacket *packet,
    uint32_t received_ms,
    uint32_t now_ms)
{
    uint32_t elapsed = (uint32_t) (now_ms - received_ms);
    uint32_t source_age = (uint32_t) packet->vision_age_ms;

    if (elapsed > (0xFFFFFFFFU - source_age)) {
        return 0xFFFFFFFFU;
    }
    return source_age + elapsed;
}

static BallObservationGateResult ball_observation_gate_evaluate(
    const BallObservationPacket *packet,
    bool has_observation,
    uint32_t received_ms,
    uint32_t now_ms,
    const BallObservationGateConfig *config)
{
    BallObservationGateResult result;
    int32_t velocity;

    result.total_age_ms = 0xFFFFFFFFU;
    result.reason = BALL_OBS_GATE_NO_OBSERVATION;
    result.source_invalid_reason =
        (uint8_t) BALL_OBSERVATION_INVALID_NONE;
    result.control_valid = false;

    if (!has_observation || (packet == 0) || (config == 0)) {
        return result;
    }

    result.total_age_ms = ball_observation_total_age_ms(
        packet, received_ms, now_ms);
    result.source_invalid_reason =
        ball_observation_packet_invalid_reason(packet);

    if (ball_observation_packet_emergency(packet)) {
        result.reason = BALL_OBS_GATE_EMERGENCY;
        return result;
    }
    if (!ball_observation_packet_valid(packet)) {
        result.reason = BALL_OBS_GATE_FLAG_INVALID;
        return result;
    }
    if (ball_observation_packet_predicted(packet) &&
        !config->allow_predicted) {
        result.reason = BALL_OBS_GATE_PREDICTED_DISALLOWED;
        return result;
    }
    if (!ball_observation_packet_found(packet) &&
        !(config->allow_predicted &&
          ball_observation_packet_predicted(packet))) {
        result.reason = BALL_OBS_GATE_NOT_FOUND;
        return result;
    }
    if (packet->confidence_milli < config->minimum_confidence_milli) {
        result.reason = BALL_OBS_GATE_LOW_CONFIDENCE;
        return result;
    }
    if ((packet->position_centi_cm < config->minimum_position_centi_cm) ||
        (packet->position_centi_cm > config->maximum_position_centi_cm)) {
        result.reason = BALL_OBS_GATE_POSITION_RANGE;
        return result;
    }

    velocity = (int32_t) packet->velocity_centi_cm_s;
    if (velocity < 0) {
        velocity = -velocity;
    }
    if ((uint32_t) velocity >
        (uint32_t) config->maximum_abs_velocity_centi_cm_s) {
        result.reason = BALL_OBS_GATE_VELOCITY_RANGE;
        return result;
    }
    if (result.total_age_ms > config->maximum_total_age_ms) {
        result.reason = BALL_OBS_GATE_STALE;
        return result;
    }

    result.reason = BALL_OBS_GATE_VALID;
    result.control_valid = true;
    return result;
}

#endif
