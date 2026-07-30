#ifndef BALL_CONTROL_DRY_RUN_TEST_H
#define BALL_CONTROL_DRY_RUN_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/ball_observation_stream.h"
#include "control/ball_observation_gate.h"
#include "control/ball_position_controller.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum {
    BALL_CONTROL_DRY_WAITING = 0,
    BALL_CONTROL_DRY_REACQUIRING = 1,
    BALL_CONTROL_DRY_ACTIVE = 2,
    BALL_CONTROL_DRY_STALE = 3,
    BALL_CONTROL_DRY_LINK_TIMEOUT = 4,
    BALL_CONTROL_DRY_EMERGENCY_LATCHED = 5
} BallControlDryRunMode;

typedef struct {
    uint32_t latest_version_seen;
    uint32_t valid_observations;
    uint32_t rejected_observations;
    uint32_t control_updates;
    uint32_t controller_resets;
    uint32_t stale_events;
    uint32_t link_timeout_events;
    uint32_t emergency_events;
    uint32_t mode_changes;

    uint32_t total_age_ms;
    uint32_t last_new_observation_ms;
    uint32_t last_control_update_ms;

    int32_t target_position_centi_cm;
    int32_t error_centi_cm;
    int32_t raw_output_mdeg;
    int32_t amplitude_limited_mdeg;
    int32_t target_offset_mdeg;

    uint8_t mode;
    uint8_t invalid_reason;
    uint8_t reacquire_count;
    uint8_t control_valid;
    uint8_t emergency_latched;
} BallControlDryRunStats;

volatile BallObservationRxStats g_ball_observation_rx_stats;
volatile BallControlDryRunStats g_ball_control_dry_run_stats;

static BallObservationGateConfig ball_control_dry_run_gate_config(void)
{
    BallObservationGateConfig config;
    config.minimum_position_centi_cm = APP_BALL_OBS_POSITION_MIN_CENTI_CM;
    config.maximum_position_centi_cm = APP_BALL_OBS_POSITION_MAX_CENTI_CM;
    config.maximum_abs_velocity_centi_cm_s =
        APP_BALL_OBS_MAX_ABS_VELOCITY_CENTI_CM_S;
    config.minimum_confidence_milli = APP_BALL_OBS_MIN_CONFIDENCE_MILLI;
    config.maximum_total_age_ms = APP_BALL_OBS_VALID_MAX_AGE_MS;
    config.allow_predicted = APP_BALL_OBS_ALLOW_PREDICTED != 0U;
    return config;
}

static BallPositionControllerConfig ball_control_dry_run_controller_config(void)
{
    BallPositionControllerConfig config;
    config.target_position_centi_cm = APP_BALL_CONTROL_TARGET_CENTI_CM;
    config.position_kp_mdeg_per_cm = APP_BALL_CONTROL_KP_MDEG_PER_CM;
    config.velocity_kd_mdeg_per_cm_s = APP_BALL_CONTROL_KD_MDEG_PER_CM_S;
    config.maximum_offset_mdeg = APP_BALL_CONTROL_MAX_OFFSET_MDEG;
    config.maximum_slew_mdeg_per_s = APP_BALL_CONTROL_MAX_SLEW_MDEG_PER_S;
    config.motor_sign = APP_BALL_CONTROL_MOTOR_SIGN;
    return config;
}

static void ball_control_dry_run_set_mode(BallControlDryRunMode mode)
{
    if (g_ball_control_dry_run_stats.mode != (uint8_t) mode) {
        g_ball_control_dry_run_stats.mode = (uint8_t) mode;
        g_ball_control_dry_run_stats.mode_changes++;
    }
}

static void ball_control_dry_run_reset_output(
    BallPositionController *controller,
    BallControlDryRunMode mode)
{
    if (controller->has_previous ||
        (g_ball_control_dry_run_stats.target_offset_mdeg != 0)) {
        g_ball_control_dry_run_stats.controller_resets++;
    }
    ball_position_controller_reset(controller);
    g_ball_control_dry_run_stats.error_centi_cm = 0;
    g_ball_control_dry_run_stats.raw_output_mdeg = 0;
    g_ball_control_dry_run_stats.amplitude_limited_mdeg = 0;
    g_ball_control_dry_run_stats.target_offset_mdeg = 0;
    g_ball_control_dry_run_stats.control_valid = 0U;
    ball_control_dry_run_set_mode(mode);
}

static void ball_control_dry_run_latch_emergency(
    BallPositionController *controller)
{
    if (g_ball_control_dry_run_stats.emergency_latched == 0U) {
        g_ball_control_dry_run_stats.emergency_latched = 1U;
        g_ball_control_dry_run_stats.emergency_events++;
    }
    g_ball_control_dry_run_stats.reacquire_count = 0U;
    ball_control_dry_run_reset_output(
        controller, BALL_CONTROL_DRY_EMERGENCY_LATCHED);
}

static void ball_control_dry_run_handle_new_observation(
    const BallObservationStream *stream,
    const BallObservationGateConfig *gate_config,
    const BallPositionControllerConfig *controller_config,
    BallPositionController *controller,
    uint32_t now_ms)
{
    const BallObservationPacket *packet =
        ball_observation_stream_latest(stream);
    BallObservationGateResult gate = ball_observation_gate_evaluate(
        packet,
        stream->stats.has_observation != 0U,
        stream->stats.latest_received_ms,
        now_ms,
        gate_config);

    g_ball_control_dry_run_stats.latest_version_seen =
        stream->stats.latest_version;
    g_ball_control_dry_run_stats.last_new_observation_ms =
        stream->stats.latest_received_ms;
    g_ball_control_dry_run_stats.total_age_ms = gate.total_age_ms;
    g_ball_control_dry_run_stats.invalid_reason = (uint8_t) gate.reason;

    if ((packet != 0) && ball_observation_packet_emergency(packet)) {
        ball_control_dry_run_latch_emergency(controller);
        return;
    }
    if (g_ball_control_dry_run_stats.emergency_latched != 0U) {
        return;
    }

    if (!gate.control_valid) {
        g_ball_control_dry_run_stats.rejected_observations++;
        g_ball_control_dry_run_stats.reacquire_count = 0U;
        ball_control_dry_run_reset_output(
            controller,
            gate.reason == BALL_OBS_GATE_STALE ?
                BALL_CONTROL_DRY_STALE : BALL_CONTROL_DRY_WAITING);
        return;
    }

    g_ball_control_dry_run_stats.valid_observations++;
    if (g_ball_control_dry_run_stats.mode != BALL_CONTROL_DRY_ACTIVE) {
        if (g_ball_control_dry_run_stats.reacquire_count < 0xFFU) {
            g_ball_control_dry_run_stats.reacquire_count++;
        }
        ball_control_dry_run_set_mode(BALL_CONTROL_DRY_REACQUIRING);
        if (g_ball_control_dry_run_stats.reacquire_count <
            APP_BALL_OBS_REACQUIRE_FRAMES) {
            g_ball_control_dry_run_stats.control_valid = 0U;
            return;
        }
        ball_control_dry_run_set_mode(BALL_CONTROL_DRY_ACTIVE);
    }

    g_ball_control_dry_run_stats.control_valid = 1U;
    (void) ball_position_controller_update(
        controller,
        controller_config,
        packet->position_centi_cm,
        packet->velocity_centi_cm_s,
        now_ms,
        APP_BALL_OBS_EXPECTED_PERIOD_MS);
    g_ball_control_dry_run_stats.control_updates++;
    g_ball_control_dry_run_stats.last_control_update_ms = now_ms;
    g_ball_control_dry_run_stats.error_centi_cm =
        controller->error_centi_cm;
    g_ball_control_dry_run_stats.raw_output_mdeg =
        controller->raw_output_mdeg;
    g_ball_control_dry_run_stats.amplitude_limited_mdeg =
        controller->amplitude_limited_mdeg;
    g_ball_control_dry_run_stats.target_offset_mdeg =
        controller->target_offset_mdeg;
}

static void ball_control_dry_run_check_age_and_link(
    const BallObservationStream *stream,
    const BallObservationGateConfig *gate_config,
    BallPositionController *controller,
    uint32_t now_ms)
{
    const BallObservationPacket *packet;
    BallObservationGateResult gate;
    uint32_t new_observation_age;

    if ((g_ball_control_dry_run_stats.emergency_latched != 0U) ||
        (stream->stats.has_observation == 0U)) {
        return;
    }

    packet = ball_observation_stream_latest(stream);
    gate = ball_observation_gate_evaluate(
        packet,
        true,
        stream->stats.latest_received_ms,
        now_ms,
        gate_config);
    g_ball_control_dry_run_stats.total_age_ms = gate.total_age_ms;
    g_ball_control_dry_run_stats.invalid_reason = (uint8_t) gate.reason;

    new_observation_age = (uint32_t) (
        now_ms - stream->stats.latest_received_ms);
    if (new_observation_age > APP_BALL_OBS_LINK_TIMEOUT_MS) {
        if (g_ball_control_dry_run_stats.mode !=
            BALL_CONTROL_DRY_LINK_TIMEOUT) {
            g_ball_control_dry_run_stats.link_timeout_events++;
        }
        g_ball_control_dry_run_stats.reacquire_count = 0U;
        ball_control_dry_run_reset_output(
            controller, BALL_CONTROL_DRY_LINK_TIMEOUT);
        return;
    }

    if (!gate.control_valid &&
        ((g_ball_control_dry_run_stats.mode == BALL_CONTROL_DRY_ACTIVE) ||
         (g_ball_control_dry_run_stats.mode ==
          BALL_CONTROL_DRY_REACQUIRING))) {
        if (gate.reason == BALL_OBS_GATE_STALE) {
            g_ball_control_dry_run_stats.stale_events++;
        }
        g_ball_control_dry_run_stats.reacquire_count = 0U;
        ball_control_dry_run_reset_output(
            controller,
            gate.reason == BALL_OBS_GATE_STALE ?
                BALL_CONTROL_DRY_STALE : BALL_CONTROL_DRY_WAITING);
    }
}

static void ball_control_dry_run_test_run(void)
{
    BallObservationStream stream;
    BallObservationGateConfig gate_config =
        ball_control_dry_run_gate_config();
    BallPositionControllerConfig controller_config =
        ball_control_dry_run_controller_config();
    BallPositionController controller;

    ball_observation_stream_init(&stream);
    ball_position_controller_init(&controller);
    memset((void *) &g_ball_observation_rx_stats,
           0,
           sizeof(g_ball_observation_rx_stats));
    memset((void *) &g_ball_control_dry_run_stats,
           0,
           sizeof(g_ball_control_dry_run_stats));
    g_ball_control_dry_run_stats.target_position_centi_cm =
        APP_BALL_CONTROL_TARGET_CENTI_CM;
    g_ball_control_dry_run_stats.invalid_reason =
        (uint8_t) BALL_OBS_GATE_NO_OBSERVATION;
    g_ball_control_dry_run_stats.total_age_ms = 0xFFFFFFFFU;
    ball_control_dry_run_set_mode(BALL_CONTROL_DRY_WAITING);

    for (;;) {
        uint32_t processed = ball_observation_stream_poll(
            &stream, APP_BALL_OBS_RX_BUDGET_BYTES);
        uint32_t now_ms = bsp_time_ms();

        memcpy((void *) &g_ball_observation_rx_stats,
               &stream.stats,
               sizeof(g_ball_observation_rx_stats));

        if (stream.stats.latest_version !=
            g_ball_control_dry_run_stats.latest_version_seen) {
            ball_control_dry_run_handle_new_observation(
                &stream,
                &gate_config,
                &controller_config,
                &controller,
                now_ms);
        }
        ball_control_dry_run_check_age_and_link(
            &stream, &gate_config, &controller, now_ms);

        /*
         * Stage T3 is intentionally compute-only. It never initializes UART1,
         * EmmV5, RodMotorControl or any motor output path.
         */
        if (processed < APP_BALL_OBS_RX_BUDGET_BYTES) {
            __WFI();
        }
    }
}

#endif
