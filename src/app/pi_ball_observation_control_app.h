#ifndef PI_BALL_OBSERVATION_CONTROL_APP_H
#define PI_BALL_OBSERVATION_CONTROL_APP_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/ball_observation_stream.h"
#include "control/ball_observation_gate.h"
#include "control/ball_position_controller.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef enum {
    BALL_CONTROL_LIVE_STARTUP = 0,
    BALL_CONTROL_LIVE_WAITING = 1,
    BALL_CONTROL_LIVE_REACQUIRING = 2,
    BALL_CONTROL_LIVE_ACTIVE = 3,
    BALL_CONTROL_LIVE_STALE = 4,
    BALL_CONTROL_LIVE_LINK_TIMEOUT = 5,
    BALL_CONTROL_LIVE_EMERGENCY_LATCHED = 6,
    BALL_CONTROL_LIVE_STARTUP_FAILED = 7
} BallControlLiveMode;

typedef struct {
    uint32_t latest_version_seen;
    uint32_t valid_observations;
    uint32_t rejected_observations;
    uint32_t control_updates;
    uint32_t controller_resets;
    uint32_t stale_events;
    uint32_t link_timeout_events;
    uint32_t emergency_events;
    uint32_t startup_failures;
    uint32_t mode_changes;

    uint32_t desired_version;
    uint32_t applied_version;
    uint32_t motor_apply_attempts;
    uint32_t motor_apply_success;
    uint32_t motor_apply_failures;
    uint32_t motor_busy_deferrals;
    uint32_t motor_enable_attempts;
    uint32_t motor_enable_success;
    uint32_t motor_disable_attempts;
    uint32_t motor_disable_success;

    uint32_t total_age_ms;
    uint32_t last_new_observation_ms;
    uint32_t last_control_update_ms;
    uint32_t last_motor_apply_ms;
    uint32_t reenable_ready_ms;

    int32_t target_position_centi_cm;
    int32_t error_centi_cm;
    int32_t raw_output_mdeg;
    int32_t amplitude_limited_mdeg;
    int32_t target_offset_mdeg;
    int32_t applied_target_offset_mdeg;
    int32_t motor_commanded_physical_mdeg;
    int32_t motor_commanded_pulse;

    uint8_t mode;
    uint8_t gate_reason;
    uint8_t source_invalid_reason;
    uint8_t reacquire_count;
    uint8_t control_valid;
    uint8_t emergency_latched;
    uint8_t link_timeout_active;
    uint8_t disable_requested;
    uint8_t reenable_requested;
    uint8_t motor_enabled;
    uint8_t motor_busy;
} BallControlLiveStats;

volatile BallObservationRxStats g_ball_observation_rx_stats;
volatile BallControlLiveStats g_ball_control_live_stats;

static BallObservationGateConfig ball_control_live_gate_config(void)
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

static BallPositionControllerConfig ball_control_live_controller_config(void)
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

static void ball_control_live_set_mode(BallControlLiveMode mode)
{
    if (g_ball_control_live_stats.mode != (uint8_t) mode) {
        g_ball_control_live_stats.mode = (uint8_t) mode;
        g_ball_control_live_stats.mode_changes++;
    }
}

static void ball_control_live_set_desired_offset(int32_t target_offset_mdeg)
{
    if (g_ball_control_live_stats.target_offset_mdeg != target_offset_mdeg) {
        g_ball_control_live_stats.target_offset_mdeg = target_offset_mdeg;
        g_ball_control_live_stats.desired_version++;
    }
}

static void ball_control_live_reset_output(BallPositionController *controller,
                                           BallControlLiveMode mode)
{
    if (controller->has_previous ||
        (g_ball_control_live_stats.target_offset_mdeg != 0)) {
        g_ball_control_live_stats.controller_resets++;
    }
    ball_position_controller_reset(controller);
    g_ball_control_live_stats.error_centi_cm = 0;
    g_ball_control_live_stats.raw_output_mdeg = 0;
    g_ball_control_live_stats.amplitude_limited_mdeg = 0;
    g_ball_control_live_stats.control_valid = 0U;
    ball_control_live_set_desired_offset(0);
    ball_control_live_set_mode(mode);
}

static void ball_control_live_latch_emergency(
    BallPositionController *controller)
{
    if (g_ball_control_live_stats.emergency_latched == 0U) {
        g_ball_control_live_stats.emergency_latched = 1U;
        g_ball_control_live_stats.emergency_events++;
    }
    g_ball_control_live_stats.reacquire_count = 0U;
    g_ball_control_live_stats.disable_requested = 1U;
    g_ball_control_live_stats.reenable_requested = 0U;
    ball_control_live_reset_output(
        controller, BALL_CONTROL_LIVE_EMERGENCY_LATCHED);
}

static void ball_control_live_handle_new_observation(
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

    g_ball_control_live_stats.latest_version_seen =
        stream->stats.latest_version;
    g_ball_control_live_stats.last_new_observation_ms =
        stream->stats.latest_received_ms;
    g_ball_control_live_stats.total_age_ms = gate.total_age_ms;
    g_ball_control_live_stats.gate_reason = (uint8_t) gate.reason;
    g_ball_control_live_stats.source_invalid_reason =
        gate.source_invalid_reason;

    if ((packet != 0) && ball_observation_packet_emergency(packet)) {
        ball_control_live_latch_emergency(controller);
        return;
    }
    if (g_ball_control_live_stats.emergency_latched != 0U) {
        return;
    }

    if (!gate.control_valid) {
        g_ball_control_live_stats.rejected_observations++;
        g_ball_control_live_stats.reacquire_count = 0U;
        ball_control_live_reset_output(
            controller,
            gate.reason == BALL_OBS_GATE_STALE ?
                BALL_CONTROL_LIVE_STALE : BALL_CONTROL_LIVE_WAITING);
        return;
    }

    g_ball_control_live_stats.valid_observations++;
    if (g_ball_control_live_stats.mode != BALL_CONTROL_LIVE_ACTIVE) {
        if (g_ball_control_live_stats.reacquire_count < 0xFFU) {
            g_ball_control_live_stats.reacquire_count++;
        }
        ball_control_live_set_mode(BALL_CONTROL_LIVE_REACQUIRING);
        ball_control_live_set_desired_offset(0);
        if (g_ball_control_live_stats.reacquire_count <
            APP_BALL_OBS_REACQUIRE_FRAMES) {
            g_ball_control_live_stats.control_valid = 0U;
            return;
        }

        g_ball_control_live_stats.link_timeout_active = 0U;
        g_ball_control_live_stats.disable_requested = 0U;
#if APP_BALL_LIVE_AUTO_REENABLE_AFTER_RECOVERY != 0U
        g_ball_control_live_stats.reenable_requested = 1U;
#endif
        ball_control_live_set_mode(BALL_CONTROL_LIVE_ACTIVE);
    }

    g_ball_control_live_stats.control_valid = 1U;
    (void) ball_position_controller_update(
        controller,
        controller_config,
        packet->position_centi_cm,
        packet->velocity_centi_cm_s,
        now_ms,
        APP_BALL_OBS_EXPECTED_PERIOD_MS);
    g_ball_control_live_stats.control_updates++;
    g_ball_control_live_stats.last_control_update_ms = now_ms;
    g_ball_control_live_stats.error_centi_cm =
        controller->error_centi_cm;
    g_ball_control_live_stats.raw_output_mdeg =
        controller->raw_output_mdeg;
    g_ball_control_live_stats.amplitude_limited_mdeg =
        controller->amplitude_limited_mdeg;
    ball_control_live_set_desired_offset(controller->target_offset_mdeg);
}

static void ball_control_live_check_age_and_link(
    const BallObservationStream *stream,
    const BallObservationGateConfig *gate_config,
    BallPositionController *controller,
    uint32_t now_ms)
{
    const BallObservationPacket *packet;
    BallObservationGateResult gate;
    uint32_t new_observation_age;

    if ((g_ball_control_live_stats.emergency_latched != 0U) ||
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
    g_ball_control_live_stats.total_age_ms = gate.total_age_ms;
    g_ball_control_live_stats.gate_reason = (uint8_t) gate.reason;
    g_ball_control_live_stats.source_invalid_reason =
        gate.source_invalid_reason;

    new_observation_age = (uint32_t) (
        now_ms - stream->stats.latest_received_ms);
    if (new_observation_age > APP_BALL_OBS_LINK_TIMEOUT_MS) {
        if (g_ball_control_live_stats.link_timeout_active == 0U) {
            g_ball_control_live_stats.link_timeout_active = 1U;
            g_ball_control_live_stats.link_timeout_events++;
        }
        g_ball_control_live_stats.reacquire_count = 0U;
#if APP_BALL_LIVE_DISABLE_ON_LINK_TIMEOUT != 0U
        g_ball_control_live_stats.disable_requested = 1U;
#endif
        ball_control_live_reset_output(
            controller, BALL_CONTROL_LIVE_LINK_TIMEOUT);
        return;
    }

    if (!gate.control_valid &&
        ((g_ball_control_live_stats.mode == BALL_CONTROL_LIVE_ACTIVE) ||
         (g_ball_control_live_stats.mode == BALL_CONTROL_LIVE_REACQUIRING))) {
        if (gate.reason == BALL_OBS_GATE_STALE) {
            g_ball_control_live_stats.stale_events++;
        }
        g_ball_control_live_stats.reacquire_count = 0U;
        ball_control_live_reset_output(
            controller,
            gate.reason == BALL_OBS_GATE_STALE ?
                BALL_CONTROL_LIVE_STALE : BALL_CONTROL_LIVE_WAITING);
    }
}

static void ball_control_live_apply_motor(RodMotorControl *motor,
                                          uint32_t now_ms)
{
    bool accepted;

    if ((g_ball_control_live_stats.emergency_latched != 0U) ||
        (g_ball_control_live_stats.disable_requested != 0U)) {
        if (motor->enabled) {
            g_ball_control_live_stats.motor_disable_attempts++;
            accepted = rod_motor_control_set_enabled(motor, false);
            if (accepted) {
                g_ball_control_live_stats.motor_disable_success++;
            }
        }
        return;
    }

    if ((g_ball_control_live_stats.reenable_requested != 0U) &&
        motor->enabled) {
        g_ball_control_live_stats.reenable_requested = 0U;
    }

    if ((g_ball_control_live_stats.reenable_requested != 0U) &&
        !motor->enabled) {
        g_ball_control_live_stats.motor_enable_attempts++;
        accepted = rod_motor_control_set_enabled(motor, true);
        if (accepted) {
            g_ball_control_live_stats.motor_enable_success++;
            g_ball_control_live_stats.reenable_requested = 0U;
            g_ball_control_live_stats.reenable_ready_ms =
                now_ms + APP_ROD_COMMAND_GAP_MS;
        }
        return;
    }

    if (!motor->enabled ||
        ((int32_t) (g_ball_control_live_stats.reenable_ready_ms - now_ms) > 0) ||
        (g_ball_control_live_stats.desired_version ==
         g_ball_control_live_stats.applied_version) ||
        ((uint32_t) (now_ms - g_ball_control_live_stats.last_motor_apply_ms) <
         APP_BALL_LIVE_MOTOR_UPDATE_PERIOD_MS)) {
        return;
    }

    if (rod_motor_control_is_busy(motor, now_ms)) {
        g_ball_control_live_stats.motor_busy_deferrals++;
        return;
    }

    g_ball_control_live_stats.motor_apply_attempts++;
    accepted = rod_motor_control_set_target_offset_mdeg(
        motor,
        g_ball_control_live_stats.target_offset_mdeg,
        now_ms);
    if (accepted) {
        g_ball_control_live_stats.applied_version =
            g_ball_control_live_stats.desired_version;
        g_ball_control_live_stats.applied_target_offset_mdeg =
            g_ball_control_live_stats.target_offset_mdeg;
        g_ball_control_live_stats.last_motor_apply_ms = now_ms;
        g_ball_control_live_stats.motor_apply_success++;
    } else {
        g_ball_control_live_stats.motor_apply_failures++;
    }
}

static void ball_control_live_drain_stale_rx(void)
{
    uint8_t ignored;
    while (vision_uart_read(&ignored)) {
    }
}

static void pi_ball_observation_control_app_run(void)
{
    BallObservationStream stream;
    BallObservationGateConfig gate_config = ball_control_live_gate_config();
    BallPositionControllerConfig controller_config =
        ball_control_live_controller_config();
    BallPositionController controller;
    RodMotorControl motor;

    ball_observation_stream_init(&stream);
    ball_position_controller_init(&controller);
    rod_motor_control_init(&motor);
    memset((void *) &g_ball_observation_rx_stats,
           0,
           sizeof(g_ball_observation_rx_stats));
    memset((void *) &g_ball_control_live_stats,
           0,
           sizeof(g_ball_control_live_stats));
    g_ball_control_live_stats.target_position_centi_cm =
        APP_BALL_CONTROL_TARGET_CENTI_CM;
    g_ball_control_live_stats.gate_reason =
        (uint8_t) BALL_OBS_GATE_NO_OBSERVATION;
    g_ball_control_live_stats.total_age_ms = 0xFFFFFFFFU;
    ball_control_live_set_mode(BALL_CONTROL_LIVE_STARTUP);

    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        g_ball_control_live_stats.startup_failures++;
        ball_control_live_set_mode(BALL_CONTROL_LIVE_STARTUP_FAILED);
        for (;;) {
            emm_v5_poll(bsp_time_ms());
            __WFI();
        }
    }

    ball_control_live_drain_stale_rx();
    ball_control_live_set_mode(BALL_CONTROL_LIVE_WAITING);

    for (;;) {
        uint32_t processed;
        uint32_t now_ms = bsp_time_ms();

        emm_v5_poll(now_ms);
        processed = ball_observation_stream_poll(
            &stream, APP_BALL_OBS_RX_BUDGET_BYTES);
        memcpy((void *) &g_ball_observation_rx_stats,
               &stream.stats,
               sizeof(g_ball_observation_rx_stats));

        if (stream.stats.latest_version !=
            g_ball_control_live_stats.latest_version_seen) {
            ball_control_live_handle_new_observation(
                &stream,
                &gate_config,
                &controller_config,
                &controller,
                now_ms);
        }
        ball_control_live_check_age_and_link(
            &stream, &gate_config, &controller, now_ms);
        ball_control_live_apply_motor(&motor, now_ms);

        g_ball_control_live_stats.motor_enabled = motor.enabled ? 1U : 0U;
        g_ball_control_live_stats.motor_busy =
            rod_motor_control_is_busy(&motor, now_ms) ? 1U : 0U;
        g_ball_control_live_stats.motor_commanded_physical_mdeg =
            motor.commanded_physical_mdeg;
        g_ball_control_live_stats.motor_commanded_pulse =
            motor.commanded_pulse;

        if (processed < APP_BALL_OBS_RX_BUDGET_BYTES) {
            __WFI();
        }
    }
}

#endif
