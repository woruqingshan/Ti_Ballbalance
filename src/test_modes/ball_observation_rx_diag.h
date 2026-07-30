#ifndef BALL_OBSERVATION_RX_DIAG_H
#define BALL_OBSERVATION_RX_DIAG_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/ball_observation_stream.h"
#include "control/ball_observation_gate.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <string.h>

#if APP_BALL_OBS_FRAME_SIZE != BALL_OBSERVATION_FRAME_SIZE
#error APP_BALL_OBS_FRAME_SIZE must match BALL_OBSERVATION_FRAME_SIZE
#endif

typedef struct {
    uint32_t evaluations;
    uint32_t state_changes;
    uint32_t latest_version_evaluated;
    uint32_t total_age_ms;
    uint32_t last_evaluation_ms;
    int16_t position_centi_cm;
    int16_t velocity_centi_cm_s;
    uint16_t confidence_milli;
    uint16_t vision_age_ms;
    uint8_t sequence;
    uint8_t flags;
    uint8_t control_valid;
    uint8_t invalid_reason;
} BallObservationObserverStats;

/* Existing T1 Watch symbol is preserved for backwards-compatible diagnostics. */
volatile BallObservationRxStats g_ball_observation_rx_stats;
volatile BallObservationObserverStats g_ball_observation_observer_stats;

static BallObservationGateConfig ball_observation_observer_gate_config(void)
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

static void ball_observation_observer_publish_rx_stats(
    const BallObservationStream *stream)
{
    memcpy((void *) &g_ball_observation_rx_stats,
           &stream->stats,
           sizeof(g_ball_observation_rx_stats));
}

static void ball_observation_observer_evaluate(
    const BallObservationStream *stream,
    const BallObservationGateConfig *config,
    uint32_t now_ms)
{
    const BallObservationPacket *packet =
        ball_observation_stream_latest(stream);
    BallObservationGateResult result = ball_observation_gate_evaluate(
        packet,
        stream->stats.has_observation != 0U,
        stream->stats.latest_received_ms,
        now_ms,
        config);
    uint8_t new_valid = result.control_valid ? 1U : 0U;
    uint8_t new_reason = (uint8_t) result.reason;

    if ((g_ball_observation_observer_stats.evaluations != 0U) &&
        ((g_ball_observation_observer_stats.control_valid != new_valid) ||
         (g_ball_observation_observer_stats.invalid_reason != new_reason))) {
        g_ball_observation_observer_stats.state_changes++;
    }

    g_ball_observation_observer_stats.evaluations++;
    g_ball_observation_observer_stats.latest_version_evaluated =
        stream->stats.latest_version;
    g_ball_observation_observer_stats.total_age_ms = result.total_age_ms;
    g_ball_observation_observer_stats.last_evaluation_ms = now_ms;
    g_ball_observation_observer_stats.control_valid = new_valid;
    g_ball_observation_observer_stats.invalid_reason = new_reason;

    if (packet != 0) {
        g_ball_observation_observer_stats.position_centi_cm =
            packet->position_centi_cm;
        g_ball_observation_observer_stats.velocity_centi_cm_s =
            packet->velocity_centi_cm_s;
        g_ball_observation_observer_stats.confidence_milli =
            packet->confidence_milli;
        g_ball_observation_observer_stats.vision_age_ms =
            packet->vision_age_ms;
        g_ball_observation_observer_stats.sequence = packet->sequence;
        g_ball_observation_observer_stats.flags = packet->flags;
    }
}

static void ball_observation_rx_diag_run(void)
{
    BallObservationStream stream;
    BallObservationGateConfig gate_config =
        ball_observation_observer_gate_config();
    uint32_t last_observer_ms = 0U;

    ball_observation_stream_init(&stream);
    memset((void *) &g_ball_observation_rx_stats,
           0,
           sizeof(g_ball_observation_rx_stats));
    memset((void *) &g_ball_observation_observer_stats,
           0,
           sizeof(g_ball_observation_observer_stats));
    g_ball_observation_observer_stats.invalid_reason =
        (uint8_t) BALL_OBS_GATE_NO_OBSERVATION;
    g_ball_observation_observer_stats.total_age_ms = 0xFFFFFFFFU;

    for (;;) {
        uint32_t now_ms;
        uint32_t processed = ball_observation_stream_poll(
            &stream, APP_BALL_OBS_RX_BUDGET_BYTES);

        ball_observation_observer_publish_rx_stats(&stream);
        now_ms = bsp_time_ms();
        if ((uint32_t) (now_ms - last_observer_ms) >=
            APP_BALL_OBS_OBSERVER_PERIOD_MS) {
            last_observer_ms = now_ms;
            ball_observation_observer_evaluate(
                &stream, &gate_config, now_ms);
        }

        /* Mode 18 remains receive/observe only. It never touches UART1 or motor. */
        if (processed < APP_BALL_OBS_RX_BUDGET_BYTES) {
            __WFI();
        }
    }
}

#endif
