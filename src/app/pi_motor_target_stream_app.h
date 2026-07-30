#ifndef PI_MOTOR_TARGET_STREAM_APP_H
#define PI_MOTOR_TARGET_STREAM_APP_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/motor_target_protocol.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

typedef struct {
    uint32_t rx_bytes;
    uint32_t completed_frames;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t flag_errors;
    uint32_t range_errors;
    uint32_t target_changes;
    uint32_t duplicate_targets;
    uint32_t busy_deferrals;
    uint32_t apply_attempts;
    uint32_t apply_success;
    uint32_t apply_failures;
    uint32_t timeout_events;
    uint32_t disable_events;
    uint32_t startup_failures;
    uint32_t last_valid_rx_ms;
    int32_t last_target_offset_mdeg;
    uint8_t last_sequence;
} PiMotorTargetStreamStats;

/* Inspect this symbol with Keil Watch when diagnostics are needed. */
volatile PiMotorTargetStreamStats g_pi_motor_target_stream_stats;

typedef struct {
    int32_t target_offset_mdeg;
    uint32_t last_valid_rx_ms;
    uint32_t version;
    uint32_t applied_version;
    uint32_t busy_reported_version;
    uint8_t sequence;
    bool has_rx;
    bool valid;
    bool disable_latched;
    bool timed_out;
} PiLatestMotorTarget;

static void pi_motor_target_stream_stats_init(void)
{
    memset((void *) &g_pi_motor_target_stream_stats,
           0,
           sizeof(g_pi_motor_target_stream_stats));
}

static void pi_latest_motor_target_init(PiLatestMotorTarget *latest)
{
    latest->target_offset_mdeg = 0;
    latest->last_valid_rx_ms = 0U;
    latest->version = 0U;
    latest->applied_version = 0U;
    latest->busy_reported_version = 0xFFFFFFFFU;
    latest->sequence = 0U;
    latest->has_rx = false;
    latest->valid = false;
    latest->disable_latched = false;
    latest->timed_out = false;
}

static void pi_motor_target_stream_accept(
    PiLatestMotorTarget *latest,
    const MotorTargetPacket *packet,
    uint32_t now_ms)
{
    bool changed = false;

    latest->sequence = packet->sequence;
    latest->last_valid_rx_ms = now_ms;
    latest->has_rx = true;
    latest->timed_out = false;

    g_pi_motor_target_stream_stats.frames_ok++;
    g_pi_motor_target_stream_stats.last_valid_rx_ms = now_ms;
    g_pi_motor_target_stream_stats.last_sequence = packet->sequence;
    g_pi_motor_target_stream_stats.last_target_offset_mdeg =
        packet->target_offset_mdeg;

    if ((packet->flags & MOTOR_TARGET_FLAG_DISABLE) != 0U) {
        if (!latest->disable_latched) {
            latest->disable_latched = true;
            latest->valid = false;
            latest->version++;
            latest->busy_reported_version = 0xFFFFFFFFU;
            g_pi_motor_target_stream_stats.disable_events++;
            g_pi_motor_target_stream_stats.target_changes++;
        } else {
            g_pi_motor_target_stream_stats.duplicate_targets++;
        }
        return;
    }

    /* Emergency disable is intentionally latched until TI reset. */
    if (latest->disable_latched) {
        g_pi_motor_target_stream_stats.duplicate_targets++;
        return;
    }

    if ((packet->flags & MOTOR_TARGET_FLAG_VALID) != 0U) {
        changed = !latest->valid ||
                  (packet->target_offset_mdeg != latest->target_offset_mdeg);
        latest->target_offset_mdeg = packet->target_offset_mdeg;
        latest->valid = true;
    } else {
        changed = latest->valid;
        latest->valid = false;
    }

    if (changed) {
        latest->version++;
        latest->busy_reported_version = 0xFFFFFFFFU;
        g_pi_motor_target_stream_stats.target_changes++;
    } else {
        g_pi_motor_target_stream_stats.duplicate_targets++;
    }
}

static void pi_motor_target_stream_note_parse_result(
    PiLatestMotorTarget *latest,
    MotorTargetParseResult result,
    const MotorTargetPacket *packet,
    uint32_t now_ms)
{
    if (result == MOTOR_TARGET_PARSE_INCOMPLETE) {
        return;
    }

    g_pi_motor_target_stream_stats.completed_frames++;
    switch (result) {
    case MOTOR_TARGET_PARSE_ACCEPTED:
        pi_motor_target_stream_accept(latest, packet, now_ms);
        break;
    case MOTOR_TARGET_PARSE_BAD_CRC:
        g_pi_motor_target_stream_stats.crc_errors++;
        break;
    case MOTOR_TARGET_PARSE_BAD_FLAGS:
        g_pi_motor_target_stream_stats.flag_errors++;
        break;
    case MOTOR_TARGET_PARSE_BAD_RANGE:
        g_pi_motor_target_stream_stats.range_errors++;
        break;
    default:
        break;
    }
}

static void pi_motor_target_stream_apply_timeout(
    PiLatestMotorTarget *latest,
    uint32_t now_ms)
{
    if (!latest->has_rx || latest->timed_out || latest->disable_latched) {
        return;
    }
    if ((uint32_t) (now_ms - latest->last_valid_rx_ms) <=
        APP_PI_TARGET_TIMEOUT_MS) {
        return;
    }

    latest->timed_out = true;
    g_pi_motor_target_stream_stats.timeout_events++;

    if (!latest->valid || latest->target_offset_mdeg != 0) {
        latest->target_offset_mdeg = 0;
        latest->valid = true;
        latest->version++;
        latest->busy_reported_version = 0xFFFFFFFFU;
        g_pi_motor_target_stream_stats.target_changes++;
        g_pi_motor_target_stream_stats.last_target_offset_mdeg = 0;
    }
}

static void pi_motor_target_stream_drain_stale_rx(void)
{
    uint8_t ignored;
    while (vision_uart_read(&ignored)) {
    }
}

static void pi_motor_target_stream_apply_latest(
    PiLatestMotorTarget *latest,
    RodMotorControl *motor,
    uint32_t now_ms)
{
    bool accepted;
    int32_t effective_target_mdeg;

    if (latest->disable_latched &&
        (latest->version != latest->applied_version)) {
        g_pi_motor_target_stream_stats.apply_attempts++;
        accepted = !motor->enabled ||
                   rod_motor_control_set_enabled(motor, false);
        if (accepted) {
            latest->applied_version = latest->version;
            g_pi_motor_target_stream_stats.apply_success++;
        } else {
            g_pi_motor_target_stream_stats.apply_failures++;
        }
        return;
    }

    if (!latest->valid ||
        latest->disable_latched ||
        (latest->version == latest->applied_version)) {
        return;
    }

    if (rod_motor_control_is_busy(motor, now_ms)) {
        if (latest->busy_reported_version != latest->version) {
            latest->busy_reported_version = latest->version;
            g_pi_motor_target_stream_stats.busy_deferrals++;
        }
        return;
    }

    effective_target_mdeg =
        APP_PI_TARGET_DIRECTION_SIGN * latest->target_offset_mdeg;
    g_pi_motor_target_stream_stats.apply_attempts++;
    accepted = rod_motor_control_set_target_offset_mdeg(
        motor,
        effective_target_mdeg,
        now_ms);
    if (accepted) {
        latest->applied_version = latest->version;
        g_pi_motor_target_stream_stats.apply_success++;
    } else {
        g_pi_motor_target_stream_stats.apply_failures++;
    }
}

static void pi_motor_target_stream_app_run(void)
{
    MotorTargetParser parser;
    MotorTargetPacket packet;
    PiLatestMotorTarget latest;
    RodMotorControl motor;

    motor_target_parser_init(&parser);
    pi_latest_motor_target_init(&latest);
    pi_motor_target_stream_stats_init();
    rod_motor_control_init(&motor);

    /*
     * Keep UART0 IRQ/ring reception enabled. This is the path verified by both
     * the CH340 finite test and the Raspberry Pi 20 Hz sweep.
     */
    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        g_pi_motor_target_stream_stats.startup_failures++;
        for (;;) {
            __WFI();
        }
    }

    /* Ignore host bytes transmitted during the blocking startup sequence. */
    pi_motor_target_stream_drain_stale_rx();

    for (;;) {
        uint32_t now_ms = bsp_time_ms();
        uint32_t processed = 0U;
        uint8_t byte;

        emm_v5_poll(now_ms);

        while ((processed < APP_PI_TARGET_RX_BUDGET_BYTES) &&
               vision_uart_read(&byte)) {
            MotorTargetParseResult result;
            g_pi_motor_target_stream_stats.rx_bytes++;
            result = motor_target_parser_push(
                &parser,
                byte,
                APP_PI_TARGET_MIN_OFFSET_MDEG,
                APP_PI_TARGET_MAX_OFFSET_MDEG,
                &packet);
            pi_motor_target_stream_note_parse_result(
                &latest,
                result,
                &packet,
                now_ms);
            processed++;
        }

        pi_motor_target_stream_apply_timeout(&latest, now_ms);
        pi_motor_target_stream_apply_latest(&latest, &motor, now_ms);
        __WFI();
    }
}

#endif
