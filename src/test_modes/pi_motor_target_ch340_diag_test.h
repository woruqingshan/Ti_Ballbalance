#ifndef PI_MOTOR_TARGET_CH340_DIAG_TEST_H
#define PI_MOTOR_TARGET_CH340_DIAG_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "protocol/crc16.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * Event-driven CH340 diagnostic.
 *
 * Wiring:
 *   CH340 TX -> TI PA11 / UART0_RX
 *   CH340 RX <- TI PA10 / UART0_TX
 *   CH340 GND <-> TI GND
 *
 * No heartbeat and no repeated request are used. The PC sends one finite
 * 8-byte target frame. TI reports each receive/parse/apply stage only when an
 * event occurs.
 *
 * This mode intentionally uses vision_uart_read(), exactly like the previously
 * successful motor_manual_control_test.h. It therefore tests whether the
 * existing UART0 IRQ + ring-buffer path can receive the binary target frame.
 */

#define PI_CH340_TARGET_SOF_0          0xA5U
#define PI_CH340_TARGET_SOF_1          0x5AU
#define PI_CH340_TARGET_FLAG_VALID     0x01U
#define PI_CH340_TARGET_FLAG_DISABLE   0x02U
#define PI_CH340_TARGET_FRAME_SIZE     8U

typedef struct {
    uint8_t frame[PI_CH340_TARGET_FRAME_SIZE];
    uint8_t index;
    uint32_t rx_bytes;
    uint32_t completed_frames;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t range_errors;
} PiCh340TargetParser;

typedef struct {
    int32_t target_offset_mdeg;
    uint32_t version;
    uint32_t applied_version;
    uint32_t busy_reported_version;
    uint8_t sequence;
    bool valid;
    bool disable_requested;
} PiCh340LatestTarget;

static void pi_ch340_parser_init(PiCh340TargetParser *parser)
{
    parser->index = 0U;
    parser->rx_bytes = 0U;
    parser->completed_frames = 0U;
    parser->frames_ok = 0U;
    parser->crc_errors = 0U;
    parser->range_errors = 0U;
}

static void pi_ch340_latest_init(PiCh340LatestTarget *latest)
{
    latest->target_offset_mdeg = 0;
    latest->version = 0U;
    latest->applied_version = 0U;
    latest->busy_reported_version = 0xFFFFFFFFU;
    latest->sequence = 0U;
    latest->valid = false;
    latest->disable_requested = false;
}

static void pi_ch340_print_motor_state(const RodMotorControl *motor,
                                       uint32_t now_ms)
{
    test_console_write(" enabled=");
    test_console_write_u32(motor->enabled ? 1U : 0U);
    test_console_write(" startup=");
    test_console_write_u32(motor->startup_complete ? 1U : 0U);
    test_console_write(" busy=");
    test_console_write_u32(
        rod_motor_control_is_busy(motor, now_ms) ? 1U : 0U);
    test_console_write(" physical_mdeg=");
    test_console_write_i32(motor->commanded_physical_mdeg);
    test_console_write(" pulse=");
    test_console_write_i32(motor->commanded_pulse);
}

static void pi_ch340_resync(PiCh340TargetParser *parser, uint8_t byte)
{
    if (byte == PI_CH340_TARGET_SOF_0) {
        parser->frame[0] = byte;
        parser->index = 1U;
    } else {
        parser->index = 0U;
    }
}

static bool pi_ch340_accept_frame(PiCh340TargetParser *parser,
                                  PiCh340LatestTarget *latest)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t raw_target;
    int32_t target_mdeg;
    uint8_t sequence;
    uint8_t flags;
    bool changed;

    parser->completed_frames++;

    received_crc = (uint16_t) parser->frame[6] |
                   ((uint16_t) parser->frame[7] << 8U);
    calculated_crc = crc16_ccitt_false(&parser->frame[2], 4U);
    sequence = parser->frame[2];
    flags = parser->frame[3];

    if (received_crc != calculated_crc) {
        parser->crc_errors++;
        test_console_write("FRAME_BAD_CRC seq=");
        test_console_write_u32(sequence);
        test_console_write(" received=");
        test_console_write_u32(received_crc);
        test_console_write(" calculated=");
        test_console_write_u32(calculated_crc);
        test_console_newline();
        return false;
    }

    raw_target = (uint16_t) parser->frame[4] |
                 ((uint16_t) parser->frame[5] << 8U);
    target_mdeg = (int32_t) ((int16_t) raw_target);

    if (((flags & PI_CH340_TARGET_FLAG_VALID) != 0U) &&
        ((target_mdeg < APP_PI_TARGET_MIN_OFFSET_MDEG) ||
         (target_mdeg > APP_PI_TARGET_MAX_OFFSET_MDEG))) {
        parser->range_errors++;
        test_console_write("FRAME_BAD_RANGE seq=");
        test_console_write_u32(sequence);
        test_console_write(" target_mdeg=");
        test_console_write_i32(target_mdeg);
        test_console_newline();
        return false;
    }

    if ((flags & PI_CH340_TARGET_FLAG_DISABLE) != 0U) {
        changed = !latest->disable_requested || latest->valid;
        latest->disable_requested = true;
        latest->valid = false;
    } else if ((flags & PI_CH340_TARGET_FLAG_VALID) != 0U) {
        changed = !latest->valid || latest->disable_requested ||
                  (target_mdeg != latest->target_offset_mdeg);
        latest->target_offset_mdeg = target_mdeg;
        latest->disable_requested = false;
        latest->valid = true;
    } else {
        changed = latest->valid || latest->disable_requested;
        latest->disable_requested = false;
        latest->valid = false;
    }

    latest->sequence = sequence;
    if (changed) {
        latest->version++;
        latest->busy_reported_version = 0xFFFFFFFFU;
    }
    parser->frames_ok++;

    test_console_write("FRAME_OK seq=");
    test_console_write_u32(sequence);
    test_console_write(" flags=");
    test_console_write_u32(flags);
    test_console_write(" target_mdeg=");
    test_console_write_i32(target_mdeg);
    test_console_write(" changed=");
    test_console_write_u32(changed ? 1U : 0U);
    test_console_write(" version=");
    test_console_write_u32(latest->version);
    test_console_newline();
    return true;
}

static void pi_ch340_parser_push(PiCh340TargetParser *parser,
                                 PiCh340LatestTarget *latest,
                                 uint8_t byte)
{
    bool completed = false;

    parser->rx_bytes++;

#if APP_PI_TARGET_DIAG_ECHO_EACH_BYTE
    test_console_write("RX_BYTE count=");
    test_console_write_u32(parser->rx_bytes);
    test_console_write(" value=");
    test_console_write_u32(byte);
    test_console_write(" parser_index=");
    test_console_write_u32(parser->index);
    test_console_newline();
#endif

    if (parser->index == 0U) {
        pi_ch340_resync(parser, byte);
        return;
    }

    if (parser->index == 1U) {
        if (byte == PI_CH340_TARGET_SOF_1) {
            parser->frame[1] = byte;
            parser->index = 2U;
        } else {
            pi_ch340_resync(parser, byte);
        }
        return;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index >= PI_CH340_TARGET_FRAME_SIZE) {
        completed = true;
        (void) pi_ch340_accept_frame(parser, latest);
        parser->index = 0U;
    }

    (void) completed;
}

static void pi_ch340_apply_latest(PiCh340LatestTarget *latest,
                                  RodMotorControl *motor,
                                  uint32_t now_ms)
{
    int32_t effective_target_mdeg;
    bool accepted;

    if (latest->disable_requested &&
        (latest->version != latest->applied_version)) {
        accepted = rod_motor_control_set_enabled(motor, false);
        test_console_write(accepted ? "DISABLE_OK" : "DISABLE_REJECT");
        pi_ch340_print_motor_state(motor, now_ms);
        test_console_newline();
        if (accepted) {
            latest->applied_version = latest->version;
        }
        return;
    }

    if (!latest->valid ||
        (latest->version == latest->applied_version)) {
        return;
    }

    if (rod_motor_control_is_busy(motor, now_ms)) {
        if (latest->busy_reported_version != latest->version) {
            latest->busy_reported_version = latest->version;
            test_console_write("APPLY_WAIT_BUSY target_mdeg=");
            test_console_write_i32(latest->target_offset_mdeg);
            pi_ch340_print_motor_state(motor, now_ms);
            test_console_newline();
        }
        return;
    }

    effective_target_mdeg =
        APP_PI_TARGET_DIRECTION_SIGN * latest->target_offset_mdeg;

    test_console_write("APPLY_TRY target_mdeg=");
    test_console_write_i32(effective_target_mdeg);
    pi_ch340_print_motor_state(motor, now_ms);
    test_console_newline();

    accepted = rod_motor_control_set_target_offset_mdeg(
        motor, effective_target_mdeg, now_ms);

    test_console_write(accepted ? "APPLY_OK target_mdeg="
                                : "APPLY_REJECT target_mdeg=");
    test_console_write_i32(effective_target_mdeg);
    pi_ch340_print_motor_state(motor, now_ms);
    test_console_newline();

    if (accepted) {
        latest->applied_version = latest->version;
    }
}

static void pi_motor_target_ch340_diag_run(void)
{
    PiCh340TargetParser parser;
    PiCh340LatestTarget latest;
    RodMotorControl motor;
    uint8_t ignored;

    pi_ch340_parser_init(&parser);
    pi_ch340_latest_init(&latest);
    rod_motor_control_init(&motor);

    test_console_write_line("TARGET_DIAG_BOOT");
    test_console_write_line(
        "Uses known-good vision_uart_read IRQ/ring receive path");

    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        test_console_write_line("TARGET_DIAG_FATAL_STARTUP");
        for (;;) {
            __WFI();
        }
    }

    /* Discard any bytes sent while the blocking startup sequence was running. */
    while (vision_uart_read(&ignored)) {
    }

    test_console_write("TARGET_DIAG_READY");
    pi_ch340_print_motor_state(&motor, bsp_time_ms());
    test_console_newline();

    for (;;) {
        uint8_t byte;
        uint32_t now_ms = bsp_time_ms();

        emm_v5_poll(now_ms);

        while (vision_uart_read(&byte)) {
            pi_ch340_parser_push(&parser, &latest, byte);
        }

        pi_ch340_apply_latest(&latest, &motor, now_ms);
        __WFI();
    }
}

#endif
