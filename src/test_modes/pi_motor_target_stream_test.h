#ifndef PI_MOTOR_TARGET_STREAM_TEST_H
#define PI_MOTOR_TARGET_STREAM_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "protocol/crc16.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define PI_TARGET_SOF_0             0xA5U
#define PI_TARGET_SOF_1             0x5AU
#define PI_TARGET_FLAG_VALID        0x01U
#define PI_TARGET_FLAG_DISABLE      0x02U

typedef struct {
    uint8_t frame[APP_PI_TARGET_FRAME_SIZE];
    uint8_t index;
} PiMotorTargetParser;

typedef struct {
    int32_t target_offset_mdeg;
    uint32_t last_valid_rx_ms;
    uint32_t version;
    uint32_t applied_version;
    uint8_t sequence;
    bool has_rx;
    bool valid;
    bool disable_requested;
    bool timed_out;
} PiLatestMotorTarget;

static void pi_motor_target_parser_init(PiMotorTargetParser *parser)
{
    parser->index = 0U;
}

static void pi_latest_motor_target_init(PiLatestMotorTarget *target)
{
    target->target_offset_mdeg = 0;
    target->last_valid_rx_ms = 0U;
    target->version = 0U;
    target->applied_version = 0U;
    target->sequence = 0U;
    target->has_rx = false;
    target->valid = false;
    target->disable_requested = false;
    target->timed_out = false;
}

static void pi_motor_target_parser_resync(PiMotorTargetParser *parser,
                                          uint8_t byte)
{
    if (byte == PI_TARGET_SOF_0) {
        parser->frame[0] = byte;
        parser->index = 1U;
    } else {
        parser->index = 0U;
    }
}

static bool pi_motor_target_accept_frame(PiMotorTargetParser *parser,
                                         PiLatestMotorTarget *latest,
                                         uint32_t now_ms)
{
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint16_t raw_target;
    int32_t target_mdeg;
    uint8_t flags;
    bool changed;

    received_crc = (uint16_t) parser->frame[6] |
                   ((uint16_t) parser->frame[7] << 8U);
    calculated_crc = crc16_ccitt_false(&parser->frame[2], 4U);
    if (received_crc != calculated_crc) {
        return false;
    }

    flags = parser->frame[3];
    raw_target = (uint16_t) parser->frame[4] |
                 ((uint16_t) parser->frame[5] << 8U);
    target_mdeg = (int32_t) ((int16_t) raw_target);

    if ((flags & PI_TARGET_FLAG_DISABLE) != 0U) {
        changed = !latest->disable_requested || latest->valid;
        latest->disable_requested = true;
        latest->valid = false;
    } else if ((flags & PI_TARGET_FLAG_VALID) != 0U) {
        if ((target_mdeg < APP_PI_TARGET_MIN_OFFSET_MDEG) ||
            (target_mdeg > APP_PI_TARGET_MAX_OFFSET_MDEG)) {
            return false;
        }
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

    latest->sequence = parser->frame[2];
    latest->last_valid_rx_ms = now_ms;
    latest->has_rx = true;
    latest->timed_out = false;
    if (changed) {
        latest->version++;
    }
    return true;
}

static bool pi_motor_target_parser_push(PiMotorTargetParser *parser,
                                        PiLatestMotorTarget *latest,
                                        uint8_t byte,
                                        uint32_t now_ms)
{
    bool accepted = false;

    if (parser->index == 0U) {
        pi_motor_target_parser_resync(parser, byte);
        return false;
    }

    if (parser->index == 1U) {
        if (byte == PI_TARGET_SOF_1) {
            parser->frame[1] = byte;
            parser->index = 2U;
        } else {
            pi_motor_target_parser_resync(parser, byte);
        }
        return false;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index >= APP_PI_TARGET_FRAME_SIZE) {
        accepted = pi_motor_target_accept_frame(parser, latest, now_ms);
        parser->index = 0U;
    }
    return accepted;
}

static void pi_motor_target_drain_uart0(void)
{
    uint8_t ignored;

    NVIC_DisableIRQ(VISION_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(VISION_UART_INST_INT_IRQN);

    while (vision_uart_read(&ignored)) {
    }
    while (!DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
        (void) DL_UART_Main_receiveData(VISION_UART_INST);
    }
}

static void pi_motor_target_apply_timeout(PiLatestMotorTarget *latest,
                                          uint32_t now_ms)
{
    if (!latest->has_rx || latest->timed_out) {
        return;
    }
    if ((uint32_t) (now_ms - latest->last_valid_rx_ms) <=
        APP_PI_TARGET_TIMEOUT_MS) {
        return;
    }

    latest->timed_out = true;
    latest->disable_requested = false;
    if (!latest->valid || latest->target_offset_mdeg != 0) {
        latest->target_offset_mdeg = 0;
        latest->valid = true;
        latest->version++;
    }
}

static void pi_motor_target_stream_test_run(void)
{
    PiMotorTargetParser parser;
    PiLatestMotorTarget latest;
    RodMotorControl motor;

    pi_motor_target_parser_init(&parser);
    pi_latest_motor_target_init(&latest);
    rod_motor_control_init(&motor);

    /* Disable the known-problematic UART0 RX ISR before any blocking startup. */
    pi_motor_target_drain_uart0();

    /*
     * Before reset, manually place the rod at the physical 0-degree reference.
     * This existing routine clears the Emm position and moves to -28.1 degrees.
     */
    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        for (;;) {
            __WFI();
        }
    }

    /* Ignore bytes accumulated while the blocking startup sequence was active. */
    pi_motor_target_drain_uart0();

    for (;;) {
        uint32_t now_ms = bsp_time_ms();
        uint32_t processed = 0U;

        emm_v5_poll(now_ms);

        while ((processed < APP_PI_TARGET_RX_BUDGET_BYTES) &&
               !DL_UART_Main_isRXFIFOEmpty(VISION_UART_INST)) {
            uint8_t byte = (uint8_t) DL_UART_Main_receiveData(VISION_UART_INST);
            (void) pi_motor_target_parser_push(&parser, &latest, byte, now_ms);
            processed++;
        }

        pi_motor_target_apply_timeout(&latest, now_ms);

        if (latest.disable_requested) {
            if (motor.enabled) {
                (void) rod_motor_control_set_enabled(&motor, false);
            }
            latest.applied_version = latest.version;
        } else if (latest.valid &&
                   (latest.version != latest.applied_version) &&
                   !rod_motor_control_is_busy(&motor, now_ms)) {
            int32_t effective_target_mdeg =
                APP_PI_TARGET_DIRECTION_SIGN * latest.target_offset_mdeg;

            if (rod_motor_control_set_target_offset_mdeg(
                    &motor, effective_target_mdeg, now_ms)) {
                latest.applied_version = latest.version;
            }
        }

        /* SysTick wakes the CPU every millisecond; UART0 itself stays polled. */
        __WFI();
    }
}

#endif
