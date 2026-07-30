#ifndef PI_TI_MANUAL_LINK_TEST_H
#define PI_TI_MANUAL_LINK_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_link.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "protocol/pi_ti_messages.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

static void pi_ti_send_ack_frame(uint8_t command,
                                 uint8_t result,
                                 int32_t detail,
                                 uint32_t token,
                                 uint32_t now_ms)
{
    uint8_t payload[PI_TI_ACK_PAYLOAD_SIZE];
    pi_ti_pack_ack(payload, command, result, detail, token);
    vision_link_send_frame(MSG_COMMAND_ACK, payload, sizeof(payload), now_ms);
}

static uint8_t pi_ti_motor_flags(const RodMotorControl *motor,
                                 bool link_alive,
                                 bool preview_armed,
                                 bool output_compiled,
                                 bool output_active,
                                 uint32_t now_ms)
{
    uint8_t flags = 0U;
    if (motor->enabled) {
        flags |= PI_MOTOR_FLAG_ENABLED;
    }
    if (motor->startup_complete) {
        flags |= PI_MOTOR_FLAG_STARTUP_COMPLETE;
    }
    if (rod_motor_control_is_busy(motor, now_ms)) {
        flags |= PI_MOTOR_FLAG_BUSY;
    }
    if (motor->soft_limit_hit) {
        flags |= PI_MOTOR_FLAG_SOFT_LIMIT;
    }
    if (link_alive) {
        flags |= PI_MOTOR_FLAG_LINK_ALIVE;
    }
    if (preview_armed) {
        flags |= PI_MOTOR_FLAG_PREVIEW_ARMED;
    }
    if (output_compiled) {
        flags |= PI_MOTOR_FLAG_OUTPUT_COMPILED;
    }
    if (output_active) {
        flags |= PI_MOTOR_FLAG_OUTPUT_ACTIVE;
    }
    return flags;
}

static void pi_ti_send_motor_status_frame(const RodMotorControl *motor,
                                          const VisionLink *link,
                                          uint8_t control_state,
                                          uint8_t last_ack,
                                          bool preview_armed,
                                          bool output_compiled,
                                          bool output_active,
                                          uint32_t faults,
                                          uint32_t now_ms)
{
    uint8_t payload[PI_TI_MOTOR_STATUS_SIZE];
    bool link_alive = link->rx_frames != 0U &&
        ((uint32_t) (now_ms - link->last_rx_timestamp_ms) <=
         APP_PI_LINK_TIMEOUT_MS);
    uint32_t age_ms = link->rx_frames == 0U
        ? 0xFFFFFFFFU
        : (uint32_t) (now_ms - link->last_rx_timestamp_ms);

    pi_ti_pack_motor_status(
        payload,
        (uint8_t) APP_MODE,
        pi_ti_motor_flags(motor, link_alive, preview_armed,
                          output_compiled, output_active, now_ms),
        control_state,
        last_ack,
        now_ms,
        age_ms,
        motor->target_physical_mdeg,
        motor->commanded_physical_mdeg,
        motor->target_pulse,
        motor->commanded_pulse,
        faults);
    vision_link_send_frame(MSG_MOTOR_STATUS, payload, sizeof(payload), now_ms);
}

static void pi_ti_send_link_stats_frame(const VisionLink *link,
                                        uint32_t now_ms)
{
    uint8_t payload[PI_TI_LINK_STATS_SIZE];
    pi_ti_pack_link_stats(payload,
                          link->rx_frames,
                          link->rx_sequence_gaps,
                          link->packets_ok,
                          link->sequence_gaps,
                          link->parser.crc_errors,
                          link->parser.length_errors,
                          link->parser.version_errors,
                          vision_uart_rx_overflow());
    vision_link_send_frame(MSG_LINK_STATS, payload, sizeof(payload), now_ms);
}

static void pi_ti_send_heartbeat_frame(uint32_t now_ms)
{
    uint8_t payload[8];

    protocol_put_u32_le(&payload[0], APP_MODE);
    protocol_put_u32_le(&payload[4], now_ms);
    vision_link_send_frame(MSG_HEARTBEAT,
                           payload,
                           sizeof(payload),
                           now_ms);
}

static bool pi_ti_duplicate_token(uint32_t token,
                                  bool has_last_token,
                                  uint32_t last_token)
{
    return has_last_token && token == last_token;
}

static void pi_ti_manual_link_test_run(void)
{
    VisionLink link;
    RodMotorControl motor;
    uint32_t last_status_ms = 0U;
    uint32_t last_heartbeat_ms = 0U;
    uint32_t last_stats_ms = 0U;
    uint32_t last_token = 0U;
    bool has_last_token = false;
    uint8_t last_command = 0U;
    uint8_t last_result = PI_ACK_OK;
    int32_t last_detail = 0;

    /*
     * Publish on UART0 before touching the motor-side UART. Mode 11 must be
     * observable immediately after reset, even when UART1/Emm is disconnected.
     * No motor command is issued automatically; STARTUP_HORIZONTAL remains the
     * only entry into the validated disable/clear/enable/startup sequence.
     */
    vision_link_init(&link);
    rod_motor_control_init(&motor);

    {
        uint32_t boot_ms = bsp_time_ms();
        pi_ti_send_heartbeat_frame(boot_ms);
        pi_ti_send_motor_status_frame(&motor, &link, 0U, last_result,
                                      false, false, false, 0U, boot_ms);
        pi_ti_send_link_stats_frame(&link, boot_ms);
        last_status_ms = boot_ms;
        last_heartbeat_ms = boot_ms;
        last_stats_ms = boot_ms;
    }

    for (;;) {
        uint32_t now_ms = bsp_time_ms();
        uint8_t command;
        uint8_t argument;
        int16_t value;
        uint32_t token;
        bool link_alive;

        emm_v5_poll(now_ms);
        vision_link_poll(&link, now_ms);

        while (vision_link_take_command_ex(&link, &command, &argument,
                                           &value, &token)) {
            uint8_t result = PI_ACK_OK;
            int32_t detail = 0;
            (void) argument;

            if (pi_ti_duplicate_token(token, has_last_token, last_token)) {
                if (command == last_command) {
                    pi_ti_send_ack_frame(last_command, last_result,
                                         last_detail, token, now_ms);
                } else {
                    pi_ti_send_ack_frame(command, PI_ACK_BAD_STATE,
                                         (int32_t) last_command, token, now_ms);
                }
                continue;
            }

            switch (command) {
            case PI_CMD_HELLO:
            case PI_CMD_GET_STATUS:
                detail = PI_TI_PROTOCOL_PROFILE_VERSION;
                break;

            case PI_CMD_STARTUP_HORIZONTAL:
                if (motor.startup_complete) {
                    result = PI_ACK_BAD_STATE;
                    detail = motor.commanded_physical_mdeg;
                    break;
                }
                pi_ti_send_ack_frame(command, PI_ACK_ACCEPTED,
                                     APP_ROD_HORIZONTAL_MDEG,
                                     token, now_ms);
                if (!rod_motor_control_startup_to_horizontal(&motor)) {
                    result = PI_ACK_BAD_STATE;
                } else {
                    result = PI_ACK_OK;
                    detail = motor.commanded_physical_mdeg;
                    link.last_rx_timestamp_ms = bsp_time_ms();
                }
                now_ms = bsp_time_ms();
                break;

            case PI_CMD_SET_OFFSET_MDEG:
                if ((value < -APP_PI_MANUAL_MAX_OFFSET_MDEG) ||
                    (value > APP_PI_MANUAL_MAX_OFFSET_MDEG)) {
                    result = PI_ACK_OUT_OF_RANGE;
                    detail = value;
                } else if (rod_motor_control_is_busy(&motor, now_ms)) {
                    result = PI_ACK_BUSY;
                } else if (!rod_motor_control_set_target_offset_mdeg(
                               &motor, (int32_t) value, now_ms)) {
                    result = motor.enabled
                        ? PI_ACK_BAD_STATE : PI_ACK_DISABLED;
                } else {
                    detail = motor.target_physical_mdeg;
                }
                break;

            case PI_CMD_RETURN_HORIZONTAL:
                if (rod_motor_control_is_busy(&motor, now_ms)) {
                    result = PI_ACK_BUSY;
                } else if (!rod_motor_control_return_horizontal(&motor,
                                                                 now_ms)) {
                    result = motor.enabled
                        ? PI_ACK_BAD_STATE : PI_ACK_DISABLED;
                } else {
                    detail = motor.target_physical_mdeg;
                }
                break;

            case PI_CMD_ENABLE_MOTOR:
                if (!motor.startup_complete) {
                    result = PI_ACK_BAD_STATE;
                } else if (!rod_motor_control_set_enabled(&motor, true)) {
                    result = PI_ACK_BAD_STATE;
                }
                break;

            case PI_CMD_DISABLE_MOTOR:
            case PI_CMD_EMERGENCY_DISABLE:
                if (!rod_motor_control_set_enabled(&motor, false)) {
                    result = PI_ACK_BAD_STATE;
                }
                break;

            case PI_CMD_CLEAR_LINK_STATS:
                link.rx_frames = 0U;
                link.rx_sequence_gaps = 0U;
                link.packets_ok = 0U;
                link.sequence_gaps = 0U;
                link.parser.crc_errors = 0U;
                link.parser.length_errors = 0U;
                link.parser.version_errors = 0U;
                break;

            default:
                result = PI_ACK_UNSUPPORTED;
                detail = command;
                break;
            }

            last_token = token;
            has_last_token = true;
            last_command = command;
            last_result = result;
            last_detail = detail;
            pi_ti_send_ack_frame(command, result, detail, token, now_ms);
            pi_ti_send_motor_status_frame(&motor, &link, 0U, result,
                                          false, false, false, 0U, now_ms);
        }

        link_alive = link.rx_frames != 0U &&
            ((uint32_t) (now_ms - link.last_rx_timestamp_ms) <=
             APP_PI_LINK_TIMEOUT_MS);
        if (!link_alive && motor.enabled) {
            (void) rod_motor_control_set_enabled(&motor, false);
        }

        if ((uint32_t) (now_ms - last_status_ms) >=
            APP_PI_STATUS_PERIOD_MS) {
            last_status_ms = now_ms;
            pi_ti_send_motor_status_frame(&motor, &link, 0U, last_result,
                                          false, false, false, 0U, now_ms);
        }
        if ((uint32_t) (now_ms - last_heartbeat_ms) >=
            APP_PI_HEARTBEAT_PERIOD_MS) {
            last_heartbeat_ms = now_ms;
            pi_ti_send_heartbeat_frame(now_ms);
        }
        if ((uint32_t) (now_ms - last_stats_ms) >=
            APP_PI_LINK_STATS_PERIOD_MS) {
            last_stats_ms = now_ms;
            pi_ti_send_link_stats_frame(&link, now_ms);
        }
        __WFI();
    }
}

#endif
