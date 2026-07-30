#ifndef BALL_STATE_LINK_TEST_H
#define BALL_STATE_LINK_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_link.h"
#include "control/vision_control_preview.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "protocol/pi_ti_messages.h"
#include "test_modes/pi_ti_manual_link_test.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

static void ball_state_send_preview(const VisionControlPreview *preview,
                                    const BallState *ball,
                                    bool has_ball,
                                    const VisionLink *link,
                                    bool motor_output_compiled,
                                    bool motor_output_active,
                                    uint32_t now_ms)
{
    uint8_t payload[PI_TI_BALL_PREVIEW_SIZE];
    uint8_t flags = 0U;
    uint8_t invalid_reason = has_ball ? ball->invalid_reason : 0xFFU;
    int16_t position = 0;
    int16_t velocity = 0;
    uint32_t frame_id = 0U;

    if (has_ball) {
        flags |= PI_PREVIEW_FLAG_HAS_BALL;
        frame_id = ball->frame_id;
        position = vision_preview_quantize_100(ball->position_cm);
        velocity = vision_preview_quantize_100(ball->velocity_cm_s);
        if (ball->predicted) {
            flags |= PI_PREVIEW_FLAG_PREDICTED;
        }
        if (ball->found) {
            flags |= PI_PREVIEW_FLAG_FOUND;
        }
    }
    if (preview->valid) {
        flags |= PI_PREVIEW_FLAG_CONTROL_VALID;
    }
    if (preview->armed) {
        flags |= PI_PREVIEW_FLAG_ARMED;
    }
    if (motor_output_compiled) {
        flags |= PI_PREVIEW_FLAG_OUTPUT_COMPILED;
    }
    if (motor_output_active) {
        flags |= PI_PREVIEW_FLAG_OUTPUT_ACTIVE;
    }

    pi_ti_pack_ball_preview(
        payload,
        flags,
        invalid_reason,
        frame_id,
        preview->age_ms,
        position,
        velocity,
        vision_preview_quantize_1000(preview->raw_control),
        vision_preview_quantize_1000(preview->limited_control),
        preview->target_offset_mdeg,
        preview->target_physical_mdeg,
        link->sequence_gaps);
    vision_link_send_frame(MSG_BALL_PREVIEW, payload, sizeof(payload), now_ms);
}

static void ball_state_link_test_run(bool simulation_mode)
{
    VisionLink link;
    RodMotorControl motor;
    VisionControlPreview preview;
    BallState ball;
    bool has_ball = false;
    bool motor_output_active = false;
    uint32_t last_control_ms = 0U;
    uint32_t last_motor_ms = 0U;
    uint32_t last_preview_ms = 0U;
    uint32_t last_status_ms = 0U;
    uint32_t last_heartbeat_ms = 0U;
    uint32_t last_stats_ms = 0U;
    uint32_t last_token = 0U;
    bool has_last_token = false;
    uint8_t last_command = 0U;
    uint8_t last_result = PI_ACK_OK;
    int32_t last_detail = 0;
    const bool motor_output_compiled = simulation_mode &&
        (APP_PHASE6_MOTOR_OUTPUT_ENABLED != 0U);

    vision_link_init(&link);
    rod_motor_control_init(&motor);
    vision_control_preview_init(&preview);
    (void) emm_v5_enable(false);

    for (;;) {
        uint32_t now_ms = bsp_time_ms();
        uint8_t command;
        uint8_t argument;
        int16_t value;
        uint32_t token;
        bool link_alive;

        emm_v5_poll(now_ms);
        vision_link_poll(&link, now_ms);
        has_ball = vision_link_get_latest(&link, &ball);

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

            case PI_CMD_ARM_PREVIEW:
                preview.armed = true;
                ball_controller_reset(&preview.controller);
                break;

            case PI_CMD_DISARM_PREVIEW:
                preview.armed = false;
                motor_output_active = false;
                ball_controller_reset(&preview.controller);
                if (motor.enabled && motor.startup_complete &&
                    !rod_motor_control_is_busy(&motor, now_ms)) {
                    (void) rod_motor_control_return_horizontal(&motor, now_ms);
                }
                break;

            case PI_CMD_SET_BALL_TARGET_CENTI_CM:
                if ((value < -1000) || (value > 1000)) {
                    result = PI_ACK_OUT_OF_RANGE;
                    detail = value;
                } else {
                    preview.target_cm = (float) value * 0.01f;
                    detail = value;
                }
                break;

            case PI_CMD_STARTUP_HORIZONTAL:
                if (!motor_output_compiled) {
                    result = PI_ACK_UNSUPPORTED;
                } else if (motor.startup_complete) {
                    result = PI_ACK_BAD_STATE;
                } else {
                    pi_ti_send_ack_frame(command, PI_ACK_ACCEPTED,
                                         APP_ROD_HORIZONTAL_MDEG,
                                         token, now_ms);
                    if (!rod_motor_control_startup_to_horizontal(&motor)) {
                        result = PI_ACK_BAD_STATE;
                    } else {
                        detail = motor.commanded_physical_mdeg;
                        link.last_rx_timestamp_ms = bsp_time_ms();
                    }
                    now_ms = bsp_time_ms();
                }
                break;

            case PI_CMD_ENABLE_MOTOR:
                if (!motor_output_compiled) {
                    result = PI_ACK_UNSUPPORTED;
                } else if (!motor.startup_complete) {
                    result = PI_ACK_BAD_STATE;
                } else if (!rod_motor_control_set_enabled(&motor, true)) {
                    result = PI_ACK_BAD_STATE;
                } else {
                    motor_output_active = true;
                }
                break;

            case PI_CMD_DISABLE_MOTOR:
            case PI_CMD_EMERGENCY_DISABLE:
                motor_output_active = false;
                if (!rod_motor_control_set_enabled(&motor, false)) {
                    result = PI_ACK_BAD_STATE;
                }
                break;

            case PI_CMD_RETURN_HORIZONTAL:
                if (!motor_output_compiled) {
                    result = PI_ACK_UNSUPPORTED;
                } else if (rod_motor_control_is_busy(&motor, now_ms)) {
                    result = PI_ACK_BUSY;
                } else if (!rod_motor_control_return_horizontal(&motor,
                                                                 now_ms)) {
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
        }

        link_alive = link.rx_frames != 0U &&
            ((uint32_t) (now_ms - link.last_rx_timestamp_ms) <=
             APP_PI_LINK_TIMEOUT_MS);

        if (!link_alive) {
            preview.armed = false;
            motor_output_active = false;
            ball_controller_reset(&preview.controller);
            if (motor.enabled) {
                (void) rod_motor_control_set_enabled(&motor, false);
            }
        }

        if ((uint32_t) (now_ms - last_control_ms) >=
            APP_CONTROL_PERIOD_MS) {
            last_control_ms = now_ms;
            vision_control_preview_update(&preview, &ball, has_ball, now_ms);
            if (!preview.valid) {
                motor_output_active = false;
            }
        }

        if (motor_output_compiled && motor_output_active &&
            preview.armed && preview.valid &&
            (uint32_t) (now_ms - last_motor_ms) >=
            APP_PHASE6_MOTOR_UPDATE_PERIOD_MS) {
            last_motor_ms = now_ms;
            if (!rod_motor_control_is_busy(&motor, now_ms)) {
                (void) rod_motor_control_set_target_offset_mdeg(
                    &motor, preview.target_offset_mdeg, now_ms);
            }
        }

        if ((uint32_t) (now_ms - last_preview_ms) >=
            APP_PI_PREVIEW_PERIOD_MS) {
            last_preview_ms = now_ms;
            ball_state_send_preview(&preview, &ball, has_ball, &link,
                                    motor_output_compiled,
                                    motor_output_active,
                                    now_ms);
        }

        if ((uint32_t) (now_ms - last_status_ms) >=
            APP_PI_STATUS_PERIOD_MS) {
            last_status_ms = now_ms;
            pi_ti_send_motor_status_frame(
                &motor, &link,
                preview.armed ? 1U : 0U,
                last_result,
                preview.armed,
                motor_output_compiled,
                motor_output_active,
                preview.valid ? 0U : 1U,
                now_ms);
        }

        if ((uint32_t) (now_ms - last_heartbeat_ms) >=
            APP_PI_HEARTBEAT_PERIOD_MS) {
            uint8_t payload[8];
            last_heartbeat_ms = now_ms;
            protocol_put_u32_le(&payload[0], APP_MODE);
            protocol_put_u32_le(&payload[4], now_ms);
            vision_link_send_frame(MSG_HEARTBEAT, payload,
                                   sizeof(payload), now_ms);
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
