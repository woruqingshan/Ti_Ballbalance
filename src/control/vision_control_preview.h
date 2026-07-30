#ifndef VISION_CONTROL_PREVIEW_H
#define VISION_CONTROL_PREVIEW_H

#include "app/app_config.h"
#include "common/common.h"
#include "control/ball_controller.h"
#include "control/ball_state.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    BallController controller;
    float target_cm;
    float raw_control;
    float limited_control;
    int32_t target_offset_mdeg;
    int32_t target_physical_mdeg;
    uint32_t age_ms;
    bool armed;
    bool valid;
} VisionControlPreview;

static int16_t vision_preview_quantize_100(float value)
{
    float scaled = value * 100.0f;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t) ((scaled >= 0.0f) ? scaled + 0.5f : scaled - 0.5f);
}

static int16_t vision_preview_quantize_1000(float value)
{
    float scaled = value * 1000.0f;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    } else if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t) ((scaled >= 0.0f) ? scaled + 0.5f : scaled - 0.5f);
}

static void vision_control_preview_init(VisionControlPreview *preview)
{
    ball_controller_init(&preview->controller,
                         APP_PREVIEW_POSITION_KP,
                         APP_PREVIEW_VELOCITY_KD,
                         APP_PREVIEW_CONTROL_MAX,
                         APP_PREVIEW_CONTROL_SLEW_PER_UPDATE);
    preview->target_cm = 0.0f;
    preview->raw_control = 0.0f;
    preview->limited_control = 0.0f;
    preview->target_offset_mdeg = 0;
    preview->target_physical_mdeg = APP_ROD_HORIZONTAL_MDEG;
    preview->age_ms = 0xFFFFFFFFU;
    preview->armed = false;
    preview->valid = false;
}

static bool vision_control_preview_measurement_valid(
    const BallState *ball,
    bool has_ball,
    uint32_t now_ms,
    uint32_t *age_ms_out)
{
    uint32_t age_ms;
    uint16_t confidence_milli;

    if (!has_ball) {
        if (age_ms_out != NULL) {
            *age_ms_out = 0xFFFFFFFFU;
        }
        return false;
    }

    age_ms = (uint32_t) (now_ms - ball->received_timestamp_ms);
    if (age_ms <= (0xFFFFFFFFU - (uint32_t) ball->vision_latency_ms)) {
        age_ms += (uint32_t) ball->vision_latency_ms;
    } else {
        age_ms = 0xFFFFFFFFU;
    }
    if (ball->confidence <= 0.0f) {
        confidence_milli = 0U;
    } else if (ball->confidence >= 1.0f) {
        confidence_milli = 1000U;
    } else {
        confidence_milli = (uint16_t) (ball->confidence * 1000.0f + 0.5f);
    }
    if (age_ms_out != NULL) {
        *age_ms_out = age_ms;
    }

    if (!ball->valid) {
        return false;
    }
    if (!ball->found) {
#if APP_VISION_ALLOW_PREDICTED != 0
        if (!ball->predicted) {
            return false;
        }
#else
        return false;
#endif
    }
    if (age_ms > APP_VISION_MAX_AGE_MS) {
        return false;
    }
    if (confidence_milli < APP_VISION_MIN_CONFIDENCE_MILLI) {
        return false;
    }
#if APP_VISION_ALLOW_PREDICTED == 0
    if (ball->predicted) {
        return false;
    }
#endif
    return true;
}

static void vision_control_preview_update(VisionControlPreview *preview,
                                          const BallState *ball,
                                          bool has_ball,
                                          uint32_t now_ms)
{
    int32_t offset_mdeg;

    preview->valid = vision_control_preview_measurement_valid(
        ball, has_ball, now_ms, &preview->age_ms);

    if (!preview->armed || !preview->valid) {
        ball_controller_reset(&preview->controller);
        preview->raw_control = 0.0f;
        preview->limited_control = 0.0f;
        preview->target_offset_mdeg = 0;
        preview->target_physical_mdeg = APP_ROD_HORIZONTAL_MDEG;
        return;
    }

    preview->limited_control = ball_controller_update(
        &preview->controller, preview->target_cm, ball);
    preview->raw_control = preview->controller.raw;

    offset_mdeg = (int32_t) (
        preview->limited_control *
        (float) APP_PREVIEW_MDEG_PER_CONTROL_UNIT);
    offset_mdeg *= APP_BALL_CONTROL_MOTOR_SIGN;
    offset_mdeg = bbc_clampi32(offset_mdeg,
                               -APP_PREVIEW_MAX_OFFSET_MDEG,
                               APP_PREVIEW_MAX_OFFSET_MDEG);
    preview->target_offset_mdeg = offset_mdeg;
    preview->target_physical_mdeg = APP_ROD_HORIZONTAL_MDEG + offset_mdeg;
}

#endif
