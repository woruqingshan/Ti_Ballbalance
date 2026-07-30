#ifndef BALL_POSITION_CONTROLLER_H
#define BALL_POSITION_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t target_position_centi_cm;
    int32_t position_kp_mdeg_per_cm;
    int32_t velocity_kd_mdeg_per_cm_s;
    int32_t maximum_offset_mdeg;
    int32_t maximum_slew_mdeg_per_s;
    int32_t motor_sign;
} BallPositionControllerConfig;

typedef struct {
    int32_t error_centi_cm;
    int32_t raw_output_mdeg;
    int32_t amplitude_limited_mdeg;
    int32_t target_offset_mdeg;
    uint32_t last_update_ms;
    bool has_previous;
} BallPositionController;

static int32_t ball_position_clamp_i32(int32_t value,
                                      int32_t minimum,
                                      int32_t maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t ball_position_divide_rounded_100(int64_t value)
{
    if (value >= 0) {
        value += 50;
    } else {
        value -= 50;
    }
    return (int32_t) (value / 100);
}

static void ball_position_controller_init(BallPositionController *controller)
{
    if (controller == 0) {
        return;
    }
    controller->error_centi_cm = 0;
    controller->raw_output_mdeg = 0;
    controller->amplitude_limited_mdeg = 0;
    controller->target_offset_mdeg = 0;
    controller->last_update_ms = 0U;
    controller->has_previous = false;
}

static void ball_position_controller_reset(BallPositionController *controller)
{
    ball_position_controller_init(controller);
}

/*
 * Fixed-point PD controller. Gains are expressed directly in motor
 * millidegrees per physical unit, avoiding float operations on MSPM0.
 *
 * position and velocity use 0.01 cm and 0.01 cm/s units, so the weighted sum
 * is divided by 100 to obtain millidegrees.
 */
static int32_t ball_position_controller_update(
    BallPositionController *controller,
    const BallPositionControllerConfig *config,
    int16_t position_centi_cm,
    int16_t velocity_centi_cm_s,
    uint32_t now_ms,
    uint32_t default_dt_ms)
{
    int64_t weighted;
    int32_t raw;
    int32_t limited;
    int32_t previous;
    int32_t maximum_delta;
    int32_t output;
    uint32_t dt_ms;

    if ((controller == 0) || (config == 0)) {
        return 0;
    }

    controller->error_centi_cm =
        (int32_t) config->target_position_centi_cm -
        (int32_t) position_centi_cm;

    weighted =
        (int64_t) config->position_kp_mdeg_per_cm *
            (int64_t) controller->error_centi_cm -
        (int64_t) config->velocity_kd_mdeg_per_cm_s *
            (int64_t) velocity_centi_cm_s;
    weighted *= (int64_t) config->motor_sign;
    raw = ball_position_divide_rounded_100(weighted);
    limited = ball_position_clamp_i32(
        raw,
        -config->maximum_offset_mdeg,
        config->maximum_offset_mdeg);

    if (controller->has_previous) {
        dt_ms = (uint32_t) (now_ms - controller->last_update_ms);
    } else {
        dt_ms = default_dt_ms;
    }
    if (dt_ms == 0U) {
        dt_ms = 1U;
    }
    if (dt_ms > 250U) {
        dt_ms = 250U;
    }

    maximum_delta = (int32_t) (
        ((int64_t) config->maximum_slew_mdeg_per_s *
         (int64_t) dt_ms) /
        1000LL);
    if ((maximum_delta < 1) &&
        (config->maximum_slew_mdeg_per_s > 0)) {
        maximum_delta = 1;
    }

    previous = controller->has_previous ?
        controller->target_offset_mdeg : 0;
    output = ball_position_clamp_i32(
        limited,
        previous - maximum_delta,
        previous + maximum_delta);

    controller->raw_output_mdeg = raw;
    controller->amplitude_limited_mdeg = limited;
    controller->target_offset_mdeg = output;
    controller->last_update_ms = now_ms;
    controller->has_previous = true;
    return output;
}

#endif
