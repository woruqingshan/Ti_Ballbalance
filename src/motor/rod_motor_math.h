#ifndef ROD_MOTOR_MATH_H
#define ROD_MOTOR_MATH_H

#include "app/app_config.h"
#include <stdint.h>

static inline int32_t rod_motor_clamp_i32(int32_t value,
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

static inline int32_t rod_motor_angle_mdeg_to_pulse(int32_t angle_mdeg)
{
    int64_t numerator = (int64_t) angle_mdeg *
                        (int64_t) APP_ROD_MOTOR_PULSES_PER_REV;

    if (numerator >= 0) {
        numerator += 180000LL;
    } else {
        numerator -= 180000LL;
    }
    return (int32_t) (numerator / 360000LL);
}

static inline int32_t rod_motor_pulse_to_angle_mdeg(int32_t pulse)
{
    int64_t numerator = (int64_t) pulse * 360000LL;

    if (numerator >= 0) {
        numerator += APP_ROD_MOTOR_PULSES_PER_REV / 2L;
    } else {
        numerator -= APP_ROD_MOTOR_PULSES_PER_REV / 2L;
    }
    return (int32_t) (numerator / APP_ROD_MOTOR_PULSES_PER_REV);
}

static inline int32_t rod_motor_raw_units_to_angle_mdeg(int32_t raw_units)
{
    int64_t numerator = (int64_t) raw_units * 360000LL;

    if (numerator >= 0) {
        numerator += APP_EMM_POSITION_UNITS_PER_REVOLUTION / 2LL;
    } else {
        numerator -= APP_EMM_POSITION_UNITS_PER_REVOLUTION / 2LL;
    }
    return (int32_t) (numerator /
                      APP_EMM_POSITION_UNITS_PER_REVOLUTION);
}

#endif
