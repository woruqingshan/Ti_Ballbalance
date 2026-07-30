#ifndef ROD_MOTOR_CONTROL_H
#define ROD_MOTOR_CONTROL_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_math.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t target_physical_mdeg;
    int32_t commanded_physical_mdeg;
    int32_t target_pulse;
    int32_t commanded_pulse;
    uint32_t busy_until_ms;
    bool enabled;
    bool startup_complete;
    bool soft_limit_hit;
} RodMotorControl;

static uint32_t rod_motor_abs_u32_from_i32(int32_t value)
{
    return (uint32_t) ((value < 0) ? -(int64_t) value : value);
}

static void rod_motor_wait_ms(uint32_t delay_ms)
{
    uint32_t start_ms = bsp_time_ms();

    while ((uint32_t) (bsp_time_ms() - start_ms) < delay_ms) {
        emm_v5_poll(bsp_time_ms());
        __WFI();
    }
}

static uint32_t rod_motor_estimate_motion_ms(int32_t delta_pulse,
                                             uint16_t speed_rpm)
{
    uint64_t motion_ms;
    uint32_t pulse_magnitude = rod_motor_abs_u32_from_i32(delta_pulse);

    if ((pulse_magnitude == 0U) || (speed_rpm == 0U)) {
        return APP_ROD_COMMAND_MARGIN_MS;
    }

    motion_ms = ((uint64_t) pulse_magnitude * 60000ULL) /
                ((uint64_t) speed_rpm *
                 (uint64_t) APP_ROD_MOTOR_PULSES_PER_REV);
    motion_ms += APP_ROD_COMMAND_MARGIN_MS;
    if (motion_ms > 0xFFFFFFFFULL) {
        return 0xFFFFFFFFU;
    }
    return (uint32_t) motion_ms;
}

static void rod_motor_control_init(RodMotorControl *motor)
{
    if (motor == NULL) {
        return;
    }

    motor->target_physical_mdeg = 0;
    motor->commanded_physical_mdeg = 0;
    motor->target_pulse = 0;
    motor->commanded_pulse = 0;
    motor->busy_until_ms = 0U;
    motor->enabled = false;
    motor->startup_complete = false;
    motor->soft_limit_hit = false;
}

static bool rod_motor_control_is_busy(const RodMotorControl *motor,
                                      uint32_t now_ms)
{
    if (motor == NULL) {
        return false;
    }
    return (int32_t) (motor->busy_until_ms - now_ms) > 0;
}

static bool rod_motor_control_set_enabled(RodMotorControl *motor,
                                          bool enabled)
{
    if (motor == NULL) {
        return false;
    }

    if (!emm_v5_enable(enabled)) {
        return false;
    }
    motor->enabled = enabled;
    return true;
}

static bool rod_motor_control_startup_to_horizontal(RodMotorControl *motor)
{
    int32_t horizontal_pulse;

    if (motor == NULL) {
        return false;
    }

    rod_motor_control_init(motor);
    rod_motor_wait_ms(APP_ROD_BOOT_DELAY_MS);

    if (!rod_motor_control_set_enabled(motor, false)) {
        return false;
    }
    rod_motor_wait_ms(APP_ROD_COMMAND_GAP_MS);

    if (!emm_v5_clear_current_position()) {
        return false;
    }
    rod_motor_wait_ms(APP_ROD_COMMAND_GAP_MS);

    if (!rod_motor_control_set_enabled(motor, true)) {
        return false;
    }
    rod_motor_wait_ms(APP_ROD_COMMAND_GAP_MS);

    horizontal_pulse = rod_motor_angle_mdeg_to_pulse(
        APP_ROD_HORIZONTAL_MDEG);
    if (!emm_v5_move_relative(horizontal_pulse,
                              APP_ROD_MANUAL_SPEED_RPM,
                              APP_ROD_MANUAL_ACCELERATION)) {
        return false;
    }

    motor->target_pulse = horizontal_pulse;
    motor->commanded_pulse = horizontal_pulse;
    motor->target_physical_mdeg = rod_motor_pulse_to_angle_mdeg(
        horizontal_pulse);
    motor->commanded_physical_mdeg = motor->target_physical_mdeg;
    motor->busy_until_ms = bsp_time_ms() + APP_ROD_STARTUP_SETTLE_MS;

    rod_motor_wait_ms(APP_ROD_STARTUP_SETTLE_MS);
    motor->busy_until_ms = bsp_time_ms();
    motor->startup_complete = true;
    return true;
}

static bool rod_motor_control_set_target_physical_mdeg(
    RodMotorControl *motor,
    int32_t target_physical_mdeg,
    uint32_t now_ms)
{
    int32_t target_pulse;
    int32_t delta_pulse;

    if ((motor == NULL) || !motor->startup_complete || !motor->enabled) {
        return false;
    }
    if (rod_motor_control_is_busy(motor, now_ms)) {
        return false;
    }

    if ((target_physical_mdeg < APP_ROD_CONTROL_MIN_MDEG) ||
        (target_physical_mdeg > APP_ROD_CONTROL_MAX_MDEG)) {
        motor->soft_limit_hit = true;
        return false;
    }

    target_physical_mdeg = rod_motor_clamp_i32(
        target_physical_mdeg,
        APP_ROD_PHYSICAL_MIN_MDEG,
        APP_ROD_PHYSICAL_MAX_MDEG);
    target_pulse = rod_motor_angle_mdeg_to_pulse(target_physical_mdeg);
    delta_pulse = target_pulse - motor->commanded_pulse;

    motor->target_physical_mdeg = target_physical_mdeg;
    motor->target_pulse = target_pulse;
    motor->soft_limit_hit = false;

    if (delta_pulse == 0) {
        motor->commanded_physical_mdeg = rod_motor_pulse_to_angle_mdeg(
            motor->commanded_pulse);
        return true;
    }

    if (!emm_v5_move_relative(delta_pulse,
                              APP_ROD_MANUAL_SPEED_RPM,
                              APP_ROD_MANUAL_ACCELERATION)) {
        return false;
    }

    motor->commanded_pulse = target_pulse;
    motor->commanded_physical_mdeg = rod_motor_pulse_to_angle_mdeg(
        motor->commanded_pulse);
    motor->busy_until_ms = now_ms + rod_motor_estimate_motion_ms(
        delta_pulse,
        APP_ROD_MANUAL_SPEED_RPM);
    return true;
}

static bool rod_motor_control_set_target_offset_mdeg(
    RodMotorControl *motor,
    int32_t offset_from_horizontal_mdeg,
    uint32_t now_ms)
{
    return rod_motor_control_set_target_physical_mdeg(
        motor,
        APP_ROD_HORIZONTAL_MDEG + offset_from_horizontal_mdeg,
        now_ms);
}

static bool rod_motor_control_return_horizontal(RodMotorControl *motor,
                                                uint32_t now_ms)
{
    return rod_motor_control_set_target_physical_mdeg(
        motor,
        APP_ROD_HORIZONTAL_MDEG,
        now_ms);
}

#endif
