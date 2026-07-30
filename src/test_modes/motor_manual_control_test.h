#ifndef MOTOR_MANUAL_CONTROL_TEST_H
#define MOTOR_MANUAL_CONTROL_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "motor/rod_motor_math.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/* Header-only test mode: no Keil project-file change is required. */

static void manual_control_print_help(void)
{
    test_console_write_line("");
    test_console_write_line("=== PHASE 3: ROD MOTOR MANUAL CONTROL ===");
    test_console_write_line("Before reset: manually place mechanism at physical 0 deg.");
    test_console_write_line("Boot automatically moves to horizontal -28.1 deg.");
    test_console_write_line("Commands:");
    test_console_write_line("  + / - : move target by selected step");
    test_console_write_line("  0 / h : return to horizontal");
    test_console_write_line("  1     : step = 0.2 deg");
    test_console_write_line("  2     : step = 0.5 deg");
    test_console_write_line("  3     : step = 1.0 deg");
    test_console_write_line("  p     : print software target/state");
    test_console_write_line("  q     : query Emm actual position");
    test_console_write_line("  e     : enable motor");
    test_console_write_line("  d / x : disable motor immediately");
    test_console_write_line("  ?     : print this help");
    test_console_write_line("Safety window: -31.1 deg to -25.1 deg.");
}

static void manual_control_print_state(const RodMotorControl *motor,
                                       int32_t step_mdeg,
                                       uint32_t now_ms)
{
    test_console_write("STATE enabled=");
    test_console_write_u32(motor->enabled ? 1U : 0U);
    test_console_write(" startup=");
    test_console_write_u32(motor->startup_complete ? 1U : 0U);
    test_console_write(" busy=");
    test_console_write_u32(
        rod_motor_control_is_busy(motor, now_ms) ? 1U : 0U);
    test_console_write(" target_deg=");
    test_console_write_fixed_milli(motor->target_physical_mdeg);
    test_console_write(" commanded_deg=");
    test_console_write_fixed_milli(motor->commanded_physical_mdeg);
    test_console_write(" pulse=");
    test_console_write_i32(motor->commanded_pulse);
    test_console_write(" step_deg=");
    test_console_write_fixed_milli(step_mdeg);
    test_console_write(" limit_hit=");
    test_console_write_u32(motor->soft_limit_hit ? 1U : 0U);
    test_console_newline();
}

static void manual_control_print_actual(const EmmV5Position *position)
{
    int32_t actual_mdeg = rod_motor_raw_units_to_angle_mdeg(
        position->raw_units);

    test_console_write("ACTUAL raw=");
    test_console_write_i32(position->raw_units);
    test_console_write(" motor_deg=");
    test_console_write_fixed_milli(actual_mdeg);
    test_console_write(" rx_ms=");
    test_console_write_u32(position->received_timestamp_ms);
    test_console_newline();
}

static void manual_control_report_command_result(bool accepted)
{
    if (accepted) {
        test_console_write_line("OK target command sent");
    } else {
        test_console_write_line(
            "REJECTED: motor disabled, busy, not initialized, or outside safety window");
    }
}

static void motor_manual_control_test_run(void)
{
    RodMotorControl motor;
    int32_t step_mdeg = APP_ROD_MANUAL_DEFAULT_STEP_MDEG;
    uint32_t last_timeout_count;

    manual_control_print_help();
    test_console_write_line("STARTUP: moving from physical 0 deg to horizontal...");

    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        test_console_write_line("FATAL: startup-to-horizontal command sequence failed");
        for (;;) {
            __WFI();
        }
    }

    test_console_write_line("READY: horizontal pose reached; manual commands enabled");
    manual_control_print_state(&motor, step_mdeg, bsp_time_ms());
    last_timeout_count = emm_v5_stats()->position_timeouts;
    (void) emm_v5_request_current_position(bsp_time_ms());

    for (;;) {
        uint8_t command;
        EmmV5Position position;
        uint32_t now_ms = bsp_time_ms();

        emm_v5_poll(now_ms);
        if (emm_v5_take_current_position(&position)) {
            manual_control_print_actual(&position);
        }
        if (emm_v5_stats()->position_timeouts != last_timeout_count) {
            last_timeout_count = emm_v5_stats()->position_timeouts;
            test_console_write_line(
                "ERR actual-position query timeout; check Emm TX -> PB5 and common GND");
        }

        while (vision_uart_read(&command)) {
            bool accepted = false;

            if ((command == '\r') || (command == '\n')) {
                continue;
            }

            switch (command) {
            case '+':
                accepted = rod_motor_control_set_target_physical_mdeg(
                    &motor,
                    motor.target_physical_mdeg + step_mdeg,
                    now_ms);
                manual_control_report_command_result(accepted);
                break;
            case '-':
                accepted = rod_motor_control_set_target_physical_mdeg(
                    &motor,
                    motor.target_physical_mdeg - step_mdeg,
                    now_ms);
                manual_control_report_command_result(accepted);
                break;
            case '0':
            case 'h':
            case 'H':
                accepted = rod_motor_control_return_horizontal(&motor, now_ms);
                manual_control_report_command_result(accepted);
                break;
            case '1':
                step_mdeg = 200L;
                test_console_write_line("OK step=0.200 deg");
                break;
            case '2':
                step_mdeg = 500L;
                test_console_write_line("OK step=0.500 deg");
                break;
            case '3':
                step_mdeg = 1000L;
                test_console_write_line("OK step=1.000 deg");
                break;
            case 'p':
            case 'P':
                manual_control_print_state(&motor, step_mdeg, now_ms);
                break;
            case 'q':
            case 'Q':
                if (!emm_v5_request_current_position(now_ms)) {
                    test_console_write_line("BUSY: position request already pending");
                }
                break;
            case 'e':
            case 'E':
                if (rod_motor_control_set_enabled(&motor, true)) {
                    test_console_write_line("OK motor enabled");
                }
                break;
            case 'd':
            case 'D':
            case 'x':
            case 'X':
                if (rod_motor_control_set_enabled(&motor, false)) {
                    test_console_write_line("OK motor disabled/released");
                }
                break;
            case '?':
                manual_control_print_help();
                break;
            default:
                test_console_write_line("UNKNOWN command; send ? for help");
                break;
            }
        }
        __WFI();
    }
}

#endif
