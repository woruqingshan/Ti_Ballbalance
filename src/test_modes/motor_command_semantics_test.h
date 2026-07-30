#ifndef MOTOR_COMMAND_SEMANTICS_TEST_H
#define MOTOR_COMMAND_SEMANTICS_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "motor/rod_motor_control.h"
#include "motor/rod_motor_math.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

/* Header-only test mode: no Keil project-file change is required. */

static void semantics_print_help(void)
{
    test_console_write_line("");
    test_console_write_line("=== PHASE 4: EMM DYNAMIC COMMAND SEMANTICS ===");
    test_console_write_line("Each reset starts from manually established physical 0 deg,");
    test_console_write_line("then automatically moves to horizontal -28.1 deg.");
    test_console_write_line("Select command gap:");
    test_console_write_line("  1=500ms  2=200ms  3=100ms  4=50ms  5=25ms");
    test_console_write_line("Run exactly ONE scenario per reset:");
    test_console_write_line("  a : +2deg then -2deg; queued final should be horizontal");
    test_console_write_line("  b : +2deg then -4deg; queued final should be -2deg offset");
    test_console_write_line("  c : -2deg then +4deg; queued final should be +2deg offset");
    test_console_write_line("  q : query actual motor position");
    test_console_write_line("  p : print selected gap and status");
    test_console_write_line("  d/x: disable/release motor");
    test_console_write_line("After a/b/c: disable, manually return to physical 0, then reset.");
}

static void semantics_wait_ms(uint32_t delay_ms)
{
    uint32_t start_ms = bsp_time_ms();

    while ((uint32_t) (bsp_time_ms() - start_ms) < delay_ms) {
        emm_v5_poll(bsp_time_ms());
        __WFI();
    }
}

static bool semantics_query_and_print_actual(void)
{
    uint32_t start_ms;
    EmmV5Position position;

    if (!emm_v5_request_current_position(bsp_time_ms())) {
        test_console_write_line("ERR position request rejected/pending");
        return false;
    }

    start_ms = bsp_time_ms();
    while ((uint32_t) (bsp_time_ms() - start_ms) <
           APP_SEMANTICS_QUERY_WAIT_MS) {
        emm_v5_poll(bsp_time_ms());
        if (emm_v5_take_current_position(&position)) {
            int32_t actual_mdeg = rod_motor_raw_units_to_angle_mdeg(
                position.raw_units);
            test_console_write("RESULT actual_raw=");
            test_console_write_i32(position.raw_units);
            test_console_write(" actual_motor_deg=");
            test_console_write_fixed_milli(actual_mdeg);
            test_console_newline();
            return true;
        }
        __WFI();
    }

    test_console_write_line("ERR result position query timed out");
    return false;
}

static void semantics_print_status(uint32_t gap_ms, bool scenario_done)
{
    test_console_write("STATUS gap_ms=");
    test_console_write_u32(gap_ms);
    test_console_write(" scenario_done=");
    test_console_write_u32(scenario_done ? 1U : 0U);
    test_console_write(" semantics_verified_macro=");
    test_console_write_u32(APP_EMM_COMMAND_SEMANTICS_VERIFIED);
    test_console_newline();
}

static void semantics_run_scenario(char scenario, uint32_t gap_ms)
{
    int32_t offset_pulse = rod_motor_angle_mdeg_to_pulse(
        APP_SEMANTICS_TEST_OFFSET_MDEG);
    int32_t first_pulse = 0;
    int32_t second_pulse = 0;
    int32_t expected_offset_mdeg = 0;

    switch (scenario) {
    case 'a':
        first_pulse = offset_pulse;
        second_pulse = -offset_pulse;
        expected_offset_mdeg = 0;
        break;
    case 'b':
        first_pulse = offset_pulse;
        second_pulse = -2 * offset_pulse;
        expected_offset_mdeg = -APP_SEMANTICS_TEST_OFFSET_MDEG;
        break;
    case 'c':
        first_pulse = -offset_pulse;
        second_pulse = 2 * offset_pulse;
        expected_offset_mdeg = APP_SEMANTICS_TEST_OFFSET_MDEG;
        break;
    default:
        return;
    }

    test_console_write("RUN scenario=");
    {
        uint8_t letter = (uint8_t) scenario;
        vision_uart_write(&letter, 1U);
    }
    test_console_write(" gap_ms=");
    test_console_write_u32(gap_ms);
    test_console_write(" first_pulse=");
    test_console_write_i32(first_pulse);
    test_console_write(" second_pulse=");
    test_console_write_i32(second_pulse);
    test_console_write(" queued_expected_offset_deg=");
    test_console_write_fixed_milli(expected_offset_mdeg);
    test_console_newline();

    test_console_write_line("SEND first command");
    (void) emm_v5_move_relative(first_pulse,
                                APP_SEMANTICS_TEST_SPEED_RPM,
                                APP_SEMANTICS_TEST_ACCELERATION);
    semantics_wait_ms(gap_ms);

    test_console_write_line("SEND second command");
    (void) emm_v5_move_relative(second_pulse,
                                APP_SEMANTICS_TEST_SPEED_RPM,
                                APP_SEMANTICS_TEST_ACCELERATION);
    semantics_wait_ms(APP_SEMANTICS_RESULT_SETTLE_MS);

    test_console_write_line("Observe path: full first move then second = queued;");
    test_console_write_line("early reversal/change = interrupt or target replacement.");
    (void) semantics_query_and_print_actual();
    test_console_write_line("TEST COMPLETE. Do not run another scenario without reset.");
}

static void motor_command_semantics_test_run(void)
{
    RodMotorControl motor;
    uint32_t gap_ms = APP_SEMANTICS_DEFAULT_GAP_MS;
    bool scenario_done = false;

    semantics_print_help();
    test_console_write_line("STARTUP: moving from physical 0 deg to horizontal...");
    if (!rod_motor_control_startup_to_horizontal(&motor)) {
        test_console_write_line("FATAL: startup-to-horizontal command sequence failed");
        for (;;) {
            __WFI();
        }
    }

    test_console_write_line("READY: select gap and run one scenario");
    semantics_print_status(gap_ms, scenario_done);

    for (;;) {
        uint8_t command;
        uint32_t now_ms = bsp_time_ms();

        emm_v5_poll(now_ms);
        while (vision_uart_read(&command)) {
            if ((command == '\r') || (command == '\n')) {
                continue;
            }

            switch (command) {
            case '1':
                gap_ms = 500U;
                test_console_write_line("OK gap=500ms");
                break;
            case '2':
                gap_ms = 200U;
                test_console_write_line("OK gap=200ms");
                break;
            case '3':
                gap_ms = 100U;
                test_console_write_line("OK gap=100ms");
                break;
            case '4':
                gap_ms = 50U;
                test_console_write_line("OK gap=50ms");
                break;
            case '5':
                gap_ms = 25U;
                test_console_write_line("OK gap=25ms");
                break;
            case 'a':
            case 'A':
            case 'b':
            case 'B':
            case 'c':
            case 'C':
                if (scenario_done) {
                    test_console_write_line(
                        "REJECTED: one scenario already ran; disable, return to physical 0, reset");
                } else {
                    char scenario = (char) command;
                    if ((scenario >= 'A') && (scenario <= 'C')) {
                        scenario = (char) (scenario - 'A' + 'a');
                    }
                    scenario_done = true;
                    semantics_run_scenario(scenario, gap_ms);
                }
                break;
            case 'q':
            case 'Q':
                (void) semantics_query_and_print_actual();
                break;
            case 'p':
            case 'P':
                semantics_print_status(gap_ms, scenario_done);
                break;
            case 'd':
            case 'D':
            case 'x':
            case 'X':
                (void) rod_motor_control_set_enabled(&motor, false);
                test_console_write_line("OK motor disabled/released");
                break;
            case '?':
                semantics_print_help();
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
