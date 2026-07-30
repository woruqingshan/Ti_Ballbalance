#ifndef MOTOR_STARTUP_HORIZONTAL_TEST_H
#define MOTOR_STARTUP_HORIZONTAL_TEST_H

#include "app/app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "test_modes/test_console.h"
#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * Header-only hardware test so no .uvprojx modification is required.
 * This header is included only by app_main.c.
 */

static int32_t startup_angle_mdeg_to_pulses(int32_t angle_mdeg)
{
    int64_t numerator = (int64_t) angle_mdeg *
                        (int64_t) APP_STARTUP_TEST_PULSES_PER_REV;

    /* Round to nearest whole pulse. */
    if (numerator >= 0) {
        numerator += 180000LL;
    } else {
        numerator -= 180000LL;
    }
    return (int32_t) (numerator / 360000LL);
}

static void startup_wait_and_drain(uint32_t delay_ms)
{
    uint32_t start = bsp_time_ms();
    while ((uint32_t) (bsp_time_ms() - start) < delay_ms) {
        emm_v5_poll(bsp_time_ms());
        __WFI();
    }
}

static void motor_startup_horizontal_test_run(void)
{
    int32_t target_pulses = startup_angle_mdeg_to_pulses(
        APP_STARTUP_TEST_TARGET_ANGLE_MDEG);

    test_console_write_line("");
    test_console_write_line("=== MOTOR STARTUP TO HORIZONTAL TEST ===");
    test_console_write_line("Assumption: before reset, mechanism is physically at 0 deg.");
    test_console_write("Target angle mdeg=");
    test_console_write_i32(APP_STARTUP_TEST_TARGET_ANGLE_MDEG);
    test_console_write(" target pulses=");
    test_console_write_i32(target_pulses);
    test_console_newline();
    test_console_write_line("Automatic motion starts after boot delay.");
    test_console_write_line("Send 'd' or 'x' later to disable and release the motor.");

    startup_wait_and_drain(APP_STARTUP_TEST_BOOT_DELAY_MS);

    test_console_write_line("STEP 1: disable motor");
    (void) emm_v5_enable(false);
    startup_wait_and_drain(APP_STARTUP_TEST_COMMAND_GAP_MS);

    test_console_write_line("STEP 2: clear current position to zero");
    (void) emm_v5_clear_current_position();
    startup_wait_and_drain(APP_STARTUP_TEST_COMMAND_GAP_MS);

    test_console_write_line("STEP 3: enable motor");
    (void) emm_v5_enable(true);
    startup_wait_and_drain(APP_STARTUP_TEST_COMMAND_GAP_MS);

    test_console_write("STEP 4: relative move pulses=");
    test_console_write_i32(target_pulses);
    test_console_write(" speed_rpm=");
    test_console_write_u32(APP_STARTUP_TEST_SPEED_RPM);
    test_console_write(" accel=");
    test_console_write_u32(APP_STARTUP_TEST_ACCELERATION);
    test_console_newline();

    (void) emm_v5_move_relative(target_pulses,
                                APP_STARTUP_TEST_SPEED_RPM,
                                APP_STARTUP_TEST_ACCELERATION);

    test_console_write_line("MOVE COMMAND SENT; motor remains enabled to hold position.");

    for (;;) {
        uint8_t command;
        emm_v5_poll(bsp_time_ms());

        while (vision_uart_read(&command)) {
            if ((command == (uint8_t) 'd') ||
                (command == (uint8_t) 'D') ||
                (command == (uint8_t) 'x') ||
                (command == (uint8_t) 'X')) {
                (void) emm_v5_enable(false);
                test_console_write_line("Motor disable command sent.");
            }
        }
        __WFI();
    }
}

#endif
