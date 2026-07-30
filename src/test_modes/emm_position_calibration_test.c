#include "emm_position_calibration_test.h"
#include "app/app_config.h"
#include "communication/vision_uart.h"
#include "motor/emm_v5_driver.h"
#include "test_console.h"

#define EMM_COMMAND_PULSES_PER_REVOLUTION   (3200LL)
#define MOTOR_DEGREES_MILLI_PER_REVOLUTION  (360000LL)

static int32_t units_to_pulse_equivalent(int32_t units)
{
    int64_t scaled = (int64_t) units * EMM_COMMAND_PULSES_PER_REVOLUTION;
    if (scaled >= 0) {
        scaled += APP_EMM_POSITION_UNITS_PER_REVOLUTION / 2;
    } else {
        scaled -= APP_EMM_POSITION_UNITS_PER_REVOLUTION / 2;
    }
    return (int32_t) (scaled / APP_EMM_POSITION_UNITS_PER_REVOLUTION);
}

static int64_t units_to_motor_degree_milli(int32_t units)
{
    return ((int64_t) units * MOTOR_DEGREES_MILLI_PER_REVOLUTION) /
           APP_EMM_POSITION_UNITS_PER_REVOLUTION;
}

static void print_help(void)
{
    test_console_write_line("=== EMM POSITION CALIBRATION ===");
    test_console_write_line("Motor is DISABLED. Move the pipe slowly by hand.");
    test_console_write_line("z: set current encoder position as relative zero");
    test_console_write_line("r: reset MIN/MAX to current relative position");
    test_console_write_line("p: print current snapshot");
    test_console_write_line("d: send motor disable again");
    test_console_write_line("h or ?: print this help");
    test_console_write_line("Fields: raw=encoder units, rel=relative units,");
    test_console_write_line("pulse=16-microstep equivalent, deg=motor-shaft degree.");
    test_console_write_line("NOTE: deg is NOT the real pipe angle.");
}

static void print_position(const EmmPositionCalibrationTest *test)
{
    int32_t relative;

    if (!test->have_position) {
        test_console_write_line("POS unavailable: no valid 0x36 response yet");
        return;
    }

    relative = test->zero_valid
        ? test->current_raw_units - test->zero_raw_units
        : 0;

    test_console_write("POS raw=");
    test_console_write_i32(test->current_raw_units);
    test_console_write(" rel=");
    test_console_write_i32(relative);
    test_console_write(" pulse=");
    test_console_write_i32(units_to_pulse_equivalent(relative));
    test_console_write(" deg=");
    test_console_write_fixed_milli(units_to_motor_degree_milli(relative));
    test_console_write(" min=");
    test_console_write_i32(test->min_relative_units);
    test_console_write(" max=");
    test_console_write_i32(test->max_relative_units);
    test_console_newline();
}

static void process_console_commands(EmmPositionCalibrationTest *test)
{
    uint8_t byte;

    while (vision_uart_read(&byte)) {
        switch (byte) {
        case 'z':
        case 'Z':
            if (test->have_position) {
                test->zero_raw_units = test->current_raw_units;
                test->zero_valid = true;
                test->min_relative_units = 0;
                test->max_relative_units = 0;
                test_console_write_line("OK zero set; MIN/MAX reset");
                print_position(test);
            } else {
                test_console_write_line("ERR zero not set: no position");
            }
            break;
        case 'r':
        case 'R':
            if (test->have_position && test->zero_valid) {
                int32_t relative =
                    test->current_raw_units - test->zero_raw_units;
                test->min_relative_units = relative;
                test->max_relative_units = relative;
                test_console_write_line("OK MIN/MAX reset to current position");
                print_position(test);
            } else {
                test_console_write_line("ERR reset failed: position/zero unavailable");
            }
            break;
        case 'p':
        case 'P':
            print_position(test);
            break;
        case 'd':
        case 'D':
            (void) emm_v5_enable(false);
            test_console_write_line("OK motor disable command sent");
            break;
        case 'h':
        case 'H':
        case '?':
            print_help();
            break;
        default:
            break;
        }
    }
}

void emm_position_calibration_test_init(EmmPositionCalibrationTest *test,
                                        uint32_t now_ms)
{
    test->have_position = false;
    test->zero_valid = false;
    test->current_raw_units = 0;
    test->zero_raw_units = 0;
    test->min_relative_units = 0;
    test->max_relative_units = 0;
    test->last_query_ms = now_ms - APP_EMM_POSITION_QUERY_PERIOD_MS;
    test->last_reported_timeout_count = 0U;

    (void) emm_v5_enable(false);
    print_help();
}

void emm_position_calibration_test_update(EmmPositionCalibrationTest *test,
                                          uint32_t now_ms)
{
    EmmV5Position position;
    const EmmV5Stats *stats;

    process_console_commands(test);

    if ((uint32_t) (now_ms - test->last_query_ms) >=
        APP_EMM_POSITION_QUERY_PERIOD_MS) {
        if (emm_v5_request_current_position(now_ms)) {
            test->last_query_ms = now_ms;
        }
    }

    if (emm_v5_take_current_position(&position)) {
        int32_t relative;
        test->current_raw_units = position.raw_units;
        test->have_position = true;

        if (!test->zero_valid) {
            test->zero_raw_units = position.raw_units;
            test->zero_valid = true;
            test->min_relative_units = 0;
            test->max_relative_units = 0;
            test_console_write_line("AUTO zero set from first valid position");
        }

        relative = test->current_raw_units - test->zero_raw_units;
        if (relative < test->min_relative_units) {
            test->min_relative_units = relative;
        }
        if (relative > test->max_relative_units) {
            test->max_relative_units = relative;
        }
        print_position(test);
    }

    stats = emm_v5_stats();
    if (stats->position_timeouts != test->last_reported_timeout_count) {
        test->last_reported_timeout_count = stats->position_timeouts;
        test_console_write("ERR position query timeout count=");
        test_console_write_u32(stats->position_timeouts);
        test_console_write_line("; check PB5<-Emm TX and common GND");
    }
}
