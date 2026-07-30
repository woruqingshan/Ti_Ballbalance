#include "motor_sign_test.h"
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

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static void print_help(void)
{
    test_console_write_line("=== MOTOR SIGN TEST (interactive) ===");
    test_console_write_line("Keep the mechanism near the middle. Remove the ball.");
    test_console_write_line("e: enable and lock motor");
    test_console_write_line("d or x: disable immediately");
    test_console_write_line("+: move positive by current step");
    test_console_write_line("-: move negative by current step");
    test_console_write_line("0: return to software command zero");
    test_console_write_line("q: query and print actual encoder position");
    test_console_write_line("z: set next valid actual position as display zero");
    test_console_write_line("1/2/3: select 5/10/20 pulse step");
    test_console_write_line("h or ?: print help");
    test_console_write_line("Observe which pipe end rises for + and - commands.");
}

static void print_state(const MotorSignTest *test)
{
    int32_t relative_units = 0;

    if (test->have_position && test->zero_valid) {
        relative_units = test->current_raw_units - test->zero_raw_units;
    }

    test_console_write("STATE enabled=");
    test_console_write_u32(test->enabled ? 1U : 0U);
    test_console_write(" step=");
    test_console_write_i32(test->step_pulses);
    test_console_write(" cmd_offset=");
    test_console_write_i32(test->command_offset_pulses);
    if (test->have_position) {
        test_console_write(" raw=");
        test_console_write_i32(test->current_raw_units);
        test_console_write(" rel_pulse=");
        test_console_write_i32(units_to_pulse_equivalent(relative_units));
        test_console_write(" rel_deg=");
        test_console_write_fixed_milli(
            units_to_motor_degree_milli(relative_units));
    } else {
        test_console_write(" actual=unavailable");
    }
    test_console_newline();
}

static void request_position_soon(MotorSignTest *test, uint32_t now_ms)
{
    test->query_due_ms = now_ms + APP_SIGN_TEST_POSITION_SETTLE_MS;
}

static void send_nudge(MotorSignTest *test, int32_t pulses, uint32_t now_ms)
{
    int32_t next_offset;

    if (!test->enabled) {
        test_console_write_line("ERR motor disabled; send 'e' first");
        return;
    }

    next_offset = test->command_offset_pulses + pulses;
    if (abs_i32(next_offset) > APP_SIGN_TEST_SOFT_LIMIT_PULSE) {
        test_console_write_line("ERR sign-test software limit reached; use '0'");
        return;
    }

    if (emm_v5_move_relative(pulses,
                             APP_SIGN_TEST_SPEED_RPM,
                             APP_SIGN_TEST_ACCELERATION)) {
        test->command_offset_pulses = next_offset;
        test_console_write("COMMAND ");
        if (pulses > 0) {
            test_console_write("POSITIVE +");
            test_console_write_i32(pulses);
        } else {
            test_console_write("NEGATIVE ");
            test_console_write_i32(pulses);
        }
        test_console_write_line(" pulse sent");
        request_position_soon(test, now_ms);
    }
}

static void return_to_zero(MotorSignTest *test, uint32_t now_ms)
{
    int32_t correction;

    if (!test->enabled) {
        test_console_write_line("ERR motor disabled; send 'e' first");
        return;
    }

    correction = -test->command_offset_pulses;
    if (correction == 0) {
        test_console_write_line("OK already at software command zero");
        return;
    }

    if (emm_v5_move_relative(correction,
                             APP_SIGN_TEST_SPEED_RPM,
                             APP_SIGN_TEST_ACCELERATION)) {
        test->command_offset_pulses = 0;
        test_console_write("COMMAND RETURN_ZERO delta=");
        test_console_write_i32(correction);
        test_console_newline();
        request_position_soon(test, now_ms);
    }
}

static void process_console_commands(MotorSignTest *test, uint32_t now_ms)
{
    uint8_t byte;

    while (vision_uart_read(&byte)) {
        switch (byte) {
        case 'e':
        case 'E':
            (void) emm_v5_enable(true);
            test->enabled = true;
            test_console_write_line("OK motor enabled");
            request_position_soon(test, now_ms);
            break;
        case 'd':
        case 'D':
        case 'x':
        case 'X':
            (void) emm_v5_enable(false);
            test->enabled = false;
            test_console_write_line("OK motor disabled");
            break;
        case '+':
            send_nudge(test, test->step_pulses, now_ms);
            break;
        case '-':
            send_nudge(test, -test->step_pulses, now_ms);
            break;
        case '0':
            return_to_zero(test, now_ms);
            break;
        case 'q':
        case 'Q':
            test->query_due_ms = now_ms;
            break;
        case 'z':
        case 'Z':
            test->zero_after_next_position = true;
            test->query_due_ms = now_ms;
            test_console_write_line("OK zero will be set from next valid position");
            break;
        case '1':
            test->step_pulses = 5;
            test_console_write_line("OK step=5 pulse");
            break;
        case '2':
            test->step_pulses = 10;
            test_console_write_line("OK step=10 pulse");
            break;
        case '3':
            test->step_pulses = 20;
            test_console_write_line("OK step=20 pulse");
            break;
        case 'p':
        case 'P':
            print_state(test);
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

void motor_sign_test_init(MotorSignTest *test, uint32_t now_ms)
{
    test->enabled = false;
    test->have_position = false;
    test->zero_valid = false;
    test->zero_after_next_position = true;
    test->command_offset_pulses = 0;
    test->step_pulses = APP_SIGN_TEST_DEFAULT_STEP_PULSE;
    test->current_raw_units = 0;
    test->zero_raw_units = 0;
    test->query_due_ms = now_ms + 200U;
    test->last_reported_timeout_count = 0U;

    (void) emm_v5_enable(false);
    print_help();
    test_console_write_line("SAFE START: motor disabled; type 'e' to enable");
}

void motor_sign_test_update(MotorSignTest *test, uint32_t now_ms)
{
    EmmV5Position position;
    const EmmV5Stats *stats;

    process_console_commands(test, now_ms);

    if (((int32_t) (now_ms - test->query_due_ms) >= 0) &&
        !emm_v5_position_request_pending()) {
        if (emm_v5_request_current_position(now_ms)) {
            test->query_due_ms = now_ms + 0x7FFFFFFFU;
        }
    }

    if (emm_v5_take_current_position(&position)) {
        test->current_raw_units = position.raw_units;
        test->have_position = true;
        if (test->zero_after_next_position || !test->zero_valid) {
            test->zero_raw_units = position.raw_units;
            test->zero_valid = true;
            test->zero_after_next_position = false;
            test_console_write_line("ACTUAL display zero updated");
        }
        print_state(test);
    }

    stats = emm_v5_stats();
    if (stats->position_timeouts != test->last_reported_timeout_count) {
        test->last_reported_timeout_count = stats->position_timeouts;
        test_console_write("ERR position query timeout count=");
        test_console_write_u32(stats->position_timeouts);
        test_console_write_line("; movement commands still use TX only");
    }
}
