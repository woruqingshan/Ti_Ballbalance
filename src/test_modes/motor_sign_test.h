#ifndef MOTOR_SIGN_TEST_H
#define MOTOR_SIGN_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled;
    bool have_position;
    bool zero_valid;
    bool zero_after_next_position;
    int32_t command_offset_pulses;
    int32_t step_pulses;
    int32_t current_raw_units;
    int32_t zero_raw_units;
    uint32_t query_due_ms;
    uint32_t last_reported_timeout_count;
} MotorSignTest;

void motor_sign_test_init(MotorSignTest *test, uint32_t now_ms);
void motor_sign_test_update(MotorSignTest *test, uint32_t now_ms);

#endif
