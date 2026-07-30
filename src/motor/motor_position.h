#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    int32_t zero_pulse;
    int32_t target_pulse;
    int32_t commanded_pulse;
    int32_t soft_min_pulse;
    int32_t soft_max_pulse;
    bool zero_valid;
    bool soft_limit_hit;
} MotorPosition;
void motor_position_init(MotorPosition *m, int32_t min_pulse, int32_t max_pulse);
void motor_position_set_zero(MotorPosition *m);
void motor_position_set_target(MotorPosition *m, int32_t target);
int32_t motor_position_delta(const MotorPosition *m);
void motor_position_commit(MotorPosition *m, int32_t sent_delta);
#endif
