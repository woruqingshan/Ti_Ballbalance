#ifndef MOTOR_SCHEDULER_H
#define MOTOR_SCHEDULER_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_position.h"
typedef struct { uint32_t last_send_ms; uint32_t sent_commands; uint32_t coalesced_updates; bool enabled; bool dry_run; } MotorScheduler;
void motor_scheduler_init(MotorScheduler *s, bool dry_run);
void motor_scheduler_set_enabled(MotorScheduler *s, bool enabled);
void motor_scheduler_update(MotorScheduler *s, MotorPosition *m, uint32_t now_ms);
#endif
