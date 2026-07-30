#ifndef SAFETY_H
#define SAFETY_H
#include <stdbool.h>
#include <stdint.h>
#include "control/ball_state.h"
#include "motor/motor_position.h"
typedef struct { uint32_t faults; bool control_allowed; } SafetyState;
void safety_init(SafetyState *s);
void safety_update(SafetyState *s,const BallState *b,bool has_ball,const MotorPosition *m,uint32_t now_ms);
void safety_set_fault(SafetyState *s,uint32_t fault);
void safety_clear(SafetyState *s);
#endif
