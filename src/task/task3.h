#ifndef TASK3_H
#define TASK3_H
#include <stdbool.h>
#include <stdint.h>
#include "control/ball_state.h"
typedef enum { TASK3_IDLE=0,TASK3_ARMED,TASK3_MOVE_PLUS5,TASK3_CONFIRM_PLUS5,TASK3_MOVE_MINUS5,TASK3_SETTLE_MINUS5,TASK3_SUCCESS,TASK3_FAILED } Task3State;
typedef struct { Task3State state; uint32_t task_start_ms; uint32_t state_start_ms; uint32_t stable_start_ms; float target_cm; uint32_t result_reason; bool start_requested; } Task3;
void task3_init(Task3 *t);
void task3_arm(Task3 *t);
void task3_request_start(Task3 *t);
void task3_abort(Task3 *t,uint32_t reason);
void task3_update(Task3 *t,const BallState *b,bool ball_available,bool vehicle_stationary,uint32_t now_ms);
bool task3_running(const Task3 *t);
#endif
