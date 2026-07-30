#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <stdint.h>
#include "task/task3.h"
#include "control/ball_state.h"
#include "control/ball_controller.h"
#include "motor/motor_position.h"
#include "motor/motor_scheduler.h"
#include "safety/safety.h"
void telemetry_send(const Task3*,const BallState*,bool,const BallController*,const MotorPosition*,const MotorScheduler*,const SafetyState*,uint32_t now_ms);
void telemetry_send_heartbeat(uint32_t now_ms,uint32_t mode);
#endif
