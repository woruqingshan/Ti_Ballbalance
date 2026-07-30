#ifndef BALL_CONTROLLER_H
#define BALL_CONTROLLER_H
#include "ball_state.h"
typedef struct { float kp; float kd; float u_max; float slew; float previous; float raw; float limited; } BallController;
void ball_controller_init(BallController *c,float kp,float kd,float umax,float slew);
float ball_controller_update(BallController *c,float target_cm,const BallState *ball);
void ball_controller_reset(BallController *c);
#endif
