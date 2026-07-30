#include "ball_controller.h"
#include "common/common.h"
void ball_controller_init(BallController*c,float kp,float kd,float umax,float slew){c->kp=kp;c->kd=kd;c->u_max=umax;c->slew=slew;c->previous=0;c->raw=0;c->limited=0;}
void ball_controller_reset(BallController*c){c->previous=0;c->raw=0;c->limited=0;}
float ball_controller_update(BallController*c,float target,const BallState*b)
{
    float lo,hi;
    c->raw=c->kp*(target-b->position_cm)-c->kd*b->velocity_cm_s;
    c->limited=bbc_clampf(c->raw,-c->u_max,c->u_max);
    lo=c->previous-c->slew;hi=c->previous+c->slew;c->limited=bbc_clampf(c->limited,lo,hi);c->previous=c->limited;return c->limited;
}
