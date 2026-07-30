#include "task3.h"
#include "app/app_config.h"
#include "common/common.h"
#include "safety/faults.h"
static bool within(float v,float target,float tol){float e=v-target;return e<tol&&e>-tol;}
static float absf(float v){return v<0?-v:v;}
void task3_init(Task3*t){t->state=TASK3_IDLE;t->task_start_ms=0;t->state_start_ms=0;t->stable_start_ms=0;t->target_cm=0;t->result_reason=0;t->start_requested=false;}
void task3_arm(Task3*t){if(t->state==TASK3_IDLE||t->state==TASK3_FAILED||t->state==TASK3_SUCCESS){t->state=TASK3_ARMED;t->target_cm=0;t->result_reason=0;t->start_requested=false;}}
void task3_request_start(Task3*t){t->start_requested=true;}
void task3_abort(Task3*t,uint32_t r){t->state=TASK3_FAILED;t->result_reason=r;t->target_cm=0;}
bool task3_running(const Task3*t){return t->state>=TASK3_MOVE_PLUS5&&t->state<=TASK3_SETTLE_MINUS5;}
void task3_update(Task3*t,const BallState*b,bool has,bool stationary,uint32_t now)
{
    if(task3_running(t)&&(uint32_t)(now-t->task_start_ms)>APP_TASK3_TIMEOUT_MS){task3_abort(t,FAULT_TASK_TIMEOUT);return;}
    switch(t->state){
    case TASK3_ARMED:
        if(t->start_requested&&has&&b->valid&&stationary&&within(b->position_cm,0.0f,1.0f)&&absf(b->velocity_cm_s)<3.0f){t->state=TASK3_MOVE_PLUS5;t->task_start_ms=now;t->state_start_ms=now;t->target_cm=5.0f;t->stable_start_ms=0;}
        break;
    case TASK3_MOVE_PLUS5:
        if(within(b->position_cm,5.0f,APP_TASK3_ARRIVAL_TOL_CM)&&absf(b->velocity_cm_s)<APP_TASK3_PLUS_MAX_SPEED_CM_S){if(t->stable_start_ms==0)t->stable_start_ms=now;if((uint32_t)(now-t->stable_start_ms)>=APP_TASK3_PLUS_HOLD_MS){t->state=TASK3_CONFIRM_PLUS5;t->state_start_ms=now;t->target_cm=-5.0f;t->stable_start_ms=0;}}else t->stable_start_ms=0;
        break;
    case TASK3_CONFIRM_PLUS5:
        if((uint32_t)(now-t->state_start_ms)>=50U){t->state=TASK3_MOVE_MINUS5;t->state_start_ms=now;}
        break;
    case TASK3_MOVE_MINUS5:
        if(within(b->position_cm,-5.0f,APP_TASK3_ARRIVAL_TOL_CM)){t->state=TASK3_SETTLE_MINUS5;t->state_start_ms=now;t->stable_start_ms=0;}
        break;
    case TASK3_SETTLE_MINUS5:
        if(within(b->position_cm,-5.0f,APP_TASK3_ARRIVAL_TOL_CM)&&absf(b->velocity_cm_s)<APP_TASK3_FINAL_MAX_SPEED_CM_S){if(t->stable_start_ms==0)t->stable_start_ms=now;if((uint32_t)(now-t->stable_start_ms)>=APP_TASK3_FINAL_HOLD_MS){t->state=TASK3_SUCCESS;t->target_cm=-5.0f;}}else t->stable_start_ms=0;
        break;
    default: break;
    }
}
