#include "safety.h"
#include "faults.h"
#include "app/app_config.h"
void safety_init(SafetyState*s){s->faults=0;s->control_allowed=false;}
void safety_set_fault(SafetyState*s,uint32_t f){s->faults|=f;s->control_allowed=false;}
void safety_clear(SafetyState*s){s->faults=0;s->control_allowed=false;}
void safety_update(SafetyState*s,const BallState*b,bool has,const MotorPosition*m,uint32_t now)
{
    uint32_t faults=0U;
    if(!has||(uint32_t)(now-b->received_timestamp_ms)>APP_VISION_TIMEOUT_MS) faults|=FAULT_VISION_TIMEOUT;
    if(has&&!b->valid) faults|=FAULT_VISION_INVALID;
    if(m->soft_limit_hit) faults|=FAULT_SOFT_LIMIT;
    s->faults=faults;
    s->control_allowed=(faults==0U);
}
