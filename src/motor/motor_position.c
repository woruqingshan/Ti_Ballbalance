#include "motor_position.h"
#include "common/common.h"
void motor_position_init(MotorPosition *m,int32_t lo,int32_t hi){m->zero_pulse=0;m->target_pulse=0;m->commanded_pulse=0;m->soft_min_pulse=lo;m->soft_max_pulse=hi;m->zero_valid=false;m->soft_limit_hit=false;}
void motor_position_set_zero(MotorPosition *m){m->zero_pulse=0;m->target_pulse=0;m->commanded_pulse=0;m->zero_valid=true;m->soft_limit_hit=false;}
void motor_position_set_target(MotorPosition *m,int32_t target){int32_t c=bbc_clampi32(target,m->soft_min_pulse,m->soft_max_pulse);m->soft_limit_hit=(c!=target);m->target_pulse=c;}
int32_t motor_position_delta(const MotorPosition *m){return m->target_pulse-m->commanded_pulse;}
void motor_position_commit(MotorPosition *m,int32_t d){m->commanded_pulse+=d;}
