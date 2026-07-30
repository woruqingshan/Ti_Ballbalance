#include "motor_scheduler.h"
#include "emm_v5_driver.h"
#include "app/app_config.h"
#include "common/common.h"
void motor_scheduler_init(MotorScheduler *s,bool dry){s->last_send_ms=0;s->sent_commands=0;s->coalesced_updates=0;s->enabled=false;s->dry_run=dry;}
void motor_scheduler_set_enabled(MotorScheduler *s,bool en){s->enabled=en;if(!s->dry_run)(void)emm_v5_enable(en);}
void motor_scheduler_update(MotorScheduler *s,MotorPosition *m,uint32_t now)
{
    int32_t d,send;
    if(!s->enabled||!m->zero_valid) return;
    d=motor_position_delta(m); if(d<APP_MOTOR_MIN_DELTA_PULSE&&d>-APP_MOTOR_MIN_DELTA_PULSE) return;
    if((uint32_t)(now-s->last_send_ms)<APP_MOTOR_COMMAND_PERIOD_MS){s->coalesced_updates++;return;}
    send=bbc_clampi32(d,-APP_MOTOR_MAX_DELTA_PULSE,APP_MOTOR_MAX_DELTA_PULSE);
    if(s->dry_run||emm_v5_move_relative(send,APP_MOTOR_SPEED_RPM,APP_MOTOR_ACCELERATION)){
        motor_position_commit(m,send);s->last_send_ms=now;s->sent_commands++;
    }
}
