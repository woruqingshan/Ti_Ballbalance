#include "telemetry.h"
#include "communication/vision_link.h"
#include "protocol/protocol.h"
#include <stdint.h>
static int16_t q_cm(float x){float v=x*100.0f;if(v>32767)v=32767;if(v<-32768)v=-32768;return(int16_t)(v>=0?v+0.5f:v-0.5f);}
static int16_t q_u(float x){float v=x*1000.0f;if(v>32767)v=32767;if(v<-32768)v=-32768;return(int16_t)(v>=0?v+0.5f:v-0.5f);}
void telemetry_send(const Task3*t,const BallState*b,bool has,const BallController*c,const MotorPosition*m,const MotorScheduler*ms,const SafetyState*s,uint32_t now)
{
    uint8_t p[32]={0};p[0]=(uint8_t)t->state;p[1]=(uint8_t)(has&&b->valid);protocol_put_u16_le(&p[2],(uint16_t)s->faults);protocol_put_u32_le(&p[4],task3_running(t)?now-t->task_start_ms:0U);protocol_put_i16_le(&p[8],q_cm(t->target_cm));protocol_put_i16_le(&p[10],q_cm(has?b->position_cm:0));protocol_put_i16_le(&p[12],q_cm(has?b->velocity_cm_s:0));protocol_put_i16_le(&p[14],q_u(c->raw));protocol_put_i16_le(&p[16],q_u(c->limited));protocol_put_u32_le(&p[18],(uint32_t)m->target_pulse);protocol_put_u32_le(&p[22],(uint32_t)m->commanded_pulse);protocol_put_u32_le(&p[26],ms->sent_commands);protocol_put_u16_le(&p[30],(uint16_t)ms->coalesced_updates);vision_link_send_frame(MSG_CONTROL_TELEMETRY,p,sizeof(p),now);
}
void telemetry_send_heartbeat(uint32_t now,uint32_t mode){uint8_t p[8];protocol_put_u32_le(&p[0],mode);protocol_put_u32_le(&p[4],now);vision_link_send_frame(MSG_HEARTBEAT,p,sizeof(p),now);}
