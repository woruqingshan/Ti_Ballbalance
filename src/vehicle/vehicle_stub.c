#include "vehicle_control_port.h"
static VehicleState g_state;
void vehicle_init(void){g_state.valid=true;g_state.stationary=true;g_state.target_speed_mps=0;g_state.speed_mps=0;g_state.accel_mps2=0;g_state.pitch_deg=0;g_state.pitch_rate_deg_s=0;g_state.yaw_rate_deg_s=0;g_state.fault_flags=0;}
VehicleState vehicle_get_state(void){return g_state;}
bool vehicle_is_stationary(void){return g_state.valid&&g_state.stationary;}
void vehicle_request_stop(void){g_state.target_speed_mps=0;g_state.stationary=true;}
