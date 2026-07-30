#ifndef VEHICLE_CONTROL_PORT_H
#define VEHICLE_CONTROL_PORT_H
#include <stdbool.h>
typedef struct { bool valid; bool stationary; float target_speed_mps; float speed_mps; float accel_mps2; float pitch_deg; float pitch_rate_deg_s; float yaw_rate_deg_s; unsigned fault_flags; } VehicleState;
void vehicle_init(void);
VehicleState vehicle_get_state(void);
bool vehicle_is_stationary(void);
void vehicle_request_stop(void);
#endif
