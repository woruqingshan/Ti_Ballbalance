#include "rod_motor_mapper.h"
#include "app/app_config.h"
int32_t rod_motor_map_control(float u)
{
    float p=u*APP_PULSES_PER_CONTROL_UNIT;
    return (int32_t)(p>=0.0f ? p+0.5f : p-0.5f);
}
