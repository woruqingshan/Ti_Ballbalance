#include <assert.h>
#include <stdint.h>

#include "../src/motor/rod_motor_math.h"

int main(void)
{
    assert(rod_motor_angle_mdeg_to_pulse(0) == 0);
    assert(rod_motor_angle_mdeg_to_pulse(-28100) == -250);
    assert(rod_motor_angle_mdeg_to_pulse(28100) == 250);
    assert(rod_motor_pulse_to_angle_mdeg(-250) == -28125);
    assert(rod_motor_pulse_to_angle_mdeg(250) == 28125);
    assert(rod_motor_clamp_i32(-40000, -31100, -25100) == -31100);
    assert(rod_motor_clamp_i32(-28000, -31100, -25100) == -28000);
    assert(rod_motor_clamp_i32(-20000, -31100, -25100) == -25100);
    assert(rod_motor_raw_units_to_angle_mdeg(32768) == 180000);
    return 0;
}
