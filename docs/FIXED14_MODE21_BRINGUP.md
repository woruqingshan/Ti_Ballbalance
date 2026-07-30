# Fixed-14 protocol alignment and Mode 21 live motor bring-up

Baseline: `Ti_Ballbalance` commit `b2a2858485f87f07a2b8c4ea2918e6bdacbb4daa`.

## Scope

This patch completes the six requested items:

1. accept Fixed-14 `invalid_reason` in flags bits 4..7;
2. verify all three frozen golden vectors;
3. rerun Mode 18 using real source invalid reasons;
4. rerun Mode 19 using real source invalid reasons;
5. implement Mode 21: Fixed-14 -> gate -> PD -> RodMotorControl -> Emm;
6. provide finite simulated-observation scripts for an unloaded motor.

The default mode remains Mode 18 for safety. Select Mode 19 or Mode 21 explicitly in `src/app/app_config.h`.

## Frozen flags

```text
bit0 found
bit1 valid
bit2 predicted
bit3 emergency_disable
bits4..7 invalid_reason
```

`0x14` and `0xB8` are valid flags. `valid=0` means position and velocity are not used for control. Emergency has highest priority and latches until reset.

## Mode 21 behavior

- manually place the mechanism at physical 0 degrees before reset;
- startup moves to the calibrated horizontal point at -28.1 degrees;
- three consecutive control-valid observations are required before ACTIVE;
- PD output is limited to +/-200 mdeg around horizontal;
- short stale data returns target offset to zero;
- link timeout after 600 ms disables the motor;
- three new valid frames can re-enable after a non-emergency timeout;
- emergency disables immediately and cannot be cleared by normal frames;
- motor commands are latest-only; busy periods do not queue old targets.

## Safety

Run live tests without the ball. Keep hands and wiring clear. Run `motion-demo` first. Run `emergency-once` last because reset is required afterward.
