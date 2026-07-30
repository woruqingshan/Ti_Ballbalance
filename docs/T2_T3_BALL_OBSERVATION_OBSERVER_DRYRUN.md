# T2/T3 BallObservation observer and control dry-run

Baseline: `Ti_Ballbalance` commit `8fe3043` (`feat(link): add T1 BallObservation receive diagnostic`).

## 1. Scope

### T2 / Mode 18

- reuse UART0 IRQ + ring-buffer receive path;
- decode the existing 14-byte BallObservation protocol;
- evaluate whether the newest observation is eligible for control;
- calculate total age = `vision_age_ms + elapsed since TI reception`;
- expose raw receive and observer state through Keil Watch;
- no UART0 TX, no ACK, no heartbeat;
- no UART1/Emm/motor initialization or movement.

### T3 / Mode 19

- reuse exactly the same receive, parser, latest-only and gate path;
- require three consecutive control-valid observations before ACTIVE;
- compute a fixed-point PD target in motor millidegrees;
- apply amplitude and dt-aware slew limits;
- reset output on invalid/stale/link-timeout observations;
- latch emergency in software until Reset;
- no UART1/Emm/motor initialization or movement.

## 2. Coordinate and validation contract

- screen right / vehicle-tail direction: positive position;
- screen left: negative position;
- positive velocity means position is increasing;
- accepted protocol frame is separate from control-valid observation;
- `valid=false` remains a legal frame but cannot enter control;
- predicted frames are rejected by default;
- position control range: `[-13.00, +13.00] cm`;
- absolute velocity limit: `50.00 cm/s`;
- minimum confidence: `350/1000`;
- maximum total age: `120 ms`;
- no new sequence for `>600 ms`: link timeout;
- duplicate sequence does not refresh latest observation age/version.

Gate reason values:

```text
0 VALID
1 NO_OBSERVATION
2 EMERGENCY
3 FLAG_INVALID
4 NOT_FOUND
5 PREDICTED_DISALLOWED
6 LOW_CONFIDENCE
7 POSITION_RANGE
8 VELOCITY_RANGE
9 STALE
```

Mode 19 state values:

```text
0 WAITING
1 REACQUIRING
2 ACTIVE
3 STALE
4 LINK_TIMEOUT
5 EMERGENCY_LATCHED
```

## 3. Files

Modified:

- `src/app/app_config.h`
- `src/app/app_main.c`
- `src/test_modes/ball_observation_rx_diag.h`

Added:

- `src/communication/ball_observation_stream.h`
- `src/control/ball_observation_gate.h`
- `src/control/ball_position_controller.h`
- `src/test_modes/ball_control_dry_run_test.h`
- `host_tests/test_ball_observation_t23.c`
- `host_tests/run_ball_observation_t23_tests.bat`
- `tools/ball_observation_t23_sender.py`
- `tools/verify_t23_ball_observation_patch.ps1`
- this document and patch manifest/readme.

Deleted: none.

No Keil project, `uvoptx`, J-Link, Flash Algorithm, Objects or Listings file is included.

## 4. Apply and software verification

Overlay the patch contents into the repository root, then run:

```cmd
powershell -ExecutionPolicy Bypass -File .\tools\verify_t23_ball_observation_patch.ps1
D:\Anaconda\python.exe tools\ball_observation_t1_sender.py self-test
host_tests\run_ball_observation_t23_tests.bat
```

Expected:

```text
T2/T3 BallObservation patch file check: PASS
BallObservation Python self-test: PASS
BallObservation T2/T3 host tests: PASS
```

The host C test requires GCC in PATH. Its absence does not prevent Keil hardware testing.

## 5. T2 / Mode 18 observer test

Set:

```c
#define APP_MODE APP_MODE_BALL_OBSERVATION_RX_DIAG
```

Keil:

```text
Project -> Clean Targets
Project -> Rebuild all target files
0 Error(s)
Flash -> Download
Reset
Debug -> Run
```

Wiring:

```text
CH340 TX  -> TI PA11 / UART0_RX
CH340 GND -> TI GND
```

Close the serial assistant, then run:

```cmd
D:\Anaconda\python.exe tools\ball_observation_t23_sender.py observer-demo --port COM9
```

One complete demo sends 57 unique valid-protocol frames / 798 bytes. Expected raw receive state:

```text
rx_bytes          = 798
completed_frames  = 57
frames_ok         = 57
latest_version    = 57
crc_errors        = 0
flag_errors       = 0
confidence_errors = 0
sequence_gaps     = 0
duplicate_frames  = 0
ring_overflows    = 0
```

Observer Watch symbols:

```text
g_ball_observation_observer_stats.control_valid
g_ball_observation_observer_stats.invalid_reason
g_ball_observation_observer_stats.total_age_ms
g_ball_observation_observer_stats.position_centi_cm
g_ball_observation_observer_stats.velocity_centi_cm_s
g_ball_observation_observer_stats.confidence_milli
g_ball_observation_observer_stats.flags
g_ball_observation_observer_stats.state_changes
```

The demo exercises valid center/right/left, low confidence, predicted, position range, velocity range, stale source age and valid recovery. Final expected state:

```text
control_valid = 1
invalid_reason = 0 (VALID)
position = 0
velocity = 0
```

The motor must remain completely stationary.

For a 40 Hz continuity test:

```cmd
D:\Anaconda\python.exe tools\ball_observation_t23_sender.py stream --port COM9 --rate 40 --duration 600 --amplitude 500 --period 4
```

After 10 minutes, `frames_ok` should approximately match sender `sent`; protocol errors, gaps, duplicates and overflow should remain zero.

### Real Raspberry Pi observer acceptance

Once the Raspberry Pi observation sender is integrated, connect:

```text
Pi GPIO14 TX / Pin 8 -> TI PA11 RX
Pi GND / Pin 6       -> TI GND
```

Mode 18 must show:

- center near `position=0`;
- vehicle-tail/right movement positive;
- left movement negative;
- velocity sign follows position change;
- lost/blocked ball produces control invalid;
- real stream remains stable at about 40 Hz;
- motor remains stationary.

## 6. T3 / Mode 19 controller dry-run

Set:

```c
#define APP_MODE APP_MODE_BALL_CONTROL_DRY_RUN
```

Clean, rebuild, download, Reset and Run again.

Watch:

```text
g_ball_control_dry_run_stats.mode
g_ball_control_dry_run_stats.invalid_reason
g_ball_control_dry_run_stats.reacquire_count
g_ball_control_dry_run_stats.control_valid
g_ball_control_dry_run_stats.total_age_ms
g_ball_control_dry_run_stats.error_centi_cm
g_ball_control_dry_run_stats.raw_output_mdeg
g_ball_control_dry_run_stats.amplitude_limited_mdeg
g_ball_control_dry_run_stats.target_offset_mdeg
g_ball_control_dry_run_stats.control_updates
g_ball_control_dry_run_stats.stale_events
g_ball_control_dry_run_stats.link_timeout_events
g_ball_control_dry_run_stats.emergency_latched
```

Run:

```cmd
D:\Anaconda\python.exe tools\ball_observation_t23_sender.py dry-run-demo --port COM9
```

Expected:

- 60 frames / 840 bytes accepted without protocol errors;
- first three valid frames move state through REACQUIRING to ACTIVE;
- right / vehicle-tail positive position gives negative error;
- with `motor_sign=+1`, right positive gives negative target offset;
- left negative gives positive target offset;
- target offset never exceeds `+/-200 mdeg`;
- target changes by at most about `20 mdeg` per 25 ms frame with the default 800 mdeg/s slew;
- invalid observation resets target offset to zero;
- final valid center sequence returns ACTIVE with zero output;
- motor remains stationary.

Timeout test:

```cmd
D:\Anaconda\python.exe tools\ball_observation_t23_sender.py timeout-demo --port COM9
```

Expected sequence:

```text
ACTIVE
pause 200 ms -> STALE, target_offset_mdeg=0
three valid frames -> ACTIVE again
pause 700 ms -> LINK_TIMEOUT, target_offset_mdeg=0
```

Emergency-latch test must be run last or after a fresh Reset:

```cmd
D:\Anaconda\python.exe tools\ball_observation_t23_sender.py emergency-once --port COM9
```

Expected:

```text
mode = 5 (EMERGENCY_LATCHED)
emergency_latched = 1
target_offset_mdeg = 0
```

Normal observations do not clear this state; Reset is required. The motor remains stationary because Mode 19 is compute-only.

## 7. Initial dry-run controller

Fixed-point formula:

```text
error_centi_cm = target_centi_cm - position_centi_cm
raw_mdeg = motor_sign *
           (Kp_mdeg_per_cm * error_centi_cm
            - Kd_mdeg_per_cm_s * velocity_centi_cm_s) / 100
```

Defaults:

```text
target = 0 cm
Kp = 60 mdeg/cm
Kd = 15 mdeg/(cm/s)
maximum offset = +/-200 mdeg
maximum slew = 800 mdeg/s
motor_sign = +1
```

These values verify signs, gate behavior, saturation and timing only. They are not live-ball tuning values.

## 8. Safety boundary

Modes 18 and 19 do not call:

- `emm_v5_init`, `emm_v5_poll` or Emm commands;
- `rod_motor_control_startup_to_horizontal`;
- `rod_motor_control_set_target_offset_mdeg`;
- UART1 motor transmission;
- heartbeat, ACK or periodic UART0 TX.

No motor movement in either stage is the required result.
