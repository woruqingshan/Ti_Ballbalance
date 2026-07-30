# T1 BallObservation receive-only bring-up

## 1. Files

Added:

- `src/communication/ball_observation_protocol.h`
- `src/test_modes/ball_observation_rx_diag.h`
- `host_tests/test_ball_observation_protocol.c`
- `host_tests/run_ball_observation_protocol_test.bat`
- `tools/ball_observation_t1_sender.py`
- `tools/verify_t1_ball_observation_patch.ps1`
- `docs/T1_BALL_OBSERVATION_RX_BRINGUP.md`
- `PATCH_README.md`

Modified:

- `src/app/app_config.h`
- `src/app/app_main.c`

Deleted: none.

## 2. Compile and flash

`app_config.h` defaults to:

```c
#define APP_MODE APP_MODE_BALL_OBSERVATION_RX_DIAG
```

In Keil:

```text
Project -> Clean Targets
Project -> Rebuild all target files
Confirm 0 Error(s)
Flash -> Download
Reset
```

Mode 18 sends no UART output. This is intentional.

## 3. Wiring

For CH340 testing:

```text
CH340 TX  -> TI PA11 / UART0_RX
CH340 GND -> TI GND
```

`CH340 RX <- TI PA10` is not required for T1 because TI never transmits.
Do not connect CH340 5 V. Ensure the ground wire is firm.

Serial configuration:

```text
115200 baud, 8 data bits, no parity, 1 stop bit, no flow control
```

Close any serial assistant before running the Python sender.

## 4. Keil Watch

Start a debug session and add:

```text
g_ball_observation_rx_stats.rx_bytes
g_ball_observation_rx_stats.completed_frames
g_ball_observation_rx_stats.frames_ok
g_ball_observation_rx_stats.crc_errors
g_ball_observation_rx_stats.flag_errors
g_ball_observation_rx_stats.confidence_errors
g_ball_observation_rx_stats.sequence_gaps
g_ball_observation_rx_stats.duplicate_frames
g_ball_observation_rx_stats.ring_overflows
g_ball_observation_rx_stats.latest_version
g_ball_observation_rx_stats.latest_position_centi_cm
g_ball_observation_rx_stats.latest_velocity_centi_cm_s
g_ball_observation_rx_stats.latest_confidence_milli
g_ball_observation_rx_stats.latest_vision_age_ms
g_ball_observation_rx_stats.latest_sequence
g_ball_observation_rx_stats.latest_flags
```

Run the CPU; do not leave it halted while sending frames.

## 5. Software self-test

```powershell
python .\tools\ball_observation_t1_sender.py self-test
```

Expected:

```text
BallObservation Python self-test: PASS
golden frame: a5 5a ...
```

Optional GCC host test:

```cmd
host_tests\run_ball_observation_protocol_test.bat
```

Expected:

```text
BallObservation protocol host tests: PASS
```

## 6. Finite hardware demo

Install pyserial in the active Python environment:

```powershell
python -m pip install pyserial
```

Replace `COM9` with the CH340 port:

```powershell
python .\tools\ball_observation_t1_sender.py demo --port COM9
```

Expected Watch totals after one run from a reset/zeroed stats state:

```text
rx_bytes             = 154
completed_frames     = 11
frames_ok            = 8
crc_errors           = 1
flag_errors          = 1
confidence_errors    = 1
duplicate_frames     = 1
sequence_gaps        = 2
latest_version       = 7
latest_sequence      = 8
latest_flags         = 0x08
emergency_frames     = 1
ring_overflows       = 0
```

The latest accepted emergency frame contains zero position/velocity and is only
recorded in T1. It does not control or disable a motor because Mode 18 never
initializes motor business logic.

The motor must remain completely stationary throughout T1.

## 7. 40 Hz soak test

Reset TI and zero/reopen the Watch view, then run:

```powershell
python .\tools\ball_observation_t1_sender.py soak `
  --port COM9 `
  --rate 40 `
  --duration 600
```

Expected after 10 minutes:

- `frames_ok` and `latest_version` approximately equal the sender `sent` count;
- `crc_errors = 0`;
- `flag_errors = 0`;
- `confidence_errors = 0`;
- `sequence_gaps = 0`;
- `duplicate_frames = 0`;
- `ring_overflows = 0`;
- latest position oscillates between approximately -500 and +500;
- latest velocity changes sign consistently with position direction;
- MCU remains running and motor remains stationary.

## 8. Failure interpretation

- `rx_bytes = 0`: wrong COM port, PA11 wiring, no common ground, CPU halted or wrong firmware.
- `rx_bytes > 0`, `frames_ok = 0`, CRC grows: baud/wiring/ground mismatch or sender protocol mismatch.
- `flag_errors > 0`: sender used bits 4..7.
- `confidence_errors > 0`: sender sent confidence greater than 1000.
- `ring_overflows > 0`: CPU halted too long, sender rate excessive or UART ring not drained.
- Watch values do not update while Debug is paused: resume CPU; IRQ processing cannot continue while halted.
