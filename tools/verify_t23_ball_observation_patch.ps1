$ErrorActionPreference = "Stop"

$required = @(
    "src/app/app_config.h",
    "src/app/app_main.c",
    "src/communication/ball_observation_stream.h",
    "src/control/ball_observation_gate.h",
    "src/control/ball_position_controller.h",
    "src/test_modes/ball_observation_rx_diag.h",
    "src/test_modes/ball_control_dry_run_test.h",
    "host_tests/test_ball_observation_t23.c",
    "host_tests/run_ball_observation_t23_tests.bat",
    "tools/ball_observation_t23_sender.py",
    "tools/verify_t23_ball_observation_patch.ps1",
    "docs/T2_T3_BALL_OBSERVATION_OBSERVER_DRYRUN.md",
    "PATCH_T23_README.md",
    "PATCH_T23_MANIFEST.txt"
)

foreach ($path in $required) {
    if (-not (Test-Path $path)) {
        throw "Missing required T2/T3 file: $path"
    }
}

$forbidden = Get-ChildItem -Recurse -File | Where-Object {
    $_.Name -match '\.(uvprojx|uvoptx)$' -or
    $_.FullName -match '[\\/](Objects|Listings)[\\/]'
}
if ($forbidden) {
    Write-Host "WARNING: repository contains local Keil/generated files; the patch itself does not require them."
}

$config = Get-Content "src/app/app_config.h" -Raw
if ($config -notmatch 'APP_MODE_BALL_OBSERVATION_RX_DIAG') {
    throw "Mode 18 definition missing"
}
if ($config -notmatch 'APP_MODE_BALL_CONTROL_DRY_RUN') {
    throw "Mode 19 definition missing"
}
if ($config -notmatch 'APP_BALL_OBS_ALLOW_PREDICTED\s+0U') {
    throw "Predicted-frame default must remain disabled for T2/T3"
}

Write-Host "T2/T3 BallObservation patch file check: PASS"
Write-Host "Default mode is Mode 18 observer; change APP_MODE to Mode 19 for dry-run."
Write-Host "Modes 18 and 19 must keep all motor/UART1 output disabled."
