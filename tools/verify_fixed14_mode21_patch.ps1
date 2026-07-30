$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$required = @(
  "src/communication/ball_observation_protocol.h",
  "src/communication/ball_observation_stream.h",
  "src/control/ball_observation_gate.h",
  "src/test_modes/ball_observation_rx_diag.h",
  "src/test_modes/ball_control_dry_run_test.h",
  "src/app/app_config.h",
  "src/app/app_main.c",
  "src/app/pi_ball_observation_control_app.h",
  "host_tests/test_ball_observation_protocol.c",
  "host_tests/test_ball_observation_t23.c",
  "host_tests/test_fixed14_mode21.c",
  "host_tests/run_fixed14_mode21_tests.bat",
  "tools/fixed14_protocol.py",
  "tools/test_fixed14_protocol_compat.py",
  "tools/test_ball_control_pipeline.py",
  "tools/test_ball_observation_motor_live.py",
  "docs/FIXED14_MODE21_BRINGUP.md"
)
foreach ($path in $required) {
  $full = Join-Path $root $path
  if (-not (Test-Path $full)) {
    throw "Missing required file: $path"
  }
}
$forbidden = Get-ChildItem -Path $root -Recurse -File | Where-Object {
  $_.Extension -in @(".uvprojx", ".uvoptx") -or
  $_.FullName -match "\\(Objects|Listings)\\" -or
  $_.Name -match "JLink"
}
if ($forbidden) {
  $forbidden | ForEach-Object { Write-Host "Forbidden: $($_.FullName)" }
  throw "Patch contains local Keil/J-Link artifacts"
}
Write-Host "Fixed-14 + Mode21 patch file check: PASS"
