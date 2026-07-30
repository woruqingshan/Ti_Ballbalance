$ErrorActionPreference = "Stop"

$required = @(
    "src/app/app_config.h",
    "src/app/app_main.c",
    "src/communication/ball_observation_protocol.h",
    "src/test_modes/ball_observation_rx_diag.h",
    "host_tests/test_ball_observation_protocol.c",
    "host_tests/run_ball_observation_protocol_test.bat",
    "tools/ball_observation_t1_sender.py",
    "docs/T1_BALL_OBSERVATION_RX_BRINGUP.md",
    "PATCH_README.md"
)

foreach ($path in $required) {
    if (-not (Test-Path $path)) {
        throw "Missing required patch file: $path"
    }
}

$config = Get-Content "src/app/app_config.h" -Raw
$main = Get-Content "src/app/app_main.c" -Raw
$protocol = Get-Content "src/communication/ball_observation_protocol.h" -Raw

if ($config -notmatch "APP_MODE_BALL_OBSERVATION_RX_DIAG\s+18") {
    throw "Mode 18 definition was not found"
}
if ($config -notmatch "#define APP_MODE APP_MODE_BALL_OBSERVATION_RX_DIAG") {
    throw "T1 is not the selected APP_MODE"
}
if ($main -notmatch "ball_observation_rx_diag_run") {
    throw "app_main.c does not dispatch the T1 receive diagnostic"
}
if ($protocol -notmatch "BALL_OBSERVATION_FRAME_SIZE\s+14U") {
    throw "14-byte BallObservation protocol definition was not found"
}

$forbidden = Get-ChildItem -Recurse -File | Where-Object {
    $_.Name -match "\.(uvprojx|uvoptx)$" -or
    $_.FullName -match "JLinkLog|\\Objects\\|\\Listings\\"
}
if ($forbidden) {
    throw "Forbidden Keil/J-Link generated file included: $($forbidden.FullName -join ', ')"
}

Write-Host "T1 BallObservation patch file check: PASS"
Write-Host "No Keil project, J-Link, Objects, Listings or uvoptx file is included."
