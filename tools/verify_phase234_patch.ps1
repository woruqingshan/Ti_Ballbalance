$ErrorActionPreference = "Stop"

$required = @(
    "src\app\app_config.h",
    "src\app\app_main.c",
    "src\motor\rod_motor_math.h",
    "src\motor\rod_motor_control.h",
    "src\test_modes\motor_manual_control_test.h",
    "src\test_modes\motor_command_semantics_test.h",
    "docs\PHASE_2_3_4_TESTS.md"
)

foreach ($path in $required) {
    if (-not (Test-Path $path)) {
        throw "Missing required patch file: $path"
    }
}

$projectFiles = Get-ChildItem -Recurse -File -Include *.uvprojx,*.uvoptx |
    Where-Object { $_.FullName -like "*Phase2_3_4_Patch*" }

if ($projectFiles) {
    throw "Patch unexpectedly contains Keil project/user files."
}

$config = Get-Content "src\app\app_config.h" -Raw
if ($config -notmatch "APP_MODE_ROD_MOTOR_MANUAL_CONTROL") {
    throw "Manual-control mode is missing from app_config.h"
}
if ($config -notmatch "APP_MODE_MOTOR_COMMAND_SEMANTICS_TEST") {
    throw "Command-semantics mode is missing from app_config.h"
}

Write-Host "Phase 2/3/4 patch file check: PASS"
Write-Host "Keil project file was not replaced by this patch."
Write-Host "Next: Clean Targets -> Rebuild -> Flash Download."
