$ErrorActionPreference = "Stop"
$required = @(
  "src/app/app_config.h",
  "src/app/app_main.c",
  "src/communication/vision_link.c",
  "src/communication/vision_link.h",
  "src/protocol/protocol.h",
  "src/protocol/pi_ti_messages.h",
  "src/control/vision_control_preview.h",
  "src/test_modes/pi_ti_manual_link_test.h",
  "src/test_modes/ball_state_link_test.h",
  "tools/pi_ti_stage567_tool.py",
  "host_tests/test_pi_ti_messages.c",
  "docs/PHASE_5_6_7_TESTS.md"
)
foreach ($file in $required) {
  if (-not (Test-Path $file)) { throw "Missing: $file" }
}
$projectChanges = git status --short -- "keil/*.uvprojx" "keil/*.uvoptx"
if ($projectChanges) {
  Write-Warning "Keil project/user files have local changes. This patch does not require them."
  $projectChanges | Write-Host
}
Write-Host "Phase 5/6/7 patch file check: PASS"
Write-Host "No Keil project file is included in the patch."
