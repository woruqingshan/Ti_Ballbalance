$Sdk = 'D:\TI\M0_SDK\mspm0_sdk_2_02_00_05'
$Paths = @(
  "$Sdk\source\third_party\CMSIS\Core\Include\core_cm0plus.h",
  "$Sdk\source\ti\driverlib\lib\keil\m0p\mspm0g1x0x_g3x0x\driverlib.a",
  "$Sdk\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s"
)
$ok = $true
foreach ($p in $Paths) { if (Test-Path $p) { Write-Host "[OK] $p" } else { Write-Host "[MISSING] $p" -ForegroundColor Red; $ok=$false } }
if (-not $ok) { exit 1 }
Write-Host '[OK] SDK path is ready for the Keil project.'
