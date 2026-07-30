@echo off
setlocal
python tools\pi_ti_stage567_tool.py self-test
if errorlevel 1 exit /b 1
where gcc >nul 2>nul
if errorlevel 1 (
  echo GCC not found; Python protocol self-test passed. C host test skipped.
  exit /b 0
)
gcc -std=c11 -Wall -Wextra -Werror -Isrc host_tests\test_pi_ti_messages.c src\protocol\protocol.c src\protocol\crc16.c -o host_tests\test_pi_ti_messages.exe
if errorlevel 1 exit /b 1
host_tests\test_pi_ti_messages.exe
if errorlevel 1 exit /b 1
echo Phase 5/6/7 host tests: PASS
