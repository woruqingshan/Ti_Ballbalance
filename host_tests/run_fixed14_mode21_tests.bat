@echo off
setlocal
cd /d "%~dp0\.."

where gcc >nul 2>nul
if errorlevel 1 (
  echo ERROR: gcc was not found in PATH.
  echo Use MSYS2/MinGW GCC, or continue with Keil hardware testing.
  exit /b 1
)

if not exist build mkdir build

gcc -std=c11 -Wall -Wextra -Werror ^
  -Isrc ^
  host_tests\test_fixed14_mode21.c ^
  src\protocol\crc16.c ^
  -o build\test_fixed14_mode21.exe
if errorlevel 1 exit /b 1

build\test_fixed14_mode21.exe
exit /b %errorlevel%
