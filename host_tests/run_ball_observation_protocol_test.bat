@echo off
setlocal
cd /d "%~dp0\.."

where gcc >nul 2>nul
if errorlevel 1 (
  echo ERROR: gcc was not found in PATH.
  echo Install MinGW/MSYS2 GCC or run the protocol self-test with the Python tool.
  exit /b 1
)

if not exist build mkdir build

gcc -std=c11 -Wall -Wextra -Werror ^
  -Isrc ^
  host_tests\test_ball_observation_protocol.c ^
  src\protocol\crc16.c ^
  -o build\test_ball_observation_protocol.exe
if errorlevel 1 exit /b 1

build\test_ball_observation_protocol.exe
exit /b %errorlevel%
