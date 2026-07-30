# v0.1.1 硬件标定测试补丁应用说明

本压缩包内的路径与仓库根目录一致。请先在 Windows 本地仓库确认工作区干净，然后把压缩包内容覆盖到仓库根目录。

```powershell
git status --short
```

覆盖后应新增/修改：

```text
src/app/app_config.h
src/app/app_main.c
src/motor/emm_v5_driver.c
src/motor/emm_v5_driver.h
src/motor/emm_v5_protocol.c
src/motor/emm_v5_protocol.h
src/test_modes/test_console.c
src/test_modes/test_console.h
src/test_modes/emm_position_calibration_test.c
src/test_modes/emm_position_calibration_test.h
src/test_modes/motor_sign_test.c
src/test_modes/motor_sign_test.h
keil/BallBalanceControl_MSPM0_v0_1.uvprojx
docs/HARDWARE_CALIBRATION_TESTS.md
host_tests/test_protocol.c
host_tests/run_host_tests.sh
README.md
```

在 Keil 打开原有 `.uvprojx`，应看到新增 `TestModes` 组和 Motor 组中的 `emm_v5_protocol.c`。

默认模式仍是 `APP_MODE_PROTOCOL_LOOPBACK`，覆盖补丁后直接烧录不会移动电机。按文档逐次切换到：

```c
APP_MODE_EMM_POSITION_CALIBRATION
APP_MODE_MOTOR_SIGN_TEST
```

不要同时开启两个模式。
