# Keil 打开、编译、烧录与上电验证

## 1. 环境检查

确认存在：

```powershell
Test-Path D:\TI\M0_SDK\mspm0_sdk_2_02_00_05\source\ti\driverlib\lib\keil\m0p\mspm0g1x0x_g3x0x\driverlib.a
Test-Path D:\TI\M0_SDK\mspm0_sdk_2_02_00_05\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s
```

如果第二个文件名在本机 SDK 中略有不同，打开工程后只需在 `Startup` 组替换成 SDK 中对应的 MSPM0G350X Keil 启动文件。

## 2. 打开与编译

1. 打开 `keil/BallBalanceControl_MSPM0_v0_1.uvprojx`。
2. Project → Options for Target → Target：设备 MSPM0G3507。
3. C/C++：Arm Compiler 6，宏 `__MSPM0G3507__`。
4. Linker：确认 DriverLib 路径和 `mspm0g3507.sct`。
5. Rebuild，预期生成 `Objects/BallBalanceControl_MSPM0_v0_1.axf` 和 HEX。

## 3. 下载设置

- 板载 LaunchPad：Debug 选择 CMSIS-DAP，调试器通常为 XDS110。
- 自制板：选择实际 J-Link/XDS 调试器，SWDIO/SWCLK/GND/RESET 正确连接。
- Utilities → Use Target Driver for Flash Programming。
- Flash Download 中确认 MSPM0G3507 对应 Flash Algorithm。

## 4. 第一次烧录

保持默认：

```c
#define APP_MODE APP_MODE_PROTOCOL_LOOPBACK
#define APP_EMM_COMMAND_SEMANTICS_VERIFIED 0
```

此模式不会使能或移动电机。使用 CH340 接 UART0：

- PA10 → CH340 RX
- PA11 ← CH340 TX
- GND 共地
- 115200 8N1

运行 `tools/pi_protocol_simulator.py --port COMx --loopback`，观察状态帧。

## 5. 电机分阶段验证

接线：

- PB4/UART1_TX → Emm RX
- PB5/UART1_RX ← Emm TX（建议接入）
- MCU GND 与驱动器 GND 共地

依次启用：

1. `APP_MODE_EMM_ENABLE_TEST`
2. `APP_MODE_EMM_STEP_SWEEP`
3. 命令语义人工实验
4. `APP_MODE_TASK3_DRY_RUN`
5. `APP_MODE_TASK3_LIVE`

只有确认新运动命令的覆盖/排队/反向/停止行为后，才将：

```c
#define APP_EMM_COMMAND_SEMANTICS_VERIFIED 1
```

## 6. 本包无法替代的本地验收

- Keil/SDK 真实编译；
- Flash Algorithm 与调试器配置；
- UART 引脚是否与用户实际板卡原理图一致；
- Emm V5.0 动态命令语义；
- 机械零点、软限位和脉冲映射；
- 真实球和视觉闭环参数。
