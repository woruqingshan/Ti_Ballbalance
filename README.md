# BallBalanceControl_MSPM0_v0_1

MSPM0G3507 + Emm V5.0 的基于视觉滚球控制 Keil 工程初版。

## v0.1 边界

- TI 板是任务状态、计时、控制和电机命令的唯一实时主控。
- Raspberry Pi 仅发送最新小球视觉状态，并接收任务事件/控制遥测。
- 官方限制下不使用摆杆外部角度传感器；闭环为“小球视觉位置闭环”。
- Emm V5.0 内部编码器仅保证电机轴跟随，不视作摆杆角度反馈。
- VehicleStub 固定报告车辆静止；循迹、MPU、车辆前馈和 Task 4～6 仅保留接口。
- 默认编译模式为 `APP_MODE_PROTOCOL_LOOPBACK`，上电不会驱动步进电机。

## 固定开发环境

- MCU：MSPM0G3507
- Keil MDK 5，Arm Compiler 6
- SDK：`D:\TI\M0_SDK\mspm0_sdk_2_02_00_05`
- UART0：PA10 TX / PA11 RX，115200 8N1，Pi/CH340
- UART1：PB4 TX / PB5 RX，115200 8N1，Emm V5.0
- 电机地址：0x01；校验尾字节：0x6B；16 细分，3200 pulse/rev

## 打开工程

打开：

`keil\BallBalanceControl_MSPM0_v0_1.uvprojx`

工程引用固定 SDK 路径，不把 TI DriverLib、CMSIS 和启动文件复制进仓库。

首次打开前确认：

1. SDK 位于上述路径；
2. Keil 已安装 MSPM0G1X0X_G3X0X Device Family Pack；
3. Keil 使用 Arm Compiler 6；
4. Debug/Flash 选择板上 XDS110 CMSIS-DAP 或实际使用的调试器。

## 安全启动

默认 `APP_MODE_PROTOCOL_LOOPBACK`：只解析 UART0 协议并回传状态，不使能电机。

修改 `src/app/app_config.h` 中 `APP_MODE` 后才进入其他模式：

- `APP_MODE_EMM_ENABLE_TEST`
- `APP_MODE_EMM_STEP_SWEEP`
- `APP_MODE_SIMULATED_BALL_CONTROL`
- `APP_MODE_TASK3_DRY_RUN`
- `APP_MODE_TASK3_LIVE`
- `APP_MODE_EMM_POSITION_CALIBRATION`
- `APP_MODE_MOTOR_SIGN_TEST`

`APP_MODE_TASK3_LIVE` 还要求将 `APP_EMM_COMMAND_SEMANTICS_VERIFIED` 设为 1。只有完成驱动器覆盖/排队/反向/停止语义实验后才允许这样做。

## 当前实现

- 两路 UART 板级初始化；
- UART0 中断接收环形缓冲；
- 版本化二进制协议、CRC16、latest-value BallState；
- Emm V5.0 使能/失能/相对位置指令；
- Emm V5.0 `0x36` 实时位置查询、返回帧解析和超时统计；
- 人工拖动机构的编码器 MIN/MAX 标定模式；
- 交互式正负方向微动测试模式；
- 软件绝对目标位置、软限位、命令节流和最新目标合并；
- 小球 PD、限幅、变化率限制；
- Task 3 状态机与 VehicleStub；
- 视觉超时、协议错误、软限位和任务超时故障；
- TaskEvent 和 ControlTelemetry 回传；
- Python 协议模拟器与主机侧纯算法测试。

## 重要限制

本包在当前环境中无法调用 Windows Keil、用户本地 SDK 或真实 MSPM0G3507 硬件完成最终编译和烧录验证。工程文件、SDK 路径和 DriverLib 用法按 TI 官方 Keil 示例组织，并完成了主机侧协议/控制模块测试。首次本地构建后，应先使用默认安全模式，再按 `docs/BRINGUP_AND_FLASH.md` 分阶段验证。


## v0.1.1 硬件标定测试

详细步骤见 `docs/HARDWARE_CALIBRATION_TESTS.md`。两个测试模式均通过 UART0 输出 ASCII 文本，串口助手使用 115200 8N1，关闭 HEX 发送并使用字符发送。
