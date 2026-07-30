# 系统架构 v0.1

```text
Raspberry Pi 5
  BALL_STATE(position, velocity, valid, age)
             | UART0 115200
             v
MSPM0G3507
  VisionLink -> SafetySupervisor -> Task3Controller
                                  -> BallPositionController
                                  -> CommandProfile
                                  -> RodMotorMapper
                                  -> MotorPositionManager
                                  -> MotorCommandScheduler
                                  -> EmmV5Driver -> UART1
  VehicleControlPort -> VehicleStub(stationary=true)
             |
             +---- TASK_EVENT / CONTROL_TELEMETRY ----> Pi
```

数据所有权：

- TI：任务状态、正式计时、目标位置、控制参数、电机软件位置和故障标志。
- Pi：视觉测量、视觉质量、视频和回放。
- BallState 为 latest-value；任务事件不能被普通遥测覆盖。

默认安全：上电不使能电机；通信恢复后不自动恢复已中断任务。
