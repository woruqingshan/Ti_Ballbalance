# 阶段 5、6、7：Pi-TI 通信、模拟小球与真实视觉观察

## 1. 基线和安全边界

- 仓库：`woruqingshan/Ti_Ballbalance`
- 基线提交：`dc44a25c76d74dcab664354b82a70f54cd3b392a`
- 分支：`master`
- UART0：PA10 TX / PA11 RX，115200 8N1，连接 Windows CH340 或 Raspberry Pi
- UART1：PB4 TX / PB5 RX，115200 8N1，连接 Emm V5.0
- 电机物理范围：约 `[-70°, 0°]`
- 水平工作点：`-28.1°`
- 初期控制窗口：`[-31.1°, -25.1°]`
- 本补丁不包含、也不修改任何 `uvprojx/uvoptx`，保留已经验证过的 J-Link 与 Flash 配置。

每次执行会真实移动电机的 `STARTUP_HORIZONTAL` 前，必须人工把机构放在定义的物理 `0°` 起始姿态，并取下钢球。

---

## 2. 协议帧

统一帧结构：

```text
A5 5A | version | type | payload_len_le | sequence_le | timestamp_ms_le | payload | crc16_le
```

- `version = 1`
- CRC：CRC16-CCITT-FALSE，计算范围从 `version` 到 payload 末尾
- UART：115200、8N1、无硬件奇偶校验
- 多字节数值：小端

### 2.1 树莓派/电脑到 TI

- `0x02 PI_STATUS`：链路心跳
- `0x10 CONTROL_COMMAND`：8 字节
  - command：u8
  - argument：u8
  - value：i16
  - token：u32
- `0x01 BALL_STATE`：16 字节
  - frame_id：u32
  - position：i16，单位 0.01 cm
  - velocity：i16，单位 0.01 cm/s
  - confidence：u16，0-1000
  - vision_latency_ms：u16
  - flags：bit0 found，bit1 predicted，bit2 valid
  - invalid_reason：u8
  - reserved：u16

### 2.2 TI 到树莓派/电脑

- `0x84 COMMAND_ACK`
- `0x85 MOTOR_STATUS`
- `0x86 LINK_STATS`
- `0x87 BALL_PREVIEW`
- `0x83 HEARTBEAT`

命令 token 用于防止串口重试导致同一运动命令重复执行。TI 对相同 token 和相同 command 只重发上一次结果，不重复动作。

---

## 3. 覆盖和构建

1. 关闭 Keil。
2. 把补丁目录内容覆盖到仓库根目录。
3. 运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\verify_phase567_patch.ps1
python .\tools\pi_ti_stage567_tool.py self-test
```

4. 打开原 Keil 工程。
5. `Project -> Clean Targets`。
6. `Project -> Rebuild all target files`。
7. 确认 `0 Error(s)`。
8. `Flash -> Download`。
9. Reset 板卡。

若 `keil/*.uvprojx` 出现变化，本补丁不需要这些变化，不应因为本补丁重新生成工程文件。

---

## 4. 阶段 5：树莓派/电脑手动控制 TI

默认模式：

```c
#define APP_MODE APP_MODE_PI_TI_MANUAL_LINK
```

上电后：

- UART0 进入纯二进制协议模式；
- 电机立即失能；
- 不自动清零、不自动移动；
- 超过 800 ms 未收到合法协议帧时，TI 自动失能电机。

### 4.1 接线

```text
Raspberry Pi / CH340 TX -> TI PA11 UART0_RX
Raspberry Pi / CH340 RX <- TI PA10 UART0_TX
GND                     <-> TI GND

TI PB4 UART1_TX -> Emm RX
TI PB5 UART1_RX <- Emm TX
TI GND          <-> Emm GND
```

不要把树莓派 TX 和 CH340 TX 同时接到 PA11。调试时可只把 CH340 RX 并接到 PA10，用于单向监听，但不能在二进制链路中混入 ASCII。

### 4.2 Windows 测试

安装：

```powershell
python -m pip install pyserial
```

查询：

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 hello
python tools\pi_ti_stage567_tool.py --port COM5 status
```

人工把机构放到物理 0°，推荐使用同一串口会话完成完整动作序列：

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 manual-demo `
  --offsets 200 0 -500 0 `
  --dwell 1.0
```

程序在一个连接内依次执行 HELLO、STARTUP、偏移往返、回水平和失能，并持续发送链路心跳。不要把启动、使能和偏移拆成间隔很长的独立进程；TI 在 800 ms 无合法帧后会自动失能。

`offset` 单位是毫度：

- `200` = 相对水平点 +0.2°
- `-500` = 相对水平点 -0.5°

### 4.3 阶段 5 验收

- HELLO、STATUS 均收到 ACK 和 MOTOR_STATUS；
- STARTUP 只执行一次，结束位置约 -28.1°；
- ±0.2°、±0.5°、±1.0°命令方向正确；
- 超过 ±3°的 offset 被拒绝；
- 相同 token 重发不会重复移动；
- 拔掉 PA11 或停止主机心跳，800 ms 左右自动失能；
- 连续 10 分钟无 CRC 错误、无环形缓冲溢出；
- 重连后不会自动恢复旧运动命令。

---

## 5. 阶段 6：模拟小球状态

切换：

```c
#define APP_MODE APP_MODE_BALL_STATE_SIMULATION
```

默认：

```c
#define APP_PHASE6_MOTOR_OUTPUT_ENABLED 0U
```

因此只计算 PD 预览，电机不会启动、使能或移动。

### 5.1 发送模拟状态

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 simulate `
  --duration 15 `
  --rate 40 `
  --pattern step `
  --amplitude 2.0 `
  --target-cm 0
```

其他模式：

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 simulate --pattern sine --amplitude 3
python tools\pi_ti_stage567_tool.py --port COM5 simulate --pattern ramp --amplitude 5
```

测试无效与超时：

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 simulate `
  --duration 10 --invalid-after 5
```

### 5.2 检查 BALL_PREVIEW

应持续显示：

- position / velocity；
- raw / limited control；
- target offset；
- target physical angle；
- measurement age；
- frame gap；
- valid / armed 标志。

重点确认：

1. 位置误差正负与目标偏移方向一致；
2. 速度项对运动方向产生阻尼；
3. 目标偏移始终限制在 ±0.5°；
4. 目标物理角度始终位于水平点附近；
5. `valid=false` 或停止数据后，preview 回零并解除 armed；
6. 序号跳变、CRC 错误、UART overflow 可通过 LINK_STATS 观察。

### 5.3 可选的小范围电机联动

只有阶段 6 dry-run 全部通过后，才允许修改：

```c
#define APP_PHASE6_MOTOR_OUTPUT_ENABLED 1U
```

安全限制：

- 目标偏移最大 ±0.5°；
- 电机更新不高于 10 Hz；
- 仍需人工置于物理 0°后发送 startup；
- 推荐在同一进程中运行：

```powershell
python tools\pi_ti_stage567_tool.py --port COM5 simulate `
  --duration 15 --rate 40 --pattern sine --amplitude 2 `
  --startup-motor --enable-motor
```

其中 `--startup-motor` 仅在机构已人工放到物理 0° 时使用；
- 无效数据或链路超时自动停止输出并失能。

第一轮应取下钢球，只观察机构方向。

---

## 6. 阶段 7：真实视觉输入，电机禁止动作

切换：

```c
#define APP_MODE APP_MODE_REAL_VISION_OBSERVER
```

该模式无论配置如何都不会开放电机输出。STARTUP、ENABLE 和电机动作命令返回 UNSUPPORTED。

树莓派正式视觉程序以约 40 Hz 发送 BALL_STATE，TI 计算并回传 BALL_PREVIEW。

### 6.1 测试场景

- 球静止在 O 点；
- 手动移到 +5 cm；
- 手动移到 -5 cm；
- 连续左右滚动；
- 短时遮挡；
- 完全取走钢球；
- 恢复钢球；
- 停止树莓派发送 1 秒；
- 恢复发送。

### 6.2 阶段 7 验收

- 接收频率约 40 Hz；
- position 和 velocity 符号与实物一致；
- TI 使用“串口接收后等待时间 + 树莓派上报的 vision_latency_ms”作为总测量 age，通常应小于 120 ms；
- 低置信度、丢球和超时被判定为无效；
- predicted 是否允许由 `APP_VISION_ALLOW_PREDICTED` 明确控制；
- 真实视觉不会触发电机动作；
- LINK_STATS 中 CRC、长度、版本错误长期为 0；
- 新链路不显著降低 Raspberry Pi 视觉 FPS。

---

## 7. 进入真实闭环前的放行条件

只有以下条件全部满足，才进入阶段 8：

- 阶段 5 连续 10 分钟稳定；
- 通信断线自动失能可靠；
- 阶段 6 的位置/速度/控制方向全部确认；
- 模拟无效和超时能立即撤销输出；
- 阶段 7 真实视觉位置方向、速度方向和有效性正确；
- 视觉测量 age、丢包、CRC 和 UART overflow 有完整记录；
- `APP_PHASE6_MOTOR_OUTPUT_ENABLED=0` 的 dry-run 结果已经留档；
- 电机命令窗口仍限制在水平点附近。

真实闭环第一步只做 O 点保持和 ±0.5 cm，小幅开放电机偏移，不直接运行完整 Task 3。


## 8. 启动命令的串口注意事项

`STARTUP_HORIZONTAL` 在 TI 端会阻塞执行约数秒。Python 工具在等待该命令完成时会暂停发送 PI_STATUS 心跳，只读取 ACK/状态，避免 256 字节 UART0 接收环形缓冲被连续心跳填满。启动结束后恢复 5 Hz 心跳。正式树莓派实现也应将启动命令视为长操作：接受 `ACK_ACCEPTED` 后等待最终 ACK，不在此期间高频重发同一命令。
