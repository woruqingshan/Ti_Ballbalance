# Pi-TI UART 协议 v0.1

所有多字节字段为小端。

```text
A5 5A | version:u8 | type:u8 | payload_len:u16 |
sequence:u32 | timestamp_ms:u32 | payload | crc16:u16
```

CRC16：CCITT-FALSE，多项式 0x1021，初值 0xFFFF；CRC 覆盖 version 到 payload，不覆盖 A5 5A，也不覆盖 CRC 自身。

消息类型：

- `0x01 BALL_STATE` Pi→TI
- `0x02 PI_STATUS` Pi→TI
- `0x10 CONTROL_COMMAND` 测试/维护命令
- `0x81 TASK_EVENT` TI→Pi
- `0x82 CONTROL_TELEMETRY` TI→Pi
- `0x83 HEARTBEAT` 双向

BALL_STATE payload（16 字节）：

```text
frame_id:u32
position_0p1mm:i16
velocity_0p1mm_s:i16
confidence_permille:u16
vision_latency_ms:u16
flags:u8  bit0 found, bit1 predicted, bit2 control_valid
invalid_reason:u8
reserved:u16
```

CONTROL_COMMAND payload：

```text
command:u8  1=ARM, 2=START_TASK3, 3=ABORT, 4=SET_ZERO, 5=MOTOR_ENABLE, 6=MOTOR_DISABLE
argument:u8
reserved:u16
token:u32
```

正式比赛应由物理按键触发 Task 3；CONTROL_COMMAND 仅用于开发和协议联调。
