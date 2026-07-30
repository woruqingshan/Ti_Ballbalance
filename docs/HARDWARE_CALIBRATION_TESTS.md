# v0.1.1 电机位置范围与正负号测试

## 接线

- UART0 调试：PA10(TX) → CH340 RX，PA11(RX) ← CH340 TX，GND 共地。
- UART1 电机：PB4(TX) → Emm RX，PB5(RX) ← Emm TX，GND 共地。
- 两路串口均为 115200、8N1。
- 测试时先移除钢球，并让机械机构处于行程中部。

## 测试 A：人工拖动，读取最小/最大电机位置

在 `src/app/app_config.h` 中改为：

```c
#define APP_MODE APP_MODE_EMM_POSITION_CALIBRATION
```

重新编译、烧录。串口助手使用字符显示。程序会先发送电机失能命令，然后每 100 ms 发送 `01 36 6B` 查询 Emm V5.0 实时位置。

启动后第一条有效位置自动设为相对零点。缓慢拖动水管至两侧安全位置，观察：

```text
POS raw=... rel=... pulse=... deg=... min=... max=...
```

字段：

- `raw`：Emm 返回的带符号实时位置单位。当前工程按 `APP_EMM_POSITION_UNITS_PER_REVOLUTION=65536` 换算，这是与现有 FD 脉冲位置命令匹配的协议变体。
- `rel`：相对本次零点的编码器单位。
- `pulse`：使用上面的每圈位置单位配置，并按 16 细分、3200 pulse/rev 换算的等效命令脉冲。
- `deg`：电机轴相对角度，不是水管真实角度。
- `min/max`：本次运行记录到的相对最小和最大编码器位置。

串口字符命令：

```text
z  当前实际位置设为零，并清空 MIN/MAX
r  把 MIN/MAX 重置为当前位置
p  立即打印
 d 再次发送失能
h  帮助
```

若持续出现 timeout，请检查 PB5 是否接到 Emm TX、双方是否共地，以及驱动器是否使用地址 0x01、115200 和 0x6B 校验模式。

## 测试 B：微动判断正负号

在 `src/app/app_config.h` 中改为：

```c
#define APP_MODE APP_MODE_MOTOR_SIGN_TEST
```

重新编译、烧录。该模式安全启动时电机保持失能，不会自动移动。

依次在串口助手发送：

```text
e   使能
2   选择 10 pulse
+   正方向移动 10 pulse
0   返回软件命令零点
-   负方向移动 10 pulse
0   返回软件命令零点
d   失能
```

记录：

```text
+10 pulse 时哪一端升高，小球理论上会向哪一侧滚动
-10 pulse 时哪一端升高，小球理论上会向哪一侧滚动
```

其他命令：

```text
1/2/3  选择 5/10/20 pulse
q      查询实际编码器位置
z      下一次有效位置作为显示零点
p      打印状态
x      立即失能
h      帮助
```

测试模式限制软件命令偏移在 ±30 pulse；超过后拒绝继续向外移动。这个限制仅用于微动测试，并不替代正式机械软限位。

## 结果记录模板

```text
位置标定：
zero_raw =
physical_safe_min_rel =
physical_safe_max_rel =
physical_safe_min_pulse_equivalent =
physical_safe_max_pulse_equivalent =

方向测试：
positive pulse -> __________ end rises
negative pulse -> __________ end rises
positive pulse -> ball tends toward visual __________ direction
recommended APP_MOTOR_DIRECTION_SIGN = +1 / -1
```

## 重要解释

`0x36` 返回的是电机内部实时位置。它能用于检测电机轴位置和建立软件软限位，但不能被当作水管实际角度传感器。曲柄、连杆、安装间隙和非线性仍需通过外部量角工具离线标定。


### 协议版本核对

Emm V5.0 不同资料版本对 `0x36` 数值单位存在差异。当前项目原有 `FD` 指令使用“脉冲数 + 相对模式”格式，因此默认按 65536 position units/rev 处理。MIN/MAX 的 `raw` 和 `rel` 不依赖角度换算，始终可用于记录边界。若实测正向 10 pulse 后 `raw` 只变化约 11，而不是约 205，请把 `APP_EMM_POSITION_UNITS_PER_REVOLUTION` 改为 `3600LL`，再重新编译。
