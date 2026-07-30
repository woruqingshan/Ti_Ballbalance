# Ti_Ballbalance 阶段 5/6/7 补丁

基线：`woruqingshan/Ti_Ballbalance` master，提交 `dc44a25c76d74dcab664354b82a70f54cd3b392a`。

实现：

- 阶段 5：Pi/Windows 到 TI 的二进制手动控制、ACK、状态、心跳、token 去重和断线失能；
- 阶段 6：40 Hz 模拟 BALL_STATE、PD 预览、无效/超时保护，默认不驱动电机；
- 阶段 7：真实视觉观察模式，回传控制预览，硬性禁止电机输出；
- Windows/Raspberry Pi 通用 Python 测试工具；
- C/Python 协议测试；
- 不包含任何 Keil 工程、J-Link 或 Flash Algorithm 文件。

默认 APP_MODE 为 `APP_MODE_PI_TI_MANUAL_LINK`。覆盖后先执行 `tools/verify_phase567_patch.ps1` 和 Python self-test，再用原 Keil 工程 Clean、Rebuild、Download。


重要：阶段 5 的真实动作序列优先使用 `manual-demo` 保持同一连接；阶段 6 可选真实电机联动时使用 `simulate --startup-motor --enable-motor`。控制预览的 age 包含视觉延迟与 TI 端接收后等待时间。
