# Embedded Study

这是一个嵌入式电机和舵机学习总仓库，用来保存近期调试过的 STM32 工程。

这些工程主要围绕总线舵机、PWM 舵机、步进电机和多舵机控制展开。代码中有一部分是在 AI 辅助下完成的，但都经过了本地调试、理解和修改。

## Modules

### bus-servos/SC09ServoTest

SC09 总线舵机测试工程。

已包含的方向：

- 舵机发包和读返回
- ID 扫描和 ID 修改
- EEPROM 解锁/上锁后写 ID、限幅等参数
- 位置模式、PWM 模式
- 位置、电压、温度、负载、电流等反馈读取
- 多舵机同步位置控制
- 按键调位置和串口调试输出

### bus-servos/ZP15DServoTesst

ZP15D 总线舵机测试工程。

主要用于验证另一类总线舵机的协议、接线、反馈读取和串口显示。

### bus-servos/DiffServo

不同总线舵机混合测试工程。

主要用于尝试在同一控制逻辑中切换不同舵机协议或波特率，验证 SC09 和 ZP15D 等舵机的混合控制思路。

### pwm-servos/LD-2015

PWM 舵机测试工程。

主要用于普通 PWM 舵机的定时器 PWM 输出和角度测试。

### stepper-motors/BUJINTest

步进电机测试工程。

主要用于步进电机基础控制、方向控制、速度/步数测试等。

### can-motors/M3508

基于 RoboMaster C 板（STM32F407）和 C620 电调的 M3508 CAN 控制工程。

已包含的方向：

- 一条 CAN 总线注册和管理 1～8 号 M3508
- `0x200`、`0x1FF` 两组电流控制帧发送
- `0x201`～`0x208` 反馈解析与在线检测
- 编码器过零处理、多圈累计和减速箱输出轴角度换算
- 位置环—速度环串级 PID 控制
- 电流限幅、失联清零、积分过零复位和低速摩擦前馈
- USART6 调试接口和中文代码注释

### can-motors/RS05

基于 RoboMaster C 板（STM32F407）和 CAN1 的 RS05 电机驱动工程。

已包含的方向：

- RS05 私有扩展帧协议与 MIT 标准帧协议
- 两套独立电机管理器和统一 CAN FIFO0 接收分发
- 运控、速度、电流、PP、CSP 五种控制模式
- `Enter...Mode()` 一次性配置与 `Set...Target()` 循环目标更新
- 机械零位、主动上报、故障位和在线状态
- U8、U16、U32、float 参数读写
- 每台电机最多 10 项最近参数缓存
- CAN1 PD0/PD1、1 Mbps 配置和 MIT 小角度往复测试

详细用法见 [`can-motors/RS05/README.md`](can-motors/RS05/README.md)。

## Notes

- 原始工程仍保留在 `C:\Clion` 下，这个仓库是整理后的学习归档副本。
- 每个子工程保留了 CubeMX/CLion/CMake 相关源码和配置。
- 构建目录、IDE 缓存和编译产物没有放进仓库。
- STM32CubeMX 工程如果重新生成代码，建议先确认 `USER CODE BEGIN/END` 中的用户代码没有被覆盖。
