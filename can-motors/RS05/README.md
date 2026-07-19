# RS05 CAN 电机驱动

这是一个基于 STM32F407 HAL 库的 RS05 电机学习与测试工程，支持 RS05 私有扩展帧协议和 MIT 标准帧协议。代码使用独立管理器保存 CAN 句柄、主机 ID 和电机对象，并在同一个 CAN FIFO0 回调中按照帧类型分发数据。

当前 `main.c` 是 MIT 运控模式的小角度往复测试程序；私有协议的五种控制模式已经封装为“进入模式”和“更新目标”两层接口。

## 硬件与总线配置

- MCU：STM32F407
- CAN 外设：CAN1
- CAN1 RX：PD0
- CAN1 TX：PD1
- GPIO 复用：AF9
- CAN 模式：Normal
- CAN 波特率：1 Mbps
- CAN 时序：Prescaler = 3，BS1 = 9 TQ，BS2 = 4 TQ，SJW = 1 TQ
- 默认主机 ID：`0xFD`
- 默认电机 ID：`0x01`

单片机不能直接连接 CAN_H/CAN_L，需要外接 CAN 收发器。电机、收发器和单片机必须共地，总线两端应按实际拓扑配置 120 Ω 终端电阻。

## 目录结构

```text
Core/Inc
├── CAN_bus.h          CAN 过滤器、启动和帧发送接口
├── RS05_Command.h     通信类型、参数索引、范围和模式宏
├── RS05_app.h         私有协议电机、管理器和应用接口
├── RS05_codec.h       大小端与浮点/整数转换
├── RS05_mit.h         MIT 协议电机、管理器和控制接口
├── RS05_parameter.h   参数类型、联合体和缓存结构
└── RS05_receive.h     CAN 接收分发接口

Core/Src
├── CAN_bus.c
├── RS05_app.c
├── RS05_codec.c
├── RS05_mit.c
├── RS05_parameter.c
├── RS05_receive.c
└── main.c
```

## 管理器初始化

私有协议和 MIT 协议分别使用自己的管理器。它们可以共用同一个 CAN 外设，但不会共用电机结构体。

```c
static RS05_ManagerTypedef RS05_Manager;
static RS05_MIT_ManagerTypedef RS05_MIT_Manager;

static RS05_MotorTypedef RS05_Motor1 = {
    .motor_id = RS05_MOTOR_ID,
};

static RS05_MIT_MotorTypedef RS05_MIT_Motor1 = {
    .motor_id = RS05_MOTOR_ID,
};

RS05_Manager_Init(&RS05_Manager, &hcan1, RS05_MASTER_ID);
RS05_MIT_Manager_Init(&RS05_MIT_Manager, &hcan1, RS05_MASTER_ID);

RS05_Manager_RegisterMotor(&RS05_Manager, &RS05_Motor1);
RS05_MIT_Manager_RegisterMotor(&RS05_MIT_Manager, &RS05_MIT_Motor1);

CAN_Init(&hcan1);
```

## 私有协议五种控制模式

接口分为两类：

- `RS05_Enter...Mode()`：停止电机、设置 `run_mode`、写入固定配置和安全初值，然后使能。切换模式时调用一次。
- `RS05_Set...Target()`：只更新控制目标。控制循环中可以反复调用。

所有接口都会检查管理器、电机注册关系和参数范围，并通过 `HAL_StatusTypeDef` 返回发送结果。

### 1. 运控模式

```c
RS05_EnterMotionMode(&RS05_Manager, &RS05_Motor1);

while (1)
{
    RS05_SetMotionTarget(&RS05_Manager,
                         &RS05_Motor1,
                         position_rad,
                         velocity_rad_s,
                         kp,
                         kd,
                         torque_ff_nm);
}
```

运控目标通过通信类型 `0x01` 的一帧数据发送，不使用参数写入帧。

### 2. 速度模式

```c
RS05_EnterSpeedMode(&RS05_Manager,
                    &RS05_Motor1,
                    acceleration_rad_s2,
                    current_limit_a);

while (1)
{
    RS05_SetSpeedTarget(&RS05_Manager,
                        &RS05_Motor1,
                        target_speed_rad_s);
}
```

进入时配置电流限制和加速度，并先把速度目标写为 0；循环中只更新 `spd_ref`。

### 3. 电流模式

```c
RS05_EnterCurrentMode(&RS05_Manager, &RS05_Motor1);

while (1)
{
    RS05_SetCurrentTarget(&RS05_Manager,
                          &RS05_Motor1,
                          target_current_a);
}
```

进入时先把电流目标写为 0；循环中只更新 `iq_ref`。

### 4. PP 位置模式

```c
RS05_EnterPPMode(&RS05_Manager,
                 &RS05_Motor1,
                 max_velocity_rad_s,
                 acceleration_rad_s2,
                 deceleration_rad_s2,
                 current_limit_a);

RS05_SetPPTarget(&RS05_Manager,
                 &RS05_Motor1,
                 target_position_rad);
```

进入时配置最大速度、加速度、减速度和电流限制，并把当前反馈位置作为初始目标；后续只更新 `loc_ref`。

### 5. CSP 位置模式

```c
RS05_EnterCSPMode(&RS05_Manager,
                  &RS05_Motor1,
                  speed_limit_rad_s,
                  current_limit_a);

RS05_SetCSPTarget(&RS05_Manager,
                  &RS05_Motor1,
                  target_position_rad);
```

进入时配置速度限制和电流限制，并把当前反馈位置作为初始目标；后续只更新 `loc_ref`。

## 设置机械零位

私有协议使用通信类型 `0x06`，`Byte[0] = 1`，将当前位置设为机械零位：

```c
RS05_SetMechanicalZero(&RS05_Manager, &RS05_Motor1);
HAL_Delay(20U);

/* 标零后电机保持停止，需要重新进入目标模式。 */
RS05_EnterSpeedMode(&RS05_Manager,
                    &RS05_Motor1,
                    acceleration_rad_s2,
                    current_limit_a);
```

函数内部执行 `Stop → 切换到运控模式但不使能 → 设置机械零位`，用于避开 PP 模式下可能屏蔽标零的问题。该机械零位掉电后丢失。

MIT 协议使用独立接口：

```c
RS05_MIT_SetZero(&RS05_MIT_Manager, RS05_MOTOR_ID);
```

## 反馈和参数缓存

`RS05_ProcessRxFifo0()` 从 CAN FIFO0 取出数据并分发：

- 扩展帧 `0x02`：普通电机反馈
- 扩展帧 `0x18`：主动上报反馈
- 扩展帧 `0x11`：单参数读取回复
- 标准帧：MIT 协议反馈或参数回复

反馈数据会更新位置、速度、力矩、温度、状态、故障位和最后反馈时间。

每个私有协议电机保存最多 10 个最近参数。相同 index 会覆盖原值；新 index 使用空槽；缓存已满时替换最久未更新的参数；未知参数和错误回复不会污染缓存。

```c
RS05_Command_ParaRead(&RS05_Manager,
                      RS05_MOTOR_ID,
                      RS05_PARAMETER_SPEED_REF);

RS05_ParameterCache *parameter = RS05_FindParameterCache(
    &RS05_Motor1,
    RS05_PARAMETER_SPEED_REF);

if ((parameter != NULL) &&
    (parameter->type == RS05_PARAMETER_FLOAT))
{
    float speed_ref = parameter->value.f32;
}
```

## MIT 协议

MIT 协议使用 11 位标准帧，并由 `RS05_MIT_ManagerTypedef` 独立管理。主要接口包括：

- 使能、停止和置零
- MIT 运控帧
- 位置帧和速度帧
- 主动上报和故障读取
- U8、U16、U32、float 参数写入
- 参数回复与反馈解析

```c
RS05_MIT_SetRunMode(&RS05_MIT_Manager,
                    RS05_MOTOR_ID,
                    RS05_MIT_RUN_MODE_MIT);
RS05_MIT_Enable(&RS05_MIT_Manager, RS05_MOTOR_ID);

RS05_MIT_Control(&RS05_MIT_Manager,
                 RS05_MOTOR_ID,
                 target_position_rad,
                 0.0f,
                 kp,
                 kd,
                 0.0f);
```

如果电机当前不是 MIT 协议，需要先用私有协议发送协议切换指令，并按照固件要求重新上电。

## 构建

需要安装 CMake、Ninja 和 GNU Arm Embedded Toolchain，并确保 `arm-none-eabi-gcc` 可用。

```powershell
cd can-motors/RS05
cmake --preset Debug
cmake --build --preset Debug
```

本次代码在 STM32 Debug 配置下完成干净编译和链接，生成 `RS05.elf`。构建目录、IDE 缓存和编译产物不提交到仓库。

## 当前验证状态

- CAN1 使用 PD0/PD1 和 1 Mbps 时序，已解决早期引脚映射错误。
- MIT 运控帧和反馈接收已用于实机转动测试。
- 双协议管理器、参数缓存和五模式 Enter/Target 重构已通过本地干净编译。
- 参数缓存已使用构造回复帧验证首次插入、重复更新、错误帧拒绝和满缓存替换。
- 最新五模式封装和机械零位接口仍建议在空载、低限流条件下逐项做实机验证。

## 注意事项

- 首次调试请降低电流限制、速度和位置变化量，并确保电机空载。
- PP/CSP 进入时会使用当前反馈位置作为安全初值，应先确认反馈在线。
- 检查每个 HAL 返回值；发送失败或反馈超时时应停止电机。
- 机械零位是易失设置，掉电后需要重新设置。
- CubeMX 重新生成代码前，应确认 CAN1 仍映射到 PD0/PD1，并保护 `USER CODE BEGIN/END` 中的用户代码。
