# ELRS 遥控接收与底盘命令解算

基于 STM32F407 的 ELRS/CRSF 接收工程。此工程只完成遥控接收和底盘命令解算，不包含电机驱动或四轮电机输出。

## 模块划分

```text
Core/UserModules/
├─ BSP/elrs_crsf.c        CRSF 帧解析、CRC 校验、16 路通道保存
└─ APP/
   ├─ ELRS_DMA.c          USART1 循环 DMA + IDLE 接收
   └─ Chassis_Remote.c    通道映射为 vx、vy、wz、档位和使能状态
```

## 接收流程

```text
USART1 RX DMA（循环模式）
  -> UART IDLE 中断仅保存 DMA 当前位置并置待处理标志
  -> 主循环 ELRS_DMA_Process() 解析本次新增字节
  -> 主循环 Chassis_Remote_Update() 读取最新通道并更新底盘命令
```

`ELRS_DMA_Process()` 在没有 IDLE 标志时立即返回；`Chassis_Remote_Update()` 每轮均执行，因此接收解帧与底盘命令更新不互相绑定。

## 硬件配置

- MCU：STM32F407IGHx
- ELRS：USART1，420000 bps，PA9 TX、PB7 RX
- USART1 RX DMA：DMA2 Stream2，循环模式

## 验证状态

已通过 CLion/CMake 干净编译。尚未进行 ELRS 接收机实机、遥控通道和底盘实际运动验证。
