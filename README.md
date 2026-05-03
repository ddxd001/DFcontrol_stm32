# DFcontrol_stm32 — 固件代码框架说明

基于 **STM32F407VETx**、**STM32CubeMX** 生成外设初始化，在 Keil MDK-ARM 下编译。本仓库在 Cube 生成代码之外，增加一层 **裸机周期调度 + 分层目录**，便于后续接入灰度传感器、DFLink 底盘、步进串口等驱动。

---

## 目录结构

| 路径 | 说明 |
|------|------|
| `Core/` | CubeMX 生成：`main.c`、`gpio.c`、`usart.c`、中断与 HAL 配置。**业务逻辑尽量勿堆在这里**，仅在 `USER CODE BEGIN/END` 内接线。 |
| `MDK-ARM/` | Keil 工程 `DFcontrol_stm32.uvprojx`、启动文件。新增源码需在工程中包含对应分组与 Include Path（已包含 `Firmware` 相关路径）。 |
| `Firmware/Config/` | 工程级可调参数，如 `fw_config.h`（调度器最大任务数等）。 |
| `Firmware/System/` | 系统能力：`scheduler`（周期任务）、`fw_fault`（故障码占位）。 |
| `Firmware/BSP/` | 板级初始化钩子：`Bsp_Init()` 在 `MX_*` 之后调用，适合放“与具体引脚相关、但尚不属于某一设备”的收尾。 |
| `Firmware/App/` | 应用入口 `App_Init()`、`App_StartScheduling()`，以及 `app_tasks.c` 中**周期任务注册与实现**。 |
| `Firmware/Drivers/` | 设备驱动（如 `buzzer_drv`、`gw_gray_sensor`、`dflink_uart5`）。新驱动在此新增 `.c/.h`，并加入 Keil 工程与 Include Path。 |
| `DFcontrol_stm32.ioc` | CubeMX 工程；改时钟/引脚/外设后重新生成代码，注意保留 `main.c` 中 USER 区修改。 |

---

## 程序启动顺序

1. `HAL_Init()` → `SystemClock_Config()`  
2. `MX_GPIO_Init()`、`MX_I2C1_Init()`、`MX_UART4_Init()`、`MX_UART5_Init()`、`MX_USART1_UART_Init()`、`MX_USART2_UART_Init()` 等（以外设实际配置为准）  
3. **`App_Init()`**：`Fault_Init()`、`Bsp_Init()`（内部可再调各驱动的 `*_Init()`）  
4. **`App_StartScheduling()`**：`Scheduler_Init()` + `App_RegisterTasks()`  
5. 主循环：**`Scheduler_RunPending()`**（按节拍调用已到期的周期任务）

与框架相关的接线写在 `Core/Src/main.c` 的 `USER CODE` 区间内，Cube 重新生成时通常可保留。

---

## 调度器约定

- 时间基准：**`HAL_GetTick()`**，分辨率为 **1 ms**。  
- API：`Firmware/System/scheduler.h`  
  - `Scheduler_Init()`  
  - `Scheduler_AddTask(函数指针, period_ms)`，返回 `-1` 表示任务槽满（上限见 `Firmware/Config/fw_config.h` 中 `FW_SCHED_MAX_TASKS`）  
  - `Scheduler_RunPending()` 在主循环中**反复调用**。  
- 同一时刻多个任务到期时，按**添加顺序**依次执行；耗时任务应尽量拆短或降频，避免拖慢后继任务。  
- 需要 **strict 固定相位**的控制环可考虑后续增加硬件定时中断 + 标志位，再在读标志的任务里运算。

默认在 `App_RegisterTasks()` 中注册了 `1 ms / 10 ms / 50 ms / 100 ms` 四档钩子；按需在里面填协议轮询、传感器读取、PID 等。

---

## 蜂鸣器驱动（示例）

- 文件：`Firmware/Drivers/buzzer_drv.c`，**PA11 高电平有效**（Cube 中已配置为输出）。  
- **非阻塞**定长：`BuzzerDrv_Beep(ms)` 依赖 **`BuzzerDrv_Process()` 在约 1 ms 周期被调用**（当前挂在 `App_Task_1ms`）。  
- 若修改 `App_Task_1ms` 的周期，`Beep` 计时会同比偏离，需同步改为基于 `HAL_GetTick` 差值的实现或在 1 ms 任务中继续调用 `Process()`。

## 感为八路灰度（I²C，`gw_gray_sensor`）

- 文件：`Firmware/Drivers/gw_gray_sensor.c`、`gw_gray_sensor.h`，寄存器宏见 **`gw_gray_regs.h`**（与 TI 工程中 `gw_grayscale_sensor.h` / `GraySensor` 约定一致）。
- HAL：**`I2C1`**，`Core/Src/i2c.c` 中为 **PB6=SCL、PB7=SDA**、约 **100 kHz**。若底板走线不同，请在 CubeMX 改脚并合并生成代码与用户区 MSP。
- 默认 **7 位地址 `0x4C`**（与原版左移一位写 `HAL_I2C_Mem_*`）。
- 使用示例：

```c
#include "gw_gray_sensor.h"

static GwGraySensor g_gray;

void InitGray(void)
{
  GwGraySensor_InitDefaults(&g_gray, &hi2c1);
  /* 可选：上电阻塞等待传感器应答，超时则不挂死 */
  (void)GwGraySensor_InitPingWait(&g_gray, 300);
}

bool SampleGrayDigital(void)
{
  return GwGray_ReadDigitalUpdate(&g_gray);
}
/* 读后：g_gray.digital_raw / g_gray.digital_inv（inv 为按位取反，等同原 GraySensor_update） */
```

- 可调：`timeout_ms`、`addr_7bit`；也可用 `GwGray_ReadAnalog8`、`GwGray_ReadLineOffsetU16`（`raw[0] | raw[1]<<8`）等与 TI 原版对应。

## 底盘 DFLink/V2.0（UART5，`dflink_uart5`）

- 文件：`Firmware/Drivers/dflink_uart5.c`、`.h`。硬件：**PC12 TX / PD2 RX**，UART 参数 **460800 8N1**（与模块一致即可）。  
- **`Bsp_Init()`** 中：`DflinkUart5_Init(&huart5)` → **`DflinkUart5_StartRx()`**（字节中断 + 512 字节环缓）。  
- 默认地址：**本机 `0x97`**、**目标（战车）`0x01`**；**`DflinkUart5_SetAddrs(host, chassis)`** 与文档中地址段（战车 1–50、飞控 51–100 等）一致即可改写。  
- 发送：**`DflinkUart5_SendFrame(type_a, type_b, payload_c, c_len, tout_ms)`**。线帧：**`0xDF`** | 目标 | 本机 | **A** | **B** | **LEN** | **C** | **`0xFD`** | **ACC 低 16 位（小端）**；校验为从**帧头到帧尾（含 `0xDF` 与 `0xFD`）**逐字节相加取低 16 位，**先发低字节**（如和为 `0x02B3` 则线上为 **`B3 02`**），详见 `dflink_uart5.c`。**A/B 含义、C 编码**以 [DFLink 协议中心](https://differ-tech.pages.dev/dflink-protocol/) 为准。  
- **匀速位移 sendVelDisplacement（示例 A=`02` B=`64`，LEN=14）**：`**DflinkChassis_SendVelDisplacement**`（`**dflink_chassis_motion.c/.h`**），载荷为 Vx/Vy/Vz（小端，`米×10000`）+ r_max（小端，`m/s×100`）。  
- **自适应位移 sendpos（A=`02` B=`65`，LEN=14）**：`**DflinkChassis_SendAdaptivePosition**`，p_x/p_y/p_z 小端 S32（`米×10000`），max_spd 小端 S16（`m/s×100`）。  
- **`HAL_UART_RxCpltCallback` / `HAL_UART_ErrorCallback`** 在 `debug_uart.c` 中仅转发 UART5 分支；`UART5_IRQHandler` 在 `stm32f4xx_it.c`。  
- 读应答：**`DflinkUart5_RxAvail()`** / **`DflinkUart5_ReadBytes()`**（按 `0xDF`…`0xFD` 拆帧可在 App 层做）。

## 当前功能概览（与代码对齐）

- **USART1 指令触发**：`AA BB XX` 协议触发 Q1~Q4、TEST1~TEST4、步进电机控制等。
- **底盘运动控制**：通过 `UART5 + DFLink` 下发自适应位移、匀速位移、转向、锁头等控制指令。
- **路口检测与测距**：
  - 灰度检测频率 100Hz（`App_Task_10ms`）
  - 含连续帧确认滤波（时间层）
  - 测距段结束后仅通过 **`UART4` 蓝牙** 发送 `l1` / `l2`（`USART1` 无 `CROSS`/`DIST` 文本）
- **蓝牙距离上报（UART4）**：
  - 测距段结束后向蓝牙发送两行：`l1 = ...cm`、`l2 = ...cm`
  - 缺失项默认发送 `2.5cm`
- **步进电机（USART2）**：支持使能/失能与 90° 相对转动。

## 串口与通信说明

- `USART1`：任务指令（`DebugUart`，`AA BB XX`）。
- `UART4`：蓝牙模块数据发送（测距结果 `l1/l2`）。
- `UART5`：底盘 DFLink 通信。
- `USART2`：步进电机驱动串口。

---

## 任务与指令文档

- 详细指令与动作流程请查看：`Firmware/App/UART_COMMAND_TASKS.md`
- 该文档与 `Firmware/App/app_tasks.c` 同步维护，包含：
  - 指令总表（`AA BB 01/02/.../D6`）
  - Q/TEST 动作步骤与关键参数
  - 测距公式、滤波参数、蓝牙发送格式

---

## 如何扩展

### 新增周期任务

在 `app_tasks.c` 中增加 `static void App_Task_Xxx(void)`，在 `App_RegisterTasks()` 里 `Scheduler_AddTask(App_Task_Xxx, 周期_ms)`。

### 新增设备驱动

1. 在 `Firmware/Drivers/` 增加 `xxx_drv.c` / `xxx_drv.h`。  
2. 在 **Keil** 的 `Firmware` 组中加入 `.c` 文件。  
3. 确认 **C/C++ Include Path** 含 `../Firmware/Drivers`（工程已配）。  
4. 在 **`Bsp_Init()`** 或 **`App_Init()`** 中调用驱动的 `Xxx_Init()`（依依赖顺序而定）。  
5. 在合适周期的任务中调用 `Xxx_Process()` / 读采样 API。

### CubeMX 侧建议（按需）

- 八路灰度：**I2C** + 上拉与速率与线长相匹配。  
- 底盘 DFLink / 步进：通常各占用一条 **UART**（当前已使用 UART5 承载底盘通信）。  
- 时钟：若板载 **HSE**，建议在 Cube 中统一为 PLL 源，避免长期 HSI 带来串口波特率误差。

---

## 编译与烧录

- 使用 **Keil uVision5** 打开 `MDK-ARM/DFcontrol_stm32.uvprojx`，目标芯片 **STM32F407VETx**。  
- HAL 路径当前指向本机 STM32Cube 仓库；**换电脑后**需在工程选项中修正 `Drivers` 的绝对路径，或将 HAL 拷入工程并改相对路径。

---

## 版本与维护

- 框架由项目维护者演进；与 ST 官方生成代码的边界以 **`USER CODE BEGIN/END`** 为准。  
- 协议与设备细节（如 DFLink）见对应文档，本 README 仅描述**代码组织与运行模型**。
