# STM32G431CBT6 BLDC 电机 FOC 控制项目开发文档

---

## 1. 项目概述

### 1.1 项目简介

本项目基于 **STM32G431CBT6** 微控制器（ARM Cortex-M4, 170MHz，带硬件 FPU），实现了一款**基于霍尔传感器的无刷直流电机 (BLDC) 磁场定向控制 (FOC)** 系统。采用三电阻电流采样方式，使用 SVPWM（空间矢量脉宽调制）技术驱动三相全桥逆变器，搭配预驱芯片 FD6288T 实现对电机的高性能控制。

### 1.2 主要特性

| 特性 | 参数 |
|------|------|
| 主控芯片 | STM32G431CBT6 (Cortex-M4F @ 170MHz) |
| 控制方式 | 霍尔传感器 FOC + SVPWM |
| PWM 频率 | 20kHz 中心对齐模式（三相互补输出） |
| 电流采样 | 三电阻采样，ADC1 注入组同步触发 |
| 电机反馈 | 三路霍尔传感器 (TIM3 硬件霍尔接口) |
| 控制模式 | 速度-电流双闭环 PID（串联/并联可选） |
| 母线电压采样 | ADC2 注入组 |
| 调速方式 | 板载电位器 (ADC2 规则组) 或规定转速 |
| 用户接口 | 启动/停止按键 + 正反转切换按键 |
| 调试接口 | USART2 (VOFA+ 上位机波形显示) |
| 极对数 | 2 对极 |
| 转速范围 | 0 ~ 2300 RPM |
| 编译环境 | CMake + GCC ARM Toolchain |
| 调试工具 | OpenOCD + GDB (ST-Link) |

### 1.3 FOC 控制架构概览

```
┌─────────────────────────────────────────────────────────┐
│                    FOC 控制环 (20kHz)                    │
│                                                         │
│   ADC采样           Clark           Park               │
│   Ia,Ib,Ic  ────►  Iα,Iβ  ────►  Id,Iq               │
│                                      │                  │
│                              ┌───────┴───────┐          │
│                              │  电流环 PID   │          │
│                              │  Id* = 0      │          │
│                              │  Iq*←速度环输出│          │
│                              └───────┬───────┘          │
│                                      │                  │
│                              ┌───────┴───────┐          │
│                              │  inverse-Park │          │
│                              │  Uα, Uβ       │          │
│                              └───────┬───────┘          │
│                                      │                  │
│                              ┌───────┴───────┐          │
│                              │    SVPWM      │          │
│                              │  三相占空比    │          │
│                              └───────┬───────┘          │
│                                      │                  │
│                              ┌───────┴───────┐          │
│                              │   TIM1 PWM    │          │
│                              │   功率驱动     │          │
│                              └───────────────┘          │
│                                                         │
│   霍尔传感器 → 转子位置角计算 → Park/逆Park 角度         │
│   霍尔传感器 → 60°时间计算  → 速度环 PID (5Hz)          │
└─────────────────────────────────────────────────────────┘
```

---

## 2. STM32CubeMX 硬件配置

### 2.1 工程基础

- **Project Name**: `STM32G431CBT6_BLDC`
- **芯片型号**: STM32G431CBT6 (LQFP-48)
- **Toolchain**: CMake (GCC)

### 2.2 时钟配置 (Clock Configuration)

| 时钟源 | 配置 | 频率 |
|--------|------|------|
| HSE | 外部晶振 | 8MHz |
| PLL Source | HSE | - |
| PLLM | /2 | - |
| PLLN | ×85 | - |
| PLLP | /2 | - |
| PLLQ | /2 | - |
| PLLR | /2 | - |
| SYSCLK | PLLCLK | **170 MHz** |
| HCLK | /1 | **170 MHz** |
| APB1 | /1 | **170 MHz** |
| APB2 | /1 | **170 MHz** |
| 电压调节器 | Scale 1 (Boost) | 支持 170MHz |

### 2.3 TIM1 — 三相 PWM 高级定时器

用于产生 6 路互补 PWM 波驱动功率 MOSFET。

| 参数 | 配置 | 说明 |
|------|------|------|
| Prescaler | 0 | 不分频，170MHz |
| Counter Mode | **Center Aligned Mode 2** | 中心对齐模式2（向下计数时发生更新事件） |
| ARR (Period) | **4250** | 170MHz / (2 × 20kHz) = 4250 |
| PWM 频率 | 20kHz | - |
| PWM Mode | **Mode 2** | CCR > CNT 时输出高电平 |
| CH1/CH1N | PA8 / PB13 | U 相上下桥臂 |
| CH2/CH2N | PA9 / PB14 | V 相上下桥臂 |
| CH3/CH3N | PA10 / PB15 | W 相上下桥臂 |
| Dead Time | **34** (~200ns) | 硬件死区，防止上下桥直通 |
| OCPolarity | HIGH | - |
| OCNPolarity | HIGH | - |
| Repetition Counter | 1 | 每两次更新事件触发一次 TRGO（目的：只在向下溢出时触发 ADC） |
| TRGO | Update Event | 触发 ADC1/ADC2 注入组采样 |

> **注意**：PWM Mode 2 下，CCR = 0 对应 100% 占空比（上管常开），CCR = ARR (4250) 对应 0% 占空比（上管常关，下管常开）。

### 2.4 TIM3 — 霍尔传感器接口

| 参数 | 配置 | 说明 |
|------|------|------|
| Mode | Hall Sensor Interface | 硬件霍尔接口（三通道异或） |
| Prescaler | **169** | 170MHz / 170 = 1MHz |
| Counter Mode | Up | 向上计数 |
| ARR | 65535 | 最大计数范围 |
| CH1/CH2/CH3 | PA6 / PA7 / PB0 | U/V/W 霍尔信号 |
| IC Filter | 15 | 最大输入滤波，抑制干扰 |
| IC Polarity | RISING | 上升沿捕获 |
| Interrupt | TIM3_IRQn (优先级 0,0) | 输入捕获中断 |

**霍尔接口原理**：TIM3 的硬件霍尔模式将三路霍尔信号进行异或运算（XOR），在任一霍尔信号发生边沿跳变时自动捕获当前计数器值并复位计数器，从而直接获取两次跳变间的时间差（60°电角度时间 Δt）。

### 2.5 ADC1 / ADC2 — 模拟采样

#### ADC1（相电流采样）

| 参数 | 配置 |
|------|------|
| Clock Prescaler | /4 (42.5MHz) |
| Resolution | 12-bit |
| Scan Mode | Enable |
| Data Alignment | Right aligned |

**注入组（同步触发，最高频率 20kHz 采样）**：

| 注入 Rank | Channel | 引脚 | 采样信号 | 采样时间 |
|-----------|---------|------|----------|----------|
| 1 | CH1 | PA0 | U 相电流 | 12.5 Cycles |
| 2 | CH2 | PA1 | V 相电流 | 12.5 Cycles |
| 3 | CH3 | PA2 | W 相电流 | 12.5 Cycles |
| 触发源 | **TIM1 TRGO** | - | 上升沿触发（PWM 下溢） |

#### ADC2（母线电压 + 电位器）

| 参数 | 配置 |
|------|------|
| Clock Prescaler | /4 (42.5MHz) |
| Resolution | 12-bit |
| Data Alignment | Right aligned |

**规则组**：

| Rank | Channel | 引脚 | 采样信号 | 采样时间 |
|------|---------|------|----------|----------|
| 1 | CH13 | PA5 | 调速电位器电压 | 640.5 Cycles |
| 触发源 | **TIM1 TRGO** | - | 上升沿触发 |

**注入组**：

| 注入 Rank | Channel | 引脚 | 采样信号 | 采样时间 |
|-----------|---------|------|----------|----------|
| 1 | CH17 | PA4 | 母线电压 | 24.5 Cycles |
| 触发源 | **TIM1 TRGO** | - | 上升沿触发 |

#### ADC 中断

- **ADC1_2_IRQn** (优先级 0,0)：共享中断，处理 ADC1 和 ADC2 的注入组/规则组转换完成回调。

### 2.6 USART2 — 调试串口

| 参数 | 配置 |
|------|------|
| 波特率 | **115200** |
| 数据位 | 8-bit |
| 停止位 | 1 |
| 校验位 | None |
| 流控 | None |
| TX/RX | PB3 / PB4 |
| FIFO | Disable |

printf 重定向到 USART2，用于打印系统运行信息以及 VOFA+ 实时波形数据。

### 2.7 GPIO 引脚分配总表

| 引脚 | 功能 | 说明 |
|------|------|------|
| PA0 | ADC1_IN1 | U 相电流采样 |
| PA1 | ADC1_IN2 | V 相电流采样 |
| PA2 | ADC1_IN3 | W 相电流采样 |
| PA4 | ADC2_IN17 | 母线电压采样 |
| PA5 | ADC2_IN13 | 调速电位器采样 |
| PA6 | TIM3_CH1 | 霍尔 U 信号 |
| PA7 | TIM3_CH2 | 霍尔 V 信号 |
| PA8 | TIM1_CH1 | PWM U 相上桥 |
| PA9 | TIM1_CH2 | PWM V 相上桥 |
| PA10 | TIM1_CH3 | PWM W 相上桥 |
| PB0 | TIM3_CH3 | 霍尔 W 信号 |
| PB3 | USART2_TX | 调试串口发送 |
| PB4 | USART2_RX | 调试串口接收 |
| PB11 | GPIO Input | CW/CCW 正反转按键 |
| PB12 | GPIO Input | START/STOP 启停按键 |
| PB13 | TIM1_CH1N | PWM U 相下桥 |
| PB14 | TIM1_CH2N | PWM V 相下桥 |
| PB15 | TIM1_CH3N | PWM W 相下桥 |

---

## 3. 项目架构

### 3.1 目录结构

```
STM32G431CBT6_BLDC/
├── Core/                          # CubeMX 生成的 HAL 层代码
│   ├── Inc/
│   │   ├── main.h                 # 主头文件、引脚宏定义
│   │   ├── stm32g4xx_it.h        # 中断服务函数声明
│   │   ├── stm32g4xx_hal_conf.h  # HAL 库配置
│   │   ├── gpio.h / adc.h / tim.h / usart.h   # 外设头文件
│   └── Src/
│       ├── main.c                 # 主函数、系统初始化、主循环、VOFA+调试函数
│       ├── stm32g4xx_it.c         # 中断服务函数（ADC、TIM3、HardFault等）
│       ├── stm32g4xx_hal_msp.c   # 外设 MSP 初始化
│       ├── gpio.c / adc.c / tim.c / usart.c    # 外设初始化配置
│       ├── system_stm32g4xx.c     # 系统时钟初始化
│       ├── syscalls.c / sysmem.c  # 底层系统调用
│
├── Drivers/                       # ST 官方驱动
│   ├── CMSIS/                     # Cortex-M4 内核文件
│   └── STM32G4xx_HAL_Driver/     # HAL 库源码
│
├── bsp/                           # 板级支持包 (Board Support Package)
│   ├── Inc/
│   │   ├── mcu_pwm.h             # PWM 驱动封装接口
│   │   ├── mcu_adc.h / mcu_adc_cb.h   # ADC 驱动及数据回调
│   │   ├── mcu_hall.h / mcu_hall_cb.h  # 霍尔传感器接口
│   │   ├── mcu_key.h             # 按键扫描接口
│   │   └── mcu_uart.h            # 串口发送接口
│   └── Src/
│       ├── mcu_pwm.c             # PWM 初始化和占空比设置
│       ├── mcu_adc.c / mcu_adc_cb.c   # ADC 初始化、采样、滤波、中断回调
│       ├── mcu_hall.c / mcu_hall_cb.c  # 霍尔传感器初始化和中断回调
│       ├── mcu_key.c             # 按键消抖与状态机扫描
│       └── mcu_uart.c            # 串口发送与 printf 重定向
│
├── motor/                         # 电机控制算法层
│   ├── Inc/
│   │   ├── typedef_header.h       # 公共类型定义（联合体、LPF宏、CLAMP宏）
│   │   ├── m_parameter.h          # 电机参数、宏常量（极对数、转速范围等）
│   │   ├── m_foc.h               # FOC 主控结构体与函数声明
│   │   ├── m_pid.h               # PID 控制器结构体与算法接口
│   │   ├── m_ctrl.h              # 电机状态机和速度控制结构体
│   │   ├── m_coordinate.h        # 坐标变换（Clark/Park/逆Park）接口
│   │   ├── m_svpwm.h             # SVPWM 计算接口
│   │   ├── m_rotor_angle.h        # 转子位置角度与霍尔处理接口
│   │   └── m_tick.h              # 系统滴答计数器接口
│   └── Src/
│       ├── m_foc.c               # FOC 主算法（Us模长、电流环、速度环、状态机）
│       ├── m_pid.c               # PID 三种实现（并联增量式/位置式/串联型）
│       ├── m_ctrl.c              # 电机启停控制、相电流偏置校准
│       ├── m_coordinate.c        # Clark/Park/逆Park/Us模长与超前角计算
│       ├── m_svpwm.c             # SVPWM 扇区/矢量/占空比计算与硬件输出
│       ├── m_rotor_angle.c        # 霍尔状态读取、转子位置角插值、速度计算
│       └── m_tick.c              # 各类定时器递减管理
│
├── cmake/                         # CMake 构建配置
│   ├── stm32cubemx/CMakeLists.txt  # CubeMX 生成的源码构建
│   ├── gcc-arm-none-eabi.cmake     # ARM GCC 工具链配置
│   └── starm-clang.cmake           # Star-M Clang 工具链配置
│
├── CMakeLists.txt                  # 顶层 CMake 构建文件
├── CMakePresets.json               # CMake 预设
├── STM32G431XX_FLASH.ld            # 链接脚本
├── openocd.cfg                     # OpenOCD 调试配置
├── startup_stm32g431xx.s           # 启动汇编文件
├── STM32G431CBT6_BLDC.ioc          # CubeMX 项目文件
└── .vscode/                        # VS Code 配置
    ├── launch.json                  # 调试启动配置
    ├── settings.json
    └── c_cpp_properties.json
```

### 3.2 四层软件架构

```
┌─────────────────────────────────────────┐
│         Application Layer               │
│   main.c  (系统初始化、主循环、调试)     │
├─────────────────────────────────────────┤
│       Motor Algorithm Layer             │
│  motor/ (FOC, PID, SVPWM, 坐标变换)    │
├─────────────────────────────────────────┤
│    Board Support Package (BSP)          │
│  bsp/ (PWM, ADC, 霍尔, 按键, 串口)     │
├─────────────────────────────────────────┤
│      HAL / CMSIS Driver Layer          │
│  Drivers/ + Core/ (HAL库, CMSIS内核)   │
├─────────────────────────────────────────┤
│          Hardware Layer                 │
│  STM32G431CBT6 + FD6288T + MOSFETs     │
└─────────────────────────────────────────┘
```

| 层级 | 职责 | 示例文件 |
|------|------|----------|
| **Application** | 系统初始化流程、主循环调度、VOFA+ 调试数据的组包与发送 | [main.c](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/Core/Src/main.c) |
| **Motor Algorithm** | FOC 核心数学计算、PID 控制、SVPWM 调制、坐标变换、转子位置估计 | [m_foc.c](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Src/m_foc.c), [m_pid.c](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Src/m_pid.c) |
| **BSP** | 封装 HAL 外设操作，提供统一硬件接口，处理中断回调 | [mcu_pwm.c](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/bsp/Src/mcu_pwm.c), [mcu_adc_cb.c](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/bsp/Src/mcu_adc_cb.c) |
| **HAL/CMSIS** | STM32 官方硬件抽象层，提供外设寄存器级操作 | HAL 库、CMSIS 内核文件 |

### 3.3 关键数据结构

#### 3.3.1 FOC 核心数据 (m_foc_unit_t)

全局变量 `m_foc_unit` 统筹所有 FOC 计算所需的中间变量，定义在 [m_foc.h](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Inc/m_foc.h#L37-L47)：

- **rotor_engle**: 转子位置电角度 (Q16, 0~65535 对应 0°~360°)
- **q_engle**: 合成电压矢量 Us 与 0° 位置的夹角
- **advance_angle**: 超前角 (电流环 Ud/Uq 自动补偿)
- **q16_us**: Us 矢量模长 (Q16)
- **coordinate**: 内嵌坐标变换数据结构 `m_coordinate_t`

#### 3.3.2 PID 控制器数据 (m_pid_unit_t)

定义在 [m_pid.h](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Inc/m_pid.h#L22-L43)：

全局实例：
- **m_id_pid_unit** (Kp=12000, Ki=800): 电流环 Id PID
- **m_iq_pid_unit** (Kp=12000, Ki=800): 电流环 Iq PID  
- **m_spd_pid_unit** (Kp=20000, Ki=200): 速度环 PID

#### 3.3.3 电机控制状态 (m_motor_ctrl_t)

定义在 [m_ctrl.h](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Inc/m_ctrl.h#L31-L39)：

状态机枚举：
- EXECUTE_MOTOR_STOP → EXECUTE_MOTOR_START → EXECUTE_MOTOR_BOOST_CHARGING → EXECUTE_MOTOR_EXECUTE
- EXECUTE_MOTOR_ALIGNMENT_TEST（霍尔对齐测试专用模式）

---

## 4. 代码运行逻辑

### 4.1 系统启动流程

```
上电 / 复位
   │
   ▼
HAL_Init()                          # HAL 库初始化、Systick 配置
   │
   ▼
SystemClock_Config()                # 系统时钟 170MHz (PLL)
   │
   ▼
MX_GPIO_Init()                      # GPIO 初始化（按键引脚）
MX_USART2_UART_Init()               # 调试串口初始化 115200
MX_TIM1_Init()                      # PWM 定时器初始化 20kHz
MX_ADC1_Init()                      # ADC1 三电阻电流采样配置
MX_ADC2_Init()                      # ADC2 母线电压+电位器配置
MX_TIM3_Init()                      # 霍尔传感器定时器初始化
   │
   ▼
drv_hall_init()                     # 启动 TIM3 霍尔接口中断
drv_adc_init()                      # ADC 校准 + 启动注入组/规则组中断
drv_pwm_init()                      # PWM 初始占空比 0%，启动六路 PWM 输出
   │
   ▼
printf("foc driver board, ...")      # 启动信息打印
HAL_Delay(200)                      # 等待系统电源稳定
m_motor_phase_current_offset_calculate()  # 相电流静态偏置校准（等待1s）
   │
   ▼
while(1) 主循环                      # 非实时任务调度
   ├── m_us_radius_calculate()       # 读电位器 → 目标转速 → 方向切换逻辑
   ├── m_motor_execute_ctrl()        # 状态机调度（启停/预充电/执行）
   ├── drv_key_scan()               # 按键扫描与消抖
   └── observer_vofa_debug()         # 实时数据通过 VOFA+ 协议发送
```

### 4.2 关键初始化细节

#### 4.2.1 相电流静态偏置校准

```c
// m_ctrl.c: m_motor_phase_current_offset_calculate()
// 在校准期间 (PHASE_CURRENT_OFFSET_TIME = 20s)，PWM 输出占空比为 0%，
// ADC 持续采集三相电流通道的静态偏置电压（即运放偏置 ≈ 1.65V 对应的 ADC 值），
// 累加求平均后存储为偏移基准值 offset。后续正常的相电流采样值减去该偏移基准值
// 即可得到真实的相电流值。
```

#### 4.2.2 自举电容预充电

```c
// 状态机进入 EXECUTE_MOTOR_START 后，先进行 BOOTSTRAP_BOOST_CHARGING_TIME (200ms)
// 的预充电等待。期间 PWM 输出占空比为 0%（上管全关、下管全开），
// 让预驱芯片 FD6288T 的自举电容充分充电，然后才进入正常 FOC 运行。
```

### 4.3 FOC 实时控制环 (20kHz 中断驱动)

整个 FOC 控制环在 **ADC 注入组转换完成中断回调** 中串行执行，频率严格等于 PWM 频率 20kHz：

```
HAL_ADCEx_InjectedConvCpltCallback()        [mcu_adc_cb.c:132]
   │
   ├── 1. drv_adc0_sample()                 # 读取 ADC1 三路相电流 + ADC2 母线电压
   │
   ├── 2. drv_adc0_filter()                 # 偏置校正/母线电压滤波
   │
   ├── 3. m_phase_current_calculate()       # 三相电流计算（弱相重构）
   │      Ia = (offset - instant) << 3      # ADC值转Q15格式
   │      Ib = (offset - instant) << 3
   │      Ic = (offset - instant) << 3
   │      检测 CCR 最小的相并用基尔霍夫定律重构  # 弱相重构：避免窄脉宽采样
   │
   ├── 4. m_clark_transform()               # Clark 变换
   │      Iα = Ia
   │      Iβ = 1/√3 × Iα + 2/√3 × Ib      # 使用 FPU 硬件加速
   │
   ├── 5. m_foc_algorithm_execute()         # FOC 核心算法 ↓↓↓
   │      │
   │      ├── m_rotor_angle_calculate()     # 转子位置角计算（详见 4.4）
   │      │
   │      ├── m_spd_pid_execute()           # 速度环 PID（详见 4.5）
   │      │
   │      ├── m_park_transform()            # Park 变换 (FPU加速)
   │      │      Id = Iα·cosθ + Iβ·sinθ
   │      │      Iq = Iβ·cosθ - Iα·sinθ
   │      │
   │      ├── m_current_pid_execute()       # 电流环 PID（详见 4.6）
   │      │
   │      ├── m_us_theta_c_calculate()      # Us模长 + 超前角计算
   │      │      Us = sqrt(Ud² + Uq²)       # FPU 硬件 sqrtf()
   │      │      θ_c = atan2(-Ud, Uq)       # 超前角 (FPU atan2f())
   │      │
   │      ├── q_engle = rotor_engle ± 90° ± advance_angle  # 合成电压角度
   │      │
   │      └── m_svpwm_generate()            # SVPWM 生成与输出 ↓↓↓
   │             ├── m_us_sector_calculate()  # 扇区判定
   │             ├── m_ux_uy_uz_calculate()   # FPU 计算 x,y,z
   │             ├── m_ta_tb_calculate()      # 矢量作用时间
   │             ├── m_taout_tbout_tcout()   # 占空比换算
   │             └── __HAL_TIM_SET_COMPARE() # 直接写入 TIM1 CCR 寄存器
   │
   └── 6. m_tick()                          # 系统滴答递减（各种定时器）
```

### 4.4 转子位置角计算 (m_rotor_angle_calculate)

使用霍尔传感器 + **线性插值** 实现连续的角度估算：

```
50us 周期内：
  ├── m_hall_value_get()           # 读取三路霍尔引脚电平 → 0~6 状态编码
  │
  ├── 如果发生了霍尔跳变 (update_sign = true):
  │   ├── 查表获取绝对基准角度      # ROTOR_ANGLE_TABLE_CCW / CW
  │   ├── 读取 TIM3 捕获的 60°时间  # Δt (硬件霍尔自动测量)
  │   ├── 对 60°时间做双重低通滤波  # LPF_CALC, 消除机械震动干扰
  │   ├── 计算恒定角速度插值步长   # rotor_angle_inc = 546133 / Δt
  │   ├── 三段式起步策略:
  │   │   阶段一(未跳变): 死锁在扇区中点，不插值
  │   │   阶段二(第1次跳变): 人为给定最低转速步长 (50 RPM)
  │   │   阶段三(第2次后): 接入真实时间，丝滑闭环
  │   └── 环形缓冲区计算真实转速   # RPM = 60,000,000 / (∑Δti × p)
  │
  └── 如果未发生跳变:
      └── 按上次算出的步长增量插值   # 角度 += rotor_angle_inc
          逐扇区宽度钳位（防止越界）  # SECTOR_MAX_WIDTH_TABLE
```

**转速计算公式**：`RPM = 60,000,000 / (hall_time_sum × MOTOR_POLE_PAIRS)`

### 4.5 速度环 PID (m_spd_pid_execute)

速度环以 **5Hz** (200ms 周期) 执行，采用位置式 PID（带抗积分饱和）：

```c
// m_foc.c: m_spd_pid_execute()
if (速度稳定标记 true 且 spd_pid_cycle_time == 0):
    spd_pid_cycle_time = SPD_PID_CYCLE_TIME (200ms)

    // 目标转速斜坡控制（防止阶跃突变）
    if (目标转速 > 当前设定值):
        设定值 += INC_SPD_RPM (100 RPM/步)
    if (目标转速 < 当前设定值):
        设定值 -= DEC_SPD_RPM (100 RPM/步)

    if (转速有更新):
        speed_update_sign = false

        // 速度环位置式 PID
        m_parallel_position_pid_algorithm(&m_spd_pid_unit)
        // Kp=20000, Ki=200, 抗积分饱和

        // 方向映射
        CCW: iq_target = +spd_pid_q15_out_val
        CW:  iq_target = -spd_pid_q15_out_val
else (速度闭环未开启):
    // 直接使用启动 Iq 电流
    iq_target = START_IQ (400，Q15格式)
```

### 4.6 电流环 PID (m_current_pid_execute)

电流环以 **20kHz** (50us) 执行，使用并联增量式 PID：

```c
// m_foc.c: m_current_pid_execute()
Id 环:
    id_target = 0                               # Id* 始终为 0
    id_actual = LPF_HEAVY_CALC(id, filter)      # 重度低通滤波 (1/8新 + 7/8旧)
    Ud = m_parallel_incremental_pid_algorithm(&m_id_pid_unit)
    # Kp=12000, Ki=800, Kd=0

Iq 环:
    iq_actual = LPF_HEAVY_CALC(iq, filter)      # 重度低通滤波
    Uq = m_parallel_incremental_pid_algorithm(&m_iq_pid_unit)
    # Kp=12000, Ki=800, Kd=0
```

**增量式 PID 公式(并联型)**:
```
ΔU = Kp × (err(n) - err(n-1)) + Ki × err(n)
U(n) = U(n-1) + ΔU
```

### 4.7 PID 算法三种实现

| 算法 | 函数 | 适用场景 | 特点 |
|------|------|----------|------|
| **并联增量式** | [m_parallel_incremental_pid_algorithm](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Src/m_pid.c#L74) | 电流环 Id/Iq | 增量输出，天然无积分饱和，无扰切换 |
| **并联位置式** | [m_parallel_position_pid_algorithm](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Src/m_pid.c#L117) | 速度环 | Kp/Ki/Kd 独立调节，带动态抗积分饱和 |
| **串联型** | [m_series_pid_algorithm](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/motor/Src/m_pid.c#L192) | 备选 | 积分项基于 Kp 输出，结构简单 |

**命令格式说明**：

- 所有 PID 计算使用定点数 Q 格式：
  - Q16 (0.16): 系数 Kp/Ki/Kd (范围 0~65535，精度 1/65536)
  - Q15 (1.15): 实际值/目标值/输出值 (范围 ±32767，精度 1/32768)
- 乘法用 `union_s32` 处理 Q16 × Q15 = Q31 → 取高16位得 Q15

### 4.8 SVPWM 空间矢量调制

```c
// m_svpwm.c: m_svpwm_generate()
输入: us_m (Us模长 Q16), us_angle (电压矢量角度 Q16)

Step 1: 扇区判定 (0~60°, 60°~120°, ... 300°~360°)
Step 2: FPU 硬件计算三角函数
        x = sin(θ)
        y = 0.5×sin(θ) + 0.866×cos(θ)
        z = 0.5×sin(θ) - 0.866×cos(θ)
Step 3: 矢量作用时间计算
        ta = M × T × first_x_y_z
        tb = M × T × second_x_y_z
Step 4: 六扇区查表分配占空比 → taout, tbout, tcout
Step 5: MIN_DUTY_VALUE / MAX_DUTY_VALUE 限幅
Step 6: 直接写 TIM1 CCR 寄存器 (CCR = ARR - duty)
```

### 4.9 主循环任务 (非实时)

主循环中的三个函数与中断 FOC 控制环无关，属于非实时的低速任务：

| 函数 | 周期 | 功能 |
|------|------|------|
| `m_us_radius_calculate()` | 每循环 | 读电位器 ADC → 转速设定值 → 方向切换处理 |
| `m_motor_execute_ctrl()` | 每循环 | 状态机：转速>0→启动，转速=0→停止 |
| `drv_key_scan()` | 每循环 | 按键消抖状态机扫描（10ms 消抖） |
| `observer_vofa_debug()` | 每循环 | VOFA+ 格式数据组包并通过串口发送 |

### 4.10 按键与方向切换

- **START/STOP 按键 (PB12)**：控制电机启停。通过改变电位器转速值间接控制（转速>0 则启动，=0 则停止）
- **CW/CCW 按键 (PB11)**：
  - 停机状态按下 → 立即切换方向
  - 运行中按下 → 先将转速强制降至 0，等待转速低于 400RPM 后停机切换方向，再延时 1000ms 等待惯性停止后重启

---

## 5. 项目调试流程

### 5.1 开发环境配置

| 工具 | 用途 |
|------|------|
| **STM32CubeMX** | 生成 HAL 初始化代码、引脚配置、时钟树 |
| **VS Code** | 代码编辑与项目管理 |
| **ARM GCC Toolchain** | 交叉编译器 (arm-none-eabi-gcc) |
| **CMake + Ninja** | 构建系统 |
| **OpenOCD** | 调试服务器 (ST-Link 连接) |
| **GDB** | 命令行/图形化调试器 |
| **VOFA+** | 串口数据实时波形显示 (Just Float 协议) |

### 5.2 编译与烧录

```bash
# 1. 配置 CMake（首次或清理后）
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
    -G Ninja

# 2. 编译
cmake --build build/Debug

# 3. 烧录 (使用 OpenOCD)
openocd -f openocd.cfg -c "program build/Debug/STM32G431CBT6_BLDC.elf verify reset exit"
```

生成文件路径：`build/Debug/STM32G431CBT6_BLDC.elf`

### 5.3 硬件调试 (OpenOCD + GDB)

**OpenOCD 配置** [openocd.cfg](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/openocd.cfg)：

```
source [find interface/stlink.cfg]
source [find target/stm32g4x.cfg]
reset_config none
```

**启动调试服务**：

```bash
openocd -f openocd.cfg
```

OpenOCD 启动后监听端口 3333 (GDB)。然后在 VS Code 中使用 [launch.json](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/.vscode/launch.json) 中配置的 "OpenOCD Debug" 启动调试。

**故障异常处理**：所有 HardFault / MemManage / BusFault / UsageFault 中断均已添加 printf 打印指示，方便快速定位：

```c
// stm32g4xx_it.c
void HardFault_Handler(void) { while(1) printf("HardFault_IRQn\n"); }
void MemManage_Handler(void) { while(1) printf("MemManage_IRQn\n"); }
void BusFault_Handler(void)  { while(1) printf("BusFault_IRQn\n"); }
void UsageFault_Handler(void){ while(1) printf("UsageFault_IRQn\n"); }
void NMI_Handler(void)       { while(1) printf("NMI_Handler\n"); }
```

### 5.4 VOFA+ 实时数据波形调试

使用 [VOFA+](https://www.vofa-plus.com/) 上位机软件的 **Just Float** 协议（channels 格式），通过串口发送实时观测数据。

**串口参数**：USART2, 115200 bps, 8N1

**主要调试函数**：

#### observer_vofa_debug() — 通用观测器调试（默认开启）

在 [main.c:251-313](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/Core/Src/main.c#L251-L313) 中，通过 `#if 1 / #if 0` 开关切换不同的调试通道：

| 开关 | 发送数据 | 用途 |
|------|----------|------|
| **#if 1** (默认) | `iq_target, iq_actual, id_actual, hall_value, angle_deg, uq, set_spd, spd` | **全8通道**：Id/Iq 电流环跟踪、霍尔状态、角度、Uq 电压、转速闭环 |
| #if 0 (备选1) | `ia, ib, ic` | 三相电流原始波形 |
| #if 0 (备选2) | `delta_theta, filter_delta_theta` | 角度增量滤波效果比对 |
| #if 0 (备选3) | `i_alpha_estimate, i_alpha` | α 轴电流估算跟踪 |
| #if 0 (备选4) | `i_beta_estimate, i_beta` | β 轴电流估算跟踪 |
| #if 0 (备选5) | `theta_e, erpm` | 估计电角度和 eRPM 跟踪 |
| #if 0 (备选6) | `e_alpha_final, e_beta_final` | α/β 反电动势估算 |

#### pid_vofa_debug() — PID 调试

在 [main.c:206-223](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/Core/Src/main.c#L206-L223) 中：

| 开关 | 发送数据 | 用途 |
|------|----------|------|
| #if 1 | `set_spd_val, spd_val` | 速度环设定值 vs 实时值 |
| #if 0 | `iq_target, iq_actual` | Iq 电流环目标值 vs 反馈值 |

#### observer_IaIbIc_vofa_debug() — 高频三相电流同步抓取

在 [main.c:225-243](file:///e:/Cubemx_space/G431CBT6_BLDC/STM32G431CBT6_BLDC/Core/Src/main.c#L225-L243) 中：

通过全局标志位 `debug_print_flag` 实现与 FOC 中断的精确同步，确保抓取的 Ia/Ib/Ic 三维电流数据来自同一时刻的中断周期。

### 5.5 调试建议流程

#### 5.5.1 硬件检查阶段

1. 上电前用万用表检测供电电压是否正常（3.3V / 母线电压）
2. 确认 ST-Link 连接正常
3. 烧录程序后，串口应显示 `"foc driver board, hall svpwm project"`
4. 用示波器检查 TIM1 六路 PWM 输出波形是否正常（20kHz 中心对齐，死区时间 200ns）

#### 5.5.2 相电流静态偏置校准验证

1. 程序启动后会进入 20 秒的偏置校准期（PWM 输出为 0%）
2. 通过 VOFA+ 观察三相电流的静态偏置值是否稳定在理论值附近（运放偏置 1.65V 对应 ADC 值约 2048）
3. 如果偏差过大，检查硬件运放电路

#### 5.5.3 霍尔传感器与角度验证

1. 使用 `observer_vofa_debug() → #if 1` 模式
2. 手动旋转电机轴，观察 `hall_value` 在 1~6 之间正常循环
3. 观察 `angle_deg` 是否在 0°~360° 之间连续变化
4. 检查 CCW/CW 方向下的霍尔顺序是否与查表一致
5. 如果不一致，使用 `EXECUTE_MOTOR_ALIGNMENT_TEST` 模式进行**霍尔锁死测表**，逐扇区标定

#### 5.5.4 电流环 PID 调试

1. 先在 `pid_vofa_debug() → #if 0 (iq_target, iq_actual)` 模式下观察
2. 在速度环未闭环时（起步阶段），Iq 目标值 = START_IQ (400)
3. 调整 Kp (12000) 和 Ki (800)：
   - Kp 偏大 → 电流振荡、电机啸叫
   - Kp 偏小 → 电流响应慢、电机无力
   - Ki 偏大 → 低频振荡
   - 典型调试顺序：先 Kp 后 Ki，Kp 调到不振荡的 70%~80%，再加 Ki

#### 5.5.5 速度环 PID 调试

1. 速度环 200ms 执行周期 (SPD_PID_CYCLE_TIME)，VOFA+ 刷新率稍低
2. 先设置低转速（如 300 RPM），观察 `set_spd` vs `spd_val` 的跟踪
3. 调整 Kp (20000) 和 Ki (200)：
   - 转速超调 → 降低 Kp
   - 转速稳态误差大 → 增大 Ki
   - 转速振荡 → 同时降低 Kp 和 Ki

#### 5.5.6 SVPWM 输出验证

1. 使用示波器同时监测 TIM1 的三路 PWM (CH1/CH2/CH3)
2. 观察在不同转速下 PWM 调制波形是否为典型的 SVPWM 马鞍波形
3. 检查死区时间（200ns）是否正常

### 5.6 常见问题排查

| 故障现象 | 可能原因 | 排查方向 |
|----------|----------|----------|
| 电机不转、无声音 | PWM 未输出或占空比一直为 0 | 检查 `drv_pwm_init()` 是否成功、CCR 值是否为 ARR |
| 电机震动、噪音大 | PID 参数不合适或角度计算有误 | 通过 VOFA+ 观察 id/iq 是否振荡，检查霍尔顺序 |
| 电机堵转/无力 | 电流环 PID 参数过小或供电不足 | 逐步增大 Kp/Ki，测量母线电压 |
| 启动失败 | 静态偏置校准不准或霍尔初始化异常 | 检查偏置校准是否完成 20s，确认 `m_rotor_angle_init()` |
| HardFault | 存储器越界或除零 | 打印故障类型，检查指针操作和除数是否为零 |
| 串口无输出/VOFA+ 无波形 | 串口配置不匹配或 printf 未重定向 | 确认 USART2 为 115200，检查 `_write()` 函数 |

---

## 附录：关键参数速查表

### PID 参数

| PID 对象 | Kp | Ki | Kd | 输出限幅 | 运行频率 |
|----------|-----|------|------|----------|----------|
| Id 电流环 | 12000 | 800 | 0 | ±32767 | 20kHz |
| Iq 电流环 | 12000 | 800 | 0 | ±32767 | 20kHz |
| 速度环 | 20000 | 200 | 0 | ±32767 | 5Hz |

### PWM 参数

| 参数 | 值 |
|------|------|
| 定时器 | TIM1 |
| 频率 | 20kHz |
| 模式 | Center Aligned Mode 2 |
| ARR | 4250 |
| 死区时间 | 34 (≈200ns @170MHz) |
| 最小有效脉宽 | MIN_DUTY_VALUE (34) |
| 最大有效脉宽 | MAX_DUTY_VALUE (4199) |

### 电机参数

| 参数 | 值 |
|------|------|
| 极对数 | 2 |
| 最低转速 | 300 RPM |
| 最高转速 | 2300 RPM |
| Us 模长最大值 | 29491 (Q15, ≈90%) |
| 调制比 M 最大值 | 58982 (Q16, ≈0.9) |
| 启动 Iq 电流 | 400 (Q15) |
| 最小 Iq 设定值 | 20 |

### 状态机时间参数

| 参数 | 值 | 说明 |
|------|------|------|
| BOOTSTRAP_BOOST_CHARGING_TIME | 200ms | 自举电容预充电 |
| PHASE_CURRENT_OFFSET_TIME | 20s | 静态偏置校准时间 |
| SPD_PID_CYCLE_TIME | 200ms (20×30) | 速度环PID周期 |
| HALL_VALUE_TIMEOUT_THRESHOLD_VALUE | 20s | 霍尔超时堵转检测 |
| MOTOR_HALL_STABILIZE_NUMBER | 12 | 速度稳定检测次数 |
| FILTER_KEEP_TIME | 10ms | 按键软件消抖 |

### 霍尔起步三段式策略

| 阶段 | cnt值 | 行为 | 转速 |
|------|-------|------|------|
| 阶段一 | 0 | 死锁在扇区中点，最大恒定推力 | 0 RPM |
| 阶段二 | 1 | 人为给定最低转速步长（防止死锁） | 50 RPM |
| 阶段三 | ≥2 | 接入真实 60° 时间，正常闭环 | 正常运行 |

---

*文档生成日期: 2026-05-19*
*项目基于 STM32CubeMX 生成，HAL 库版本 STM32G4xx_HAL_Driver*