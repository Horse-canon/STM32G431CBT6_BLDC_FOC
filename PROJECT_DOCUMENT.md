# STM32G431CBT6 BLDC 电机控制项目文档

## 1. 项目概述

本项目基于 STM32G431CBT6 微控制器实现 BLDC（无刷直流电机）的开环 SVPWM（空间矢量脉宽调制）控制。项目已完成开环 SVPWM 调试，运行状态良好。

### 主要功能
- 基于 STM32G431CBT6 的 BLDC 电机控制
- 开环 SVPWM 驱动
- 霍尔传感器信号采集
- 电流/电压采样
- 串口通信

## 2. 硬件配置

### 2.1 芯片信息
- **芯片型号**：STM32G431CBT6
- **封装**：LQFP48
- **主频**：170MHz
- **Flash**：128KB
- **RAM**：32KB

### 2.2 STM32CubeMX 管脚配置

| 管脚 | 模式 | 功能 | 用途 |
|------|------|------|------|
| PA0 | IN1-Single-Ended | ADC1_IN1 | 相电流采样 |
| PA1 | IN2-Single-Ended | ADC1_IN2 | 相电流采样 |
| PA2 | IN3-Single-Ended | ADC1_IN3 | 相电流采样 |
| PA4 | IN17-Single-Ended | ADC2_IN17 | 母线电压采样 |
| PA5 | IN13-Single-Ended | ADC2_IN13 | 速度调节电位器 |
| PA6 | Xored_Inputs_Hall_Sensor_Interface | TIM3_CH1 | 霍尔传感器信号 |
| PA7 | Xored_Inputs_Hall_Sensor_Interface | TIM3_CH2 | 霍尔传感器信号 |
| PA8 | PWM Generation1 CH1 CH1N | TIM1_CH1 | PWM 输出（上桥臂） |
| PA9 | PWM Generation2 CH2 CH2N | TIM1_CH2 | PWM 输出（上桥臂） |
| PA10 | PWM Generation3 CH3 CH3N | TIM1_CH3 | PWM 输出（上桥臂） |
| PA13 | Serial_Wire | SYS_JTMS-SWDIO | 调试接口 |
| PA14 | Serial_Wire | SYS_JTCK-SWCLK | 调试接口 |
| PB0 | Xored_Inputs_Hall_Sensor_Interface | TIM3_CH3 | 霍尔传感器信号 |
| PB3 | Asynchronous | USART2_TX | 串口发送 |
| PB4 | Asynchronous | USART2_RX | 串口接收 |
| PB13 | PWM Generation1 CH1 CH1N | TIM1_CH1N | PWM 输出（下桥臂） |
| PB14 | PWM Generation2 CH2 CH2N | TIM1_CH2N | PWM 输出（下桥臂） |
| PB15 | PWM Generation3 CH3 CH3N | TIM1_CH3N | PWM 输出（下桥臂） |
| PF0 | HSE-External-Oscillator | RCC_OSC_IN | 外部晶振输入 |
| PF1 | HSE-External-Oscillator | RCC_OSC_OUT | 外部晶振输出 |

### 2.3 时钟配置
- **外部晶振**：8MHz
- **PLL 配置**：
  - PLLM：2 (8MHz / 2 = 4MHz)
  - PLLN：85 (4MHz * 85 = 340MHz)
  - PLLP：2 (340MHz / 2 = 170MHz)
- **系统时钟**：170MHz
- **APB1/APB2 时钟**：170MHz

### 2.4 外设配置

#### 2.4.1 TIM1（PWM 输出）
- **计数器模式**：中心对齐模式 2
- **周期**：4250（约 20kHz PWM 频率）
- **死区时间**：203
- **触发输出**：TIM_TRGO_UPDATE（用于触发 ADC 采样）
- **通道模式**：PWM 模式 2

#### 2.4.2 TIM3（霍尔传感器）
- **预分频器**：169
- **输入捕获滤波器**：15

#### 2.4.3 ADC1
- **注入通道**：3 个通道
- **外部触发**：TIM1_TRGO
- **采样时间**：12.5 个周期

#### 2.4.4 ADC2
- **规则通道**：1 个通道（ADC2_IN13）
- **注入通道**：1 个通道（ADC2_IN17）
- **外部触发**：TIM1_TRGO
- **采样时间**：规则通道 640.5 个周期，注入通道 24.5 个周期

#### 2.4.5 USART2
- **模式**：异步通信
- **波特率**：默认配置

## 3. 项目架构

```
STM32G431CBT6_BLDC/
├── Core/
│   ├── Inc/          # 核心头文件
│   └── Src/          # 核心源代码
│       └── main.c     # 主程序
├── Drivers/          # STM32 官方驱动
│   ├── CMSIS/        # CMSIS 标准库
│   └── STM32G4xx_HAL_Driver/  # HAL 驱动
├── bsp/              # 板级支持包
│   ├── Inc/          # BSP 头文件
│   │   ├── mcu_adc.h         # ADC 驱动
│   │   ├── mcu_hall.h        # 霍尔传感器驱动
│   │   ├── mcu_pwm.h         # PWM 驱动
│   │   └── mcu_uart.h        # UART 驱动
│   └── Src/          # BSP 源代码
│       ├── mcu_adc.c         # ADC 实现
│       ├── mcu_hall.c        # 霍尔传感器实现
│       ├── mcu_pwm.c         # PWM 实现
│       └── mcu_uart.c        # UART 实现
├── motor/            # 电机控制算法
│   ├── Inc/          # 算法头文件
│   │   ├── m_ctrl.h          # 电机控制
│   │   ├── m_foc.h           # FOC 算法
│   │   ├── m_parameter.h     # 电机参数
│   │   ├── m_rotor_angle.h   # 转子角度计算
│   │   └── m_svpwm.h         # SVPWM 实现
│   └── Src/          # 算法源代码
│       ├── m_ctrl.c          # 电机控制实现
│       ├── m_foc.c           # FOC 算法实现
│       ├── m_rotor_angle.c   # 转子角度计算实现
│       └── m_svpwm.c         # SVPWM 实现
├── CMakeLists.txt    # CMake 配置
├── STM32G431CBT6_BLDC.ioc  # STM32CubeMX 配置
└── openocd.cfg       # OpenOCD 调试配置
```

## 4. 核心函数功能

### 4.1 主程序相关

#### `main()` 函数
- **功能**：系统初始化和主循环
- **流程**：
  1. 初始化 HAL 库
  2. 配置系统时钟
  3. 初始化 GPIO、USART2、TIM1、ADC1、ADC2、TIM3
  4. 初始化霍尔传感器、ADC、PWM
  5. 设置电机方向为逆时针（CCW）
  6. 打印初始化信息
  7. 进入主循环，执行 SVPWM 计算和电机控制

#### `SystemClock_Config()` 函数
- **功能**：配置系统时钟为 170MHz
- **实现**：使用 HSE 作为 PLL 输入，通过 PLL 倍频到 170MHz

### 4.2 电机控制相关

#### `m_motor_execute_ctrl()` 函数
- **功能**：电机控制状态机执行
- **状态**：
  - `EXECUTE_MOTOR_STOP`：电机停止
  - `EXECUTE_MOTOR_START`：电机启动
  - `EXECUTE_MOTOR_BOOST_CHARGING`：等待充电
  - `EXECUTE_MOTOR_EXECUTE`：电机正常运行

#### `m_us_radius_calculate()` 函数
- **功能**：计算 SVPWM 的空间矢量半径
- **说明**：开环控制中，根据设定的速度值计算 SVPWM 的占空比

#### `m_foc_algorithm_execute()` 函数
- **功能**：执行 FOC（磁场定向控制）算法
- **说明**：处理电流采样数据，计算转子角度，生成 SVPWM 信号

### 4.3 驱动层相关

#### `drv_hall_init()` 函数
- **功能**：初始化霍尔传感器接口
- **实现**：配置 TIM3 为霍尔传感器模式，用于捕获转子位置

#### `drv_adc_init()` 函数
- **功能**：初始化 ADC 模块
- **实现**：配置 ADC1 和 ADC2，用于电流和电压采样

#### `drv_pwm_init()` 函数
- **功能**：初始化 PWM 输出
- **实现**：配置 TIM1 为 PWM 模式，设置死区时间，启动 PWM 输出

## 5. 开环 SVPWM 实现

### 5.1 SVPWM 原理

SVPWM（空间矢量脉宽调制）是一种高效的 PWM 控制技术，通过控制逆变器的六个开关管，在电机定子中产生圆形旋转磁场。

### 5.2 实现方式

1. **电压空间矢量生成**：根据参考电压矢量的角度和幅值，选择合适的基本空间矢量
2. **扇区判断**：确定参考电压矢量所在的扇区
3. **作用时间计算**：计算各基本矢量的作用时间
4. **PWM 占空比计算**：根据作用时间计算各相的 PWM 占空比

### 5.3 开环控制特点

- **优势**：实现简单，无需电流反馈
- **劣势**：速度控制精度较低，负载变化时可能失步
- **应用场景**：适用于对速度精度要求不高的场合

## 6. 遇到的问题及解决方案

### 6.1 问题一：PWM 输出异常
- **现象**：电机不转或转动异常
- **原因**：
  1. PWM 死区时间设置不当
  2. 上下桥臂 PWM 信号不同步
  3. 触发信号配置错误
- **解决方案**：
  1. 调整 TIM1 的死区时间（当前设置为 203）
  2. 确保使用中心对齐模式（当前使用模式 2）
  3. 正确配置 TIM_TRGO_UPDATE 触发信号

### 6.2 问题二：ADC 采样不准确
- **现象**：电流/电压采样值波动较大
- **原因**：
  1. 采样时间过短
  2. 信号干扰
  3. 触发时机不当
- **解决方案**：
  1. 增加采样时间（规则通道使用 640.5 个周期）
  2. 增加硬件滤波电路
  3. 使用 TIM1 触发 ADC 采样，确保在 PWM 空闲时间采样

### 6.3 问题三：霍尔传感器信号异常
- **现象**：转子位置检测不准确
- **原因**：
  1. 霍尔传感器安装位置不当
  2. 信号滤波不足
  3. TIM3 配置不当
- **解决方案**：
  1. 调整霍尔传感器安装位置
  2. 增加输入捕获滤波器（当前设置为 15）
  3. 正确配置 TIM3 的预分频器和工作模式

### 6.4 问题四：电机启动困难
- **现象**：电机启动时抖动或无法启动
- **原因**：
  1. 启动电压不足
  2. 初始角度计算错误
  3. 启动时序不当
- **解决方案**：
  1. 增加启动时的电压幅值
  2. 优化初始角度检测算法
  3. 实现软启动功能，逐步增加电压

## 7. 调试方法

### 7.1 硬件调试
- **示波器**：观察 PWM 波形、电流波形
- **万用表**：测量电压、电流值
- **逻辑分析仪**：分析霍尔传感器信号

### 7.2 软件调试
- **OpenOCD + GDB**：使用 ST-Link 进行在线调试
- **串口调试**：通过 USART2 输出调试信息
- **断点调试**：在关键函数处设置断点，观察变量值

### 7.3 调试配置

```bash
# 启动 OpenOCD
openocd -f openocd.cfg

# 启动 GDB
arm-none-eabi-gdb build/Debug/STM32G431CBT6_BLDC.elf

# GDB 命令
target remote localhost:3333
load
monitor reset halt
break m_us_radius_calculate
continue
```

## 8. 性能指标

- **PWM 频率**：约 20kHz
- **控制周期**：主循环执行周期
- **系统时钟**：170MHz
- **代码大小**：约 1.5MB（包含调试信息）

## 9. 后续优化方向

1. **闭环控制**：添加电流反馈，实现闭环 FOC 控制
2. **速度控制**：添加速度反馈，实现速度闭环控制
3. **参数自整定**：实现电机参数自动识别
4. **故障保护**：添加过流、过压、过热保护
5. **效率优化**：优化 PWM 调制策略，提高效率

## 10. 总结

本项目成功实现了基于 STM32G431CBT6 的 BLDC 电机开环 SVPWM 控制。通过合理的硬件配置和软件设计，电机能够稳定运行。项目架构清晰，代码组织合理，为后续的功能扩展和性能优化奠定了良好的基础。

开环 SVPWM 控制虽然简单，但在实际应用中需要注意各种细节问题，如死区时间设置、ADC 采样时机、霍尔传感器信号处理等。通过本次调试，我们积累了丰富的 BLDC 电机控制经验，为后续的闭环控制实现做好了准备。