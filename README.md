# STM32F407 机器人项目

这个仓库包含从 `gcs-gold-medal-main/yyb_stm32` 整理的 Keil 固件、原项目的视觉识别脚本和 PCB 设计资料。固件仍包含原车比赛任务逻辑；移植到另一块板前须核对引脚、供电、电机地址及机构运动范围。

## 默认行为

仓库中的 [`USER/app_config.h`](USER/app_config.h) 默认只启用 `APP_SAFE_IDLE=1`。上电后初始化延时并保持空闲，不会驱动舵机或底盘。要选择一个功能入口，把空闲模式设为 `0`，并且只将下表中的**一个**模式设为 `1`。配置头会在选择多个或没有选择模式时给出编译错误。

| 模式宏 | 行为 |
| --- | --- |
| `APP_SAFE_IDLE` | 安全空闲；默认值 |
| `PA2_PA3_SERVO_TEST` | PA2/PA3 舵机测试；PA3 为 838 ↔ 878 µs、PA2 为 648 ↔ 688 µs；`PA3_FAST_SWEEP_TEST=1` 时仅 PA3 往返，PA2 保持初值 |
| `CHASSIS_CAN_MOTOR_TEST` | 旧四轮 CAN 速度测试，原车地址为 2/1/3/4；当前装配未使用 CAN 电机 |
| `CHASSIS_POWER_ON_TEST` | 原车两轮固定路线；需要 IMU、CAN 电机与相应机构 |
| `APP_LEGACY_TASK` | 原比赛任务入口；依赖多种外设和树莓派通信 |

`CAN_INTERNAL_LOOPBACK_TEST` 可在 CAN 测试时选择 MCU 内部回环。值为 `1` 时外部电机不会收到报文。

当前装配由 PA3 驱动升降、PA2 驱动夹爪，CAN 控制的电机暂未使用。启用舵机测试前须按实际机构核对供电和限位；缩小后的测试脉宽仍需现场确认。

## 构建

1. 使用 Keil µVision 5 打开 [`USER/Template.uvprojx`](USER/Template.uvprojx)。原项目目标芯片为 STM32F407ZG，使用 Arm Compiler 6.22；工程引用目录内的标准外设库与 CMSIS 源码。
2. 按硬件选择 [`USER/app_config.h`](USER/app_config.h) 的模式并检查 [`引脚配置汇总.md`](引脚配置汇总.md)。
3. 在 Keil 执行 Rebuild，核对 Error(s) 和 Warning(s)，然后再烧录对应硬件。

`USER/Template.uvprojx` 内使用相对路径；生成文件写入 `OBJ/`，该目录已被 Git 忽略。`*.uvoptx` 和 `*.uvguix*` 是各台电脑的本地设置，也不纳入版本控制。

## 移植入口

具体修改位置见 [`docs/移植检查表.md`](docs/移植检查表.md)。常用模块：

- `can_ZDT/`：CAN1 收发、ZDT 电机协议与原车轮序；
- `Hardware/`：舵机 PWM、开关和航向 PID；当前装配定义 PA3 控制升降、PA2 控制夹爪，旧源码名称仍为云台和物块平台；
- `SYSTEM/Serial.c`：USART1 与上位机/树莓派通信；
- `imu/`：HWT101、USART2 与补光灯 TIM3；
- `ckp/`、`lanya/`、`OLED/`、`LED/`：屏幕、UART5、OLED 与指示灯。

视觉识别脚本见 [`cv2_python/README.md`](cv2_python/README.md)，PCB 文件见 [`PCB/README.md`](PCB/README.md)。当前装配见 [`docs/方案说明.md`](docs/方案说明.md)，实际板卡上电和接线先看 [`docs/转接板使用手册.md`](docs/转接板使用手册.md)。

## 来源与发布

此仓库由工作区内现有 `yyb_stm32` 工程整理而来，其中含 STM32 标准外设库、CMSIS、ZDT 电机示例代码和其他第三方内容。保留原文件中的署名与说明。当前没有可确认的统一授权许可，因此此仓库先作为**本地复用模板**；公开发布前应逐项核实第三方文件的许可与来源。此处没有替原作者声明新的许可。
