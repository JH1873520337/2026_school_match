# `scool_match` 固件分层架构草案

## 1. 文档目的

这份文档不是通用模板，而是基于当前仓库的真实现状整理出的可落地架构草案。

当前已经确认的事实：

- 芯片是 `STM32F407VET6`
- 工程由 `STM32CubeMX + CMake + FreeRTOS(CMSIS-RTOS2)` 生成
- `Core/`、`Drivers/`、`Middlewares/`、`scool_match.ioc` 属于 CubeMX 主干
- `Bsp/` 已有驱动壳子，但基本还是空实现
- `APP/`、`Algorithm/`、`Module/` 目前为空目录
- 当前已初始化外设包括：
  - `SPI1` -> `ICM42688P`
  - `TIM1/TIM2/TIM4/TIM5` -> 4 路编码器
  - `TIM9/TIM12` -> TB6612 PWM 输出
  - `UART4` -> camera
  - `UART5` -> screen
  - `PB2` -> buzzer

当前题目 PDF 不能直接读取正文，所以本文档先按“简易绳驱巡检装置”这一类典型控制系统来设计。后续如果补充题目细则，只需要在 APP 状态机和业务流程上收敛，不需要推翻整个分层。

---

## 2. 总体分层

建议按下面 5 层组织：

1. `Core/`：启动、时钟、外设初始化、中断、FreeRTOS 框架入口
2. `Bsp/`：板级驱动层，直接面向外设和芯片资源
3. `Module/`：服务层/功能模块层，组合多个 BSP 驱动形成稳定能力
4. `Algorithm/`：纯算法层，输入数据、输出控制量，不直接碰 HAL
5. `APP/`：RTOS 应用层，负责任务划分、状态机、业务流程

一句话理解：

- `Bsp` 解决“怎么读写硬件”
- `Module` 解决“把硬件组合成什么能力”
- `Algorithm` 解决“怎么计算”
- `APP` 解决“什么时候做什么”

---

## 3. 当前工程下推荐的目录职责

### 3.1 `Core/`

保留 CubeMX 生成逻辑，只写很薄的一层 glue code。

应该放的内容：

- `main.c`：初始化后调用应用总入口
- `freertos.c`：创建任务、队列、事件组、互斥锁
- `stm32f4xx_it.c`：中断入口和极少量 ISR 逻辑
- `main.h`：CubeMX 生成的引脚定义

不应该放的内容：

- 电机闭环
- 视觉解析业务逻辑
- 姿态融合
- 整个赛题状态机

建议做法：

- `main.c` 的 `USER CODE BEGIN 2` 只调用 `App_Init()`
- `freertos.c` 的 `USER CODE BEGIN RTOS_THREADS` 中创建应用任务
- `stm32f4xx_it.c` 只做中断转发、置标志、发信号，不做重计算

### 3.2 `Bsp/`

这是板级驱动层。

职责：

- 直接调用 HAL
- 面向具体器件或具体外设
- 提供稳定、简单、无业务语义的接口

这一层的接口应该像：

- `TB6612_SetMotorPwm(...)`
- `Encoder_GetCount(...)`
- `ICM42688_ReadRaw(...)`
- `CameraUart_PushRxByte(...)`

不应该出现的内容：

- “巡检模式”
- “目标丢失”
- “自动爬升”
- “报警状态机”

### 3.3 `Module/`，建议作为服务层使用

仓库里已经有 `Module/` 目录，建议直接把它当作“服务层”。

也就是说：

- 物理目录名继续用 `Module/`
- 逻辑上把它视为 `Service Layer`

职责：

- 封装多个 BSP 驱动
- 输出更接近业务的系统能力
- 屏蔽底层设备细节

这一层的接口应该像：

- `Motor_UpdateFeedback()`
- `Chassis_SetSpeed(left, right)`
- `Imu_Update()`
- `Vision_GetResult()`
- `Display_PublishStatus()`

### 3.4 `Algorithm/`

职责：

- 只放算法
- 不依赖 GPIO/UART/SPI/TIM 句柄
- 输入普通数据结构，输出普通数据结构

适合放在这里的内容：

- PID
- 滤波
- 姿态解算
- 路径/目标跟踪控制律
- 故障检测判据

### 3.5 `APP/`

这是 RTOS 应用层。

职责：

- 创建和组织任务
- 定义运行模式
- 驱动状态机
- 串联传感、控制、通信、显示、告警

APP 层不应该直接操作某个 GPIO 引脚。

APP 层应该调用：

- `Module/` 提供的服务
- `Algorithm/` 提供的算法

---

## 4. 推荐文件树

下面是基于当前工程的推荐文件树，不要求一次建完，但建议按这个方向扩展。

```text
scool_match/
|-- Core/
|   |-- Inc/
|   |-- Src/
|
|-- Bsp/
|   |-- bsp_common.h
|   |-- bsp_common.c
|   |-- tb6612.h
|   |-- tb6612.c
|   |-- encoder.h
|   |-- encoder.c
|   |-- icm42688p.h
|   |-- icm42688p.c
|   |-- camera_uart.h
|   |-- camera_uart.c
|   |-- screen_uart.h
|   |-- screen_uart.c
|   |-- buzzer.h
|   |-- buzzer.c
|
|-- Module/
|   |-- module_types.h
|   |-- module_status.h
|   |-- module_status.c
|   |-- motor.h
|   |-- motor.c
|   |-- chassis.h
|   |-- chassis.c
|   |-- imu.h
|   |-- imu.c
|   |-- vision.h
|   |-- vision.c
|   |-- display.h
|   |-- display.c
|   |-- alarm.h
|   |-- alarm.c
|
|-- Algorithm/
|   |-- algo_types.h
|   |-- pid.h
|   |-- pid.c
|   |-- filter.h
|   |-- filter.c
|   |-- attitude.h
|   |-- attitude.c
|   |-- tracking_ctrl.h
|   |-- tracking_ctrl.c
|   |-- fault_detect.h
|   |-- fault_detect.c
|
|-- APP/
|   |-- app.h
|   |-- app.c
|   |-- app_config.h
|   |-- app_data.h
|   |-- app_state_machine.h
|   |-- app_state_machine.c
|   |-- app_task_sensor.h
|   |-- app_task_sensor.c
|   |-- app_task_control.h
|   |-- app_task_control.c
|   |-- app_task_comm.h
|   |-- app_task_comm.c
|   |-- app_task_ui.h
|   |-- app_task_ui.c
|   |-- app_task_guard.h
|   |-- app_task_guard.c
```

---

## 5. 每一层该实现什么

## 5.1 BSP 层设计

### `bsp_common.h` / `bsp_common.c`

用途：

- 提供公共宏和基础工具
- 统一 `bool`、错误码、限幅宏、时间戳接口

建议内容：

- `CLAMP(x, min, max)`
- `ARRAY_SIZE(x)`
- `Bsp_GetTickMs()`
- `bsp_status_t`

### `tb6612.h` / `tb6612.c`

用途：

- 4 路电机 PWM 和方向控制

建议职责：

- 初始化 TB6612 所需 GPIO/PWM 通道状态
- 设置单电机输出方向和占空比
- 提供统一电机编号
- 提供急停和全部停止

建议接口：

```c
typedef enum {
    MOTOR_ID_1 = 0,
    MOTOR_ID_2,
    MOTOR_ID_3,
    MOTOR_ID_4,
    MOTOR_ID_MAX
} motor_id_t;

void TB6612_Init(void);
void TB6612_SetMotorPwm(motor_id_t id, int16_t pwm);
void TB6612_StopMotor(motor_id_t id);
void TB6612_StopAll(void);
```

约束：

- 输入 `pwm > 0` 表示正转
- 输入 `pwm < 0` 表示反转
- 输入 `pwm == 0` 表示停止
- 驱动层只做映射，不做速度闭环

### `encoder.h` / `encoder.c`

用途：

- 4 路编码器原始计数读取

建议职责：

- 读取各定时器计数值
- 提供差分计数
- 提供清零接口

建议接口：

```c
void Encoder_Init(void);
int32_t Encoder_GetCount(motor_id_t id);
int32_t Encoder_GetDeltaCount(motor_id_t id);
void Encoder_Reset(motor_id_t id);
```

约束：

- `encoder.c` 只输出原始或半原始数据
- 不在这里做速度滤波和 PID

### `icm42688p.h` / `icm42688p.c`

用途：

- IMU 底层寄存器访问和原始数据读取

建议职责：

- SPI 读写封装
- 设备 ID 校验
- 基本初始化
- 原始加速度、陀螺仪读取
- 中断到达标志管理

建议接口：

```c
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
} icm42688_raw_t;

bool ICM42688_Init(void);
bool ICM42688_ReadRaw(icm42688_raw_t *raw);
void ICM42688_SetDataReadyFlag(void);
bool ICM42688_FetchDataReadyFlag(void);
```

建议说明：

- EXTI4 中断只调用 `ICM42688_SetDataReadyFlag()`
- 真正 SPI 读取放到任务里做

### `camera_uart.h` / `camera_uart.c`

用途：

- 摄像头串口收发和帧缓存

建议职责：

- UART 接收字节缓存
- 帧头/帧尾/长度校验
- 输出一帧完整视觉原始报文

建议接口：

```c
void CameraUart_Init(void);
void CameraUart_RxByteIRQ(uint8_t byte);
bool CameraUart_GetFrame(uint8_t *buf, uint16_t *len);
bool CameraUart_Send(const uint8_t *buf, uint16_t len);
```

### `screen_uart.h` / `screen_uart.c`

用途：

- 串口屏底层协议发送

建议职责：

- 文本刷新
- 页面切换
- 数值更新

建议接口：

```c
void Screen_Init(void);
void Screen_ShowPage(uint8_t page_id);
void Screen_SetText(uint16_t widget_id, const char *text);
void Screen_SetValue(uint16_t widget_id, int32_t value);
```

### `buzzer.h` / `buzzer.c`

用途：

- 蜂鸣器控制

建议职责：

- 开/关
- 固定时长鸣叫
- 节奏播放

建议接口：

```c
void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_RequestBeep(uint16_t on_ms);
```

---

## 5.2 服务层 `Module/` 设计

这一层把 BSP 的“设备接口”变成“可直接使用的系统能力”。

### `module_types.h`

建议放：

- 通用模块数据结构
- 姿态、速度、视觉结果等标准结构体

示例：

```c
typedef struct {
    float speed_rps;
    int32_t count;
} motor_feedback_t;

typedef struct {
    float pitch_deg;
    float roll_deg;
    float yaw_rate_dps;
} imu_attitude_t;

typedef struct {
    uint8_t valid;
    int16_t offset_x;
    int16_t offset_y;
    uint16_t width;
    uint16_t height;
} vision_result_t;
```

### `module_status.h` / `module_status.c`

用途：

- 存放系统运行状态快照
- 作为各任务之间共享的轻量状态中心

建议内容：

- 当前模式
- 运行标志
- 故障标志
- 最近一次传感器结果

### `motor.h` / `motor.c`

用途：

- 单电机抽象

依赖：

- `tb6612`
- `encoder`
- `pid`
- `filter`

建议职责：

- 每个电机的目标速度管理
- 编码器反馈更新
- 速度计算和滤波
- 速度环输出到 PWM

建议接口：

```c
void Motor_InitAll(void);
void Motor_SetTargetSpeed(motor_id_t id, float speed_rps);
float Motor_GetTargetSpeed(motor_id_t id);
float Motor_GetActualSpeed(motor_id_t id);
void Motor_UpdateFeedback(void);
void Motor_RunControl(void);
void Motor_StopAll(void);
```

说明：

- `Motor_UpdateFeedback()` 读取编码器并换算速度
- `Motor_RunControl()` 计算 PID 并调用 `TB6612_SetMotorPwm()`

### `chassis.h` / `chassis.c`

用途：

- 四电机联动控制

建议职责：

- 统一定义车体或绳驱装置的运动接口
- 把线速度/角速度映射到各电机目标速度
- 处理方向反相、安装差异

建议接口：

```c
void Chassis_Init(void);
void Chassis_SetWheelSpeed(float m1, float m2, float m3, float m4);
void Chassis_SetDifferential(float left, float right);
void Chassis_Stop(void);
```

如果装置最终不是“底盘模型”，也依然建议保留这一层，因为它很适合承接“多电机协同关系”。

### `imu.h` / `imu.c`

用途：

- IMU 服务层

依赖：

- `icm42688p`
- `filter`
- `attitude`

建议职责：

- 标定零偏
- 原始值转换到物理单位
- 姿态角或角速度更新
- 输出稳定的姿态数据给 APP

建议接口：

```c
bool Imu_Init(void);
void Imu_StartCalibration(void);
void Imu_Update(void);
imu_attitude_t Imu_GetAttitude(void);
```

### `vision.h` / `vision.c`

用途：

- 视觉服务层

依赖：

- `camera_uart`

建议职责：

- 解析摄像头协议帧
- 输出统一视觉结果结构
- 管理超时、目标丢失、结果有效位

建议接口：

```c
void Vision_Init(void);
void Vision_Update(void);
vision_result_t Vision_GetResult(void);
bool Vision_IsOnline(void);
```

### `display.h` / `display.c`

用途：

- 屏幕显示服务层

建议职责：

- 把系统状态映射为屏幕内容
- 不把页面细节暴露给 APP

建议接口：

```c
void Display_Init(void);
void Display_ShowBoot(void);
void Display_ShowRuntime(void);
void Display_ShowFault(uint32_t fault_mask);
void Display_UpdatePeriodic(void);
```

### `alarm.h` / `alarm.c`

用途：

- 告警服务层

建议职责：

- 根据故障码触发蜂鸣策略
- 提供统一请求接口

建议接口：

```c
void Alarm_Init(void);
void Alarm_ReportFault(uint32_t fault_mask);
void Alarm_Clear(void);
void Alarm_TaskTick(void);
```

---

## 5.3 算法层 `Algorithm/` 设计

这一层尽量做到“可离线测试”。

### `algo_types.h`

建议放：

- 所有算法共享的数据类型
- 控制输入输出结构体

### `pid.h` / `pid.c`

用途：

- 电机速度环/位置环 PID

建议职责：

- 初始化参数
- 限幅
- 积分分离或抗积分饱和

建议接口：

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float out_min, float out_max);
void PID_Reset(pid_t *pid);
float PID_Update(pid_t *pid, float target, float feedback, float dt_s);
```

### `filter.h` / `filter.c`

用途：

- 一阶低通、滑动平均、限幅变化率

建议职责：

- 编码器速度滤波
- IMU 数据去抖
- 控制目标斜坡限制

建议接口：

```c
float LPF1_Update(float input, float last_output, float alpha);
float Ramp_Limit(float target, float current, float step);
```

### `attitude.h` / `attitude.c`

用途：

- 姿态估算

建议职责：

- 互补滤波版本先落地
- 后续可升级为更复杂融合

建议接口：

```c
void Attitude_Init(void);
void Attitude_Update(float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt_s);
float Attitude_GetPitchDeg(void);
float Attitude_GetRollDeg(void);
float Attitude_GetYawRateDps(void);
```

### `tracking_ctrl.h` / `tracking_ctrl.c`

用途：

- 视觉或巡检目标跟踪控制律

建议职责：

- 输入目标偏差
- 输出左右轮速度补偿或车体角速度指令

建议接口：

```c
typedef struct {
    float left_speed;
    float right_speed;
} tracking_output_t;

tracking_output_t TrackingCtrl_Update(int16_t offset_x, uint8_t valid);
```

### `fault_detect.h` / `fault_detect.c`

用途：

- 故障检测判据

建议职责：

- IMU 超限
- 电机堵转
- 视觉超时
- 编码器异常

建议接口：

```c
uint32_t FaultDetect_Run(void);
```

建议故障位定义：

```c
#define FAULT_NONE            0x00000000u
#define FAULT_IMU_OFFLINE     0x00000001u
#define FAULT_VISION_OFFLINE  0x00000002u
#define FAULT_MOTOR_STALL     0x00000004u
#define FAULT_ATTITUDE_LIMIT  0x00000008u
```

---

## 5.4 APP 层与 RTOS 设计

APP 层建议把“任务”和“状态机”拆开。

任务负责周期执行。

状态机负责业务决策。

### `app.h` / `app.c`

用途：

- APP 总入口

建议职责：

- 调用各层初始化
- 创建 RTOS 所需对象
- 提供给 `main.c` 的唯一调用入口

建议接口：

```c
void App_Init(void);
```

建议初始化顺序：

1. `Bsp` 初始化
2. `Module` 初始化
3. `Algorithm` 状态初始化
4. RTOS 对象初始化
5. 任务创建

### `app_config.h`

用途：

- 系统常量集中管理

建议内容：

- 控制周期
- UI 刷新周期
- 安全阈值
- 目标速度上下限
- 姿态阈值

示例：

```c
#define APP_TASK_SENSOR_PERIOD_MS   5
#define APP_TASK_CONTROL_PERIOD_MS  5
#define APP_TASK_COMM_PERIOD_MS    10
#define APP_TASK_UI_PERIOD_MS     100
#define APP_TASK_GUARD_PERIOD_MS   20

#define APP_PITCH_LIMIT_DEG        20.0f
#define APP_ROLL_LIMIT_DEG         20.0f
```

### `app_data.h`

用途：

- APP 共享数据结构

建议内容：

- 全局运行模式
- 目标命令
- 任务间共享状态
- 事件标志定义

建议数据类型：

```c
typedef enum {
    APP_MODE_IDLE = 0,
    APP_MODE_READY,
    APP_MODE_RUN,
    APP_MODE_TRACK,
    APP_MODE_FAULT,
    APP_MODE_STOP
} app_mode_t;
```

### `app_state_machine.h` / `app_state_machine.c`

用途：

- 整个装置的业务状态机核心

建议职责：

- 根据传感器结果、故障码、外部命令切换模式
- 生成高层控制目标

建议状态：

1. `IDLE`
2. `READY`
3. `RUN`
4. `TRACK`
5. `FAULT`
6. `STOP`

建议每个状态含义：

- `IDLE`：上电待命，不驱动运动
- `READY`：自检完成，允许进入工作状态
- `RUN`：按固定速度或基本控制逻辑运动
- `TRACK`：进入视觉或巡检目标跟踪
- `FAULT`：检测到故障，进行限速/停车/报警
- `STOP`：人工或系统主动停机

建议接口：

```c
void AppStateMachine_Init(void);
void AppStateMachine_Step(void);
app_mode_t AppStateMachine_GetMode(void);
```

### `app_task_sensor.h` / `app_task_sensor.c`

用途：

- 传感任务

建议职责：

- 周期更新 IMU
- 周期更新电机反馈
- 拉取视觉结果
- 更新时间戳和在线状态

推荐周期：

- `5 ms`

主要调用：

- `Imu_Update()`
- `Motor_UpdateFeedback()`
- `Vision_Update()`

### `app_task_control.h` / `app_task_control.c`

用途：

- 控制任务

建议职责：

- 跑状态机
- 根据模式生成目标速度
- 调用底层电机控制

推荐周期：

- `5 ms`

主要调用：

- `AppStateMachine_Step()`
- `TrackingCtrl_Update()`
- `Chassis_SetDifferential(...)`
- `Motor_RunControl()`

### `app_task_comm.h` / `app_task_comm.c`

用途：

- 通信任务

建议职责：

- 处理摄像头命令发送
- 处理屏幕非周期显示请求
- 预留未来调试串口接口

推荐周期：

- `10 ms`

### `app_task_ui.h` / `app_task_ui.c`

用途：

- 界面任务

建议职责：

- 刷新运行状态到串口屏
- 显示模式、速度、姿态、故障码

推荐周期：

- `100 ms`

### `app_task_guard.h` / `app_task_guard.c`

用途：

- 安全监控任务

建议职责：

- 执行故障检测
- 触发告警
- 必要时强制停车

推荐周期：

- `20 ms`

主要调用：

- `FaultDetect_Run()`
- `Alarm_ReportFault(...)`
- `Chassis_Stop()`

---

## 6. RTOS 对象建议

除了任务，建议使用以下 RTOS 对象：

### 6.1 Event Flags

用途：

- 轻量同步事件

建议事件位：

- `EVT_IMU_DRDY`
- `EVT_CAMERA_FRAME`
- `EVT_FAULT_CHANGED`

适合场景：

- IMU 数据就绪中断唤醒传感任务
- 摄像头收到一帧完整数据时通知通信或视觉更新

### 6.2 Message Queue

用途：

- 传递离散消息

建议队列：

- `vision_msg_queue`
- `ui_msg_queue`
- `alarm_msg_queue`

适合场景：

- 屏幕显示请求
- 摄像头帧解析结果传递
- 告警事件上报

### 6.3 Mutex

用途：

- 保护共享状态

建议保护对象：

- 全局系统状态结构体
- 视觉结果结构体
- 显示缓冲区

---

## 7. 中断与任务的边界

这部分非常关键。

建议原则：

- 中断只做最小动作
- 复杂逻辑全部放任务

### IMU EXTI4 中断

应该做：

- 置位 `EVT_IMU_DRDY`
- 或设置一个 `data_ready` 标志

不应该做：

- SPI 读满整包数据
- 姿态融合计算
- 控制律计算

### UART 接收中断

应该做：

- 读取字节
- 推入环形缓冲区
- 检测到完整帧后发事件

不应该做：

- 大量字符串处理
- 复杂协议解析
- 界面刷新逻辑

---

## 8. 数据流建议

推荐把整个系统看成 4 条主链路：

### 8.1 运动链

```text
Encoder -> Motor Feedback -> PID -> TB6612 -> Motor
```

### 8.2 姿态链

```text
ICM42688 Raw -> Filter -> Attitude -> Fault Detect / Control Compensation
```

### 8.3 视觉链

```text
Camera UART -> Frame Parser -> Vision Result -> Tracking Control -> Chassis Command
```

### 8.4 状态链

```text
Sensors + Faults + Commands -> App State Machine -> Target Motion / UI / Alarm
```

---

## 9. 建议先实现的最小闭环

不要一开始把所有模块都写满。建议按以下顺序推进。

### 第一阶段：运动闭环跑通

先建这些文件：

- `Bsp/tb6612.*`
- `Bsp/encoder.*`
- `Algorithm/pid.*`
- `Module/motor.*`
- `Module/chassis.*`
- `APP/app.*`
- `APP/app_task_control.*`

目标：

- 电机能转
- 编码器能读
- 速度环能闭合
- 能通过固定目标速度运行

### 第二阶段：姿态数据跑通

再建这些文件：

- `Bsp/icm42688p.*`
- `Algorithm/filter.*`
- `Algorithm/attitude.*`
- `Module/imu.*`
- `APP/app_task_sensor.*`

目标：

- 能稳定读取 IMU
- 能输出俯仰/横滚估计
- 能做倾角保护

### 第三阶段：视觉和显示跑通

再建这些文件：

- `Bsp/camera_uart.*`
- `Bsp/screen_uart.*`
- `Module/vision.*`
- `Module/display.*`
- `APP/app_task_comm.*`
- `APP/app_task_ui.*`

目标：

- 摄像头数据可接入
- 屏幕能显示运行状态
- 视觉结果可进入控制链

### 第四阶段：完善状态机与安全

最后建这些文件：

- `Algorithm/fault_detect.*`
- `Module/alarm.*`
- `APP/app_state_machine.*`
- `APP/app_task_guard.*`

目标：

- 系统具备完整工作流
- 具备异常处理和停车能力

---

## 10. 当前仓库需要同步修改的地方

这部分是当前工程最容易漏掉的点。

### 10.1 `CMakeLists.txt`

现在 `APP/`、`Algorithm/`、`Module/` 只是被加入了头文件搜索路径，还没有自动参与编译。

这意味着：

- 新建 `.h` 文件可以被包含
- 新建 `.c` 文件不会自动编译

所以新增源码后，要把它们写进根目录 `CMakeLists.txt` 的 `target_sources(...)` 中。

建议写法：

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    APP/app.c
    APP/app_state_machine.c
    APP/app_task_sensor.c
    APP/app_task_control.c
    APP/app_task_comm.c
    APP/app_task_ui.c
    APP/app_task_guard.c

    Module/module_status.c
    Module/motor.c
    Module/chassis.c
    Module/imu.c
    Module/vision.c
    Module/display.c
    Module/alarm.c

    Algorithm/pid.c
    Algorithm/filter.c
    Algorithm/attitude.c
    Algorithm/tracking_ctrl.c
    Algorithm/fault_detect.c
)
```

### 10.2 `main.c`

建议在 `USER CODE BEGIN Includes` 加：

```c
#include "app.h"
```

然后在 `USER CODE BEGIN 2` 中调用：

```c
App_Init();
```

### 10.3 `freertos.c`

建议把当前单一的 `defaultTask` 改成：

- `SensorTask`
- `ControlTask`
- `CommTask`
- `UiTask`
- `GuardTask`

`defaultTask` 可以删除，或者保留为空壳后逐步替换。

---

## 11. 推荐的任务优先级

建议初版采用下面的优先级关系：

1. `ControlTask`：最高
2. `SensorTask`
3. `GuardTask`
4. `CommTask`
5. `UiTask`

原因：

- 控制回路最敏感
- 传感更新必须及时
- 安全监控优先级高于界面
- 屏幕刷新最不敏感

注意：

- 不要把 UI 任务优先级设得太高
- 不要让串口屏刷新阻塞控制任务

---

## 12. 建议的共享数据结构组织方式

为了避免全局变量失控，建议分 3 类共享数据。

### 12.1 传感快照

由 `SensorTask` 更新，其他任务只读：

- IMU 姿态
- 编码器速度
- 视觉结果

### 12.2 控制目标

由状态机或控制任务更新：

- 目标左轮速度
- 目标右轮速度
- 目标模式

### 12.3 系统状态

由 GuardTask / StateMachine 更新：

- 当前模式
- 故障位
- 在线状态
- 启停标志

建议放到 `app_data.h` 和 `module_status.h` 中统一管理，不要零散地在每个 `.c` 里定义匿名全局变量。

---

## 13. 推荐编码约束

### 驱动层约束

- `Bsp` 层不要引用 `APP/*.h`
- `Bsp` 层不要出现状态机枚举
- `Bsp` 层函数尽量短小、同步、直接

### 服务层约束

- `Module` 层可以依赖 `Bsp` 和 `Algorithm`
- `Module` 层不直接依赖 `APP`

### 算法层约束

- `Algorithm` 层不依赖 HAL 头文件
- 算法函数只收普通数值和结构体

### APP 层约束

- `APP` 层不直接操作 GPIO 引脚
- `APP` 层尽量通过服务接口控制系统

---

## 14. 一套可直接开工的最小文件清单

如果现在就开始落地，建议先建下面这些文件：

```text
Bsp/
  tb6612.h
  tb6612.c
  encoder.h
  encoder.c

Module/
  motor.h
  motor.c
  chassis.h
  chassis.c

Algorithm/
  pid.h
  pid.c
  filter.h
  filter.c

APP/
  app.h
  app.c
  app_config.h
  app_task_control.h
  app_task_control.c
```

这个最小集合足够实现：

- PWM 输出
- 编码器测速
- 速度环控制
- FreeRTOS 控制任务

然后再扩展 IMU、camera、screen、fault、state machine。

---

## 15. 最终建议

对于当前这个仓库，最合理的落地方式不是“先写一大堆功能”，而是按下面顺序推进：

1. 先把 `Bsp + Module + Algorithm + APP` 的骨架搭出来
2. 先跑通电机闭环
3. 再接 IMU 姿态
4. 再接摄像头和屏幕
5. 最后补全状态机、异常处理和比赛流程

如果后续题目要求更明确，这个架构依然成立，通常只需要调整：

- `APP/app_state_machine.c`
- `Algorithm/tracking_ctrl.c`
- `Module/vision.c`

不需要重写驱动和 RTOS 框架。
