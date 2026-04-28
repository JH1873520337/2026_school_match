# `scool_match` 架构与后续开发计划

## 1. 文档目的

这份文档用于固定当前工程的分层边界、调用方向和后续开发顺序，避免随着 PID、IMU 滤波、视觉、串口屏和 RTOS 任务逐步加入后，代码职责混乱。

这不是抽象模板，而是基于当前仓库实际结构给出的落地计划。

当前已知条件：

- 芯片：`STM32F407VET6`
- 工程基础：`STM32CubeMX + HAL + FreeRTOS(CMSIS-RTOS2) + CMake`
- 当前目录：`Core/`、`Bsp/`、`Module/`、`Algorithm/`、`APP/`
- 当前外设：
  - `SPI1` -> `ICM42688P`
  - `TIM1/TIM2/TIM4/TIM5` -> 编码器
  - `TIM9/TIM12` -> TB6612 PWM
  - `UART4` -> OpenMV camera
  - `UART5` -> 串口屏
  - `PB2` -> 蜂鸣器
- OpenMV 当前综合脚本已经固定为 `openmv/main_all_in_one.py`

本计划的目标是：

1. 明确每层应该写什么
2. 明确每层不应该写什么
3. 明确后续文件该放在哪
4. 明确 APP 与 RTOS 如何隔离
5. 明确后续功能的推荐实现顺序

---

## 2. 总体分层

建议整个工程按 5 层理解：

1. `Core/`
2. `Bsp/`
3. `Algorithm/`
4. `Module/`
5. `APP/`

其中：

- `Core`：CubeMX 和 RTOS 基础框架
- `Bsp`：具体硬件驱动
- `Algorithm`：纯算法
- `Module`：服务层/功能模块层
- `APP`：业务流程层

一句话概括：

- `Bsp` 解决“怎么读写硬件”
- `Algorithm` 解决“怎么计算”
- `Module` 解决“把硬件和算法组合成什么能力”
- `APP` 解决“系统当前该做什么”
- `RTOS` 只解决“什么时候调用 APP”

---

## 3. 依赖方向

后续代码尽量只允许下面这种单向依赖：

```text
Core/CubeMX -> Bsp -> Module -> APP
Algorithm -> Module -> APP
RTOS -> APP
```

更具体一点：

- `Bsp` 可以依赖 HAL、CubeMX 生成的外设句柄
- `Algorithm` 不应该依赖 HAL、FreeRTOS、Bsp
- `Module` 可以依赖 `Bsp` 和 `Algorithm`
- `APP` 可以依赖 `Module`，必要时依赖少量算法公共类型
- `freertos.c` 或 RTOS 包装文件只依赖 `APP`

不建议出现这些反向依赖：

- `Bsp -> Module`
- `Bsp -> APP`
- `Algorithm -> Bsp`
- `Algorithm -> FreeRTOS`
- `Module -> RTOS`
- `Core/freertos.c -> 直接业务实现`

---

## 4. 各层职责定义

### 4.1 `Core/`

职责：

- 时钟初始化
- 外设初始化
- NVIC / DMA / 外设中断入口
- FreeRTOS 基础对象和任务壳
- 少量桥接回调

应该放的内容：

- `main.c`
- `freertos.c`
- `stm32f4xx_it.c`
- `usart.c` / `dma.c` / `spi.c` / `tim.c`

不应该放的内容：

- PID 计算
- IMU 姿态融合
- 摄像头协议解析
- 串口屏页面业务
- 巡检状态机
- 火焰处理逻辑

建议：

- `main.c` 只做启动和硬件 bring-up
- `freertos.c` 只做任务创建和任务壳
- 中断里只做最小动作和回调转发

### 4.2 `Bsp/`

职责：

- 面向具体硬件设备
- 直接使用 HAL
- 提供稳定、简单、无业务语义的接口

这一层应当只关心：

- GPIO 怎么拉高拉低
- PWM 怎么输出
- UART 怎么收发
- SPI 怎么读写
- 编码器怎么取值
- DMA 怎么搬数据

这一层不应该关心：

- 当前是巡检模式还是火焰模式
- 当前应该停车还是转向
- 某个视觉结果是否代表业务事件

### 4.3 `Algorithm/`

职责：

- 只放纯算法
- 输入和输出都使用普通数值或结构体
- 不直接依赖任何外设句柄

这一层后续主要放：

- PID
- 一阶低通 / 滑动平均 / 斜坡限制
- IMU 滤波
- 姿态解算
- 视觉跟踪控制律
- 故障判据

### 4.4 `Module/`

职责：

- 把一个或多个 BSP 驱动组织成稳定功能
- 在需要的时候调用算法层
- 对 APP 暴露更接近业务的服务接口

这一层应当屏蔽：

- 硬件接线细节
- DMA / 中断细节
- 寄存器层细节

APP 应该通过 `Module` 获得：

- 视觉结果
- IMU 姿态
- 电机反馈
- 底盘/绳驱控制能力
- 屏幕显示能力
- 告警能力

### 4.5 `APP/`

职责：

- 组织业务流程
- 维护状态机
- 组合多个模块实现比赛逻辑

这一层不应该直接操作：

- GPIO
- DMA
- 原始 UART 字节流
- 原始寄存器数据

APP 应该只表达：

- 当前处于什么模式
- 什么时候进入巡检
- 什么时候响应火焰
- 什么时候刷新 UI
- 什么时候停车、报警、恢复

---

## 5. APP 与 RTOS 的隔离原则

这是后续最关键的约束之一。

### 5.1 正确关系

RTOS 不是业务层，RTOS 只是调度器。

正确的关系应该是：

```text
RTOS task shell -> APP process function -> Module -> Bsp / Algorithm
```

### 5.2 建议写法

APP 层尽量写成不依赖 RTOS 的业务函数，例如：

```c
void VisionApp_Init(void);
void VisionApp_Process(void);

void PatrolApp_Init(void);
void PatrolApp_Process(void);

void UiApp_Init(void);
void UiApp_Process(void);
```

然后 RTOS 任务只做调度壳：

```c
for (;;)
{
    VisionApp_Process();
    osDelay(5);
}
```

### 5.3 不建议写法

不要把下面这些直接堆进 `freertos.c`：

- 视觉协议状态机
- PID 运算
- IMU 姿态融合
- 串口屏页面逻辑
- 比赛流程状态机

也不要把 APP 核心逻辑写成“只能在 RTOS 里运行”的形式。

这样后续即使临时切回裸机 super loop，也还能复用 APP 层。

---

## 6. 当前与后续推荐文件归属

### 6.1 摄像头链路

当前建议与现状一致：

- `Bsp/camera_uart.c/.h`
  - `UART4 + DMA + 环形缓冲`
  - 只管原始字节收发
- `Module/vision_service.c/.h`
  - OpenMV 双目标固定帧协议解析
  - 在线状态、目标有效位、结果缓存
- `APP/vision_app.c/.h`
  - 后续补充
  - 负责决定如何消费视觉结果

注意：

- `camera_uart` 不解析火焰和圆点语义
- `vision_service` 不做业务状态机
- `APP` 不碰原始字节流

### 6.2 IMU 链路

建议分三层：

- `Bsp/icm42688p.c/.h`
  - 寄存器读写
  - 初始化
  - 原始加速度/角速度读取
  - 中断标志
- `Algorithm/filter.c/.h`
  - 低通、平滑
- `Algorithm/attitude.c/.h`
  - 姿态融合
- `Module/imu_service.c/.h`
  - 标定
  - 原始值转物理单位
  - 调用滤波和姿态融合
  - 输出稳定姿态

### 6.3 PID 与运动控制链路

建议分三层：

- `Bsp/encoder.c/.h`
  - 原始计数读写
- `Bsp/TB6612.c/.h`
  - PWM 与方向控制
- `Algorithm/pid.c/.h`
  - PID 核心算法
- `Algorithm/filter.c/.h`
  - 速度滤波、目标斜坡限制
- `Module/motor_service.c/.h`
  - 单电机目标速度、反馈、闭环
- `Module/chassis_service.c/.h`
  - 多电机协同控制

### 6.4 串口屏链路

建议分两层：

- `Bsp/screen_uart.c/.h`
  - 只做 `UART5` 原始收发
- `Module/screen_service.c/.h`
  - 页面切换
  - 文本更新
  - 数值更新
  - 如有触摸反馈，也放在这一层解析
- `APP/ui_app.c/.h`
  - 决定当前显示什么

### 6.5 蜂鸣器与告警链路

建议分两层：

- `Bsp/buzzer.c/.h`
  - GPIO 开关
- `Module/alarm_service.c/.h`
  - 根据故障码输出蜂鸣策略

---

## 7. 推荐的目录和文件命名

为了后续保持统一，建议后续优先使用下面这种命名风格：

### 7.1 `Bsp/`

- `camera_uart.c/h`
- `screen_uart.c/h`
- `icm42688p.c/h`
- `encoder.c/h`
- `tb6612.c/h`
- `buzzer.c/h`

### 7.2 `Algorithm/`

- `pid.c/h`
- `filter.c/h`
- `attitude.c/h`
- `tracking_ctrl.c/h`
- `fault_detect.c/h`

### 7.3 `Module/`

- `vision_service.c/h`
- `imu_service.c/h`
- `motor_service.c/h`
- `chassis_service.c/h`
- `screen_service.c/h`
- `alarm_service.c/h`
- `module_types.h`
- `module_status.c/h`

### 7.4 `APP/`

- `app_main.c/h`
- `vision_app.c/h`
- `patrol_app.c/h`
- `fire_app.c/h`
- `ui_app.c/h`
- `app_state_machine.c/h`
- `app_tasks_rtos.c/h`
- `app_config.h`
- `app_data.h`

---

## 8. 中断、DMA 与任务的边界

### 8.1 原则

- 中断只做最小动作
- DMA 只做搬运
- 复杂逻辑一律放任务或 APP 周期处理

### 8.2 IMU 中断

应该做：

- 置位数据就绪标志
- 或发轻量事件

不应该做：

- SPI 读整包数据
- 姿态融合
- 故障判断

### 8.3 UART / DMA 回调

应该做：

- 把收到的字节或块数据写入缓冲
- 必要时重启 DMA 接收

不应该做：

- 业务状态机切换
- 屏幕刷新
- PID 计算
- 大量字符串处理

### 8.4 任务

任务里适合做：

- 模块更新
- 协议解析
- 状态机推进
- PID 计算
- UI 周期刷新

---

## 9. 建议的数据流

### 9.1 运动链

```text
Encoder -> MotorService feedback -> PID -> TB6612 -> Motor
```

### 9.2 姿态链

```text
ICM42688 raw -> Filter -> Attitude -> ImuService -> APP / FaultDetect
```

### 9.3 视觉链

```text
OpenMV UART -> camera_uart -> vision_service -> vision_app / tracking_ctrl
```

### 9.4 UI 链

```text
APP state/data -> screen_service -> screen_uart -> Serial Screen
```

### 9.5 告警链

```text
FaultDetect / APP state -> alarm_service -> buzzer
```

---

## 10. 推荐开发顺序

不要一开始把所有模块同时铺开，建议按闭环能力逐步推进。

### 第一阶段：运动闭环先跑通

优先完成：

- `Bsp/tb6612.*`
- `Bsp/encoder.*`
- `Algorithm/pid.*`
- `Algorithm/filter.*`
- `Module/motor_service.*`
- `Module/chassis_service.*`

目标：

- 电机输出正常
- 编码器反馈正常
- 速度闭环能稳定运行

### 第二阶段：IMU 链路跑通

优先完成：

- `Bsp/icm42688p.*`
- `Algorithm/attitude.*`
- `Module/imu_service.*`

目标：

- 原始数据稳定
- 姿态或角速度可用
- 能做基本姿态保护

### 第三阶段：视觉与串口屏跑通

优先完成：

- `Bsp/camera_uart.*`
- `Module/vision_service.*`
- `Bsp/screen_uart.*`
- `Module/screen_service.*`

目标：

- 摄像头数据稳定接入
- 串口屏可显示关键状态
- 视觉数据能提供给 APP

### 第四阶段：APP 状态机与安全策略

优先完成：

- `APP/app_state_machine.*`
- `APP/patrol_app.*`
- `APP/fire_app.*`
- `Algorithm/fault_detect.*`
- `Module/alarm_service.*`

目标：

- 巡检、目标响应、停机、告警流程成型

### 第五阶段：RTOS 任务化

最后再把 APP 层逻辑挂到 RTOS 上。

推荐任务类型：

- `SensorTask`
- `ControlTask`
- `CommTask`
- `UiTask`
- `GuardTask`

推荐优先级关系：

1. `ControlTask`
2. `SensorTask`
3. `GuardTask`
4. `CommTask`
5. `UiTask`

---

## 11. 当前工程需要保持的实现约束

### 11.1 `main.c`

应保持：

- 外设初始化
- 底层桥接初始化
- 中断回调转发

不应继续堆积：

- 业务状态机
- 模块调度
- UI 逻辑

### 11.2 `freertos.c`

当前还没有正式配置任务。

建议保持为空壳，直到 APP 层任务划分明确后，再把：

- `VisionApp_Process()`
- `PatrolApp_Process()`
- `UiApp_Process()`

这类函数挂进去。

### 11.3 `CMakeLists.txt`

当前根 CMake 仍然需要手工把新增 `.c` 文件加入编译列表。

这意味着：

- 新增头文件后可以 include
- 新增源文件后必须显式加入 `CMakeLists.txt`

后续新增 `APP/`、`Module/`、`Algorithm/` 文件时，要同步维护构建列表。

---

## 12. 需要避免的常见错误

后续最容易写乱的方式有这些：

1. 把 PID 写进 `TB6612.c`
2. 把 IMU 滤波写进 `icm42688p.c`
3. 把火焰/圆点协议解析写进 `camera_uart.c`
4. 把串口屏页面业务写进 `screen_uart.c`
5. 把比赛状态机写进 `freertos.c`
6. 把大量业务逻辑堆进 `main.c`
7. 在 `Module` 里直接使用 `osDelay`、队列、互斥锁作为核心接口
8. 在 `Algorithm` 里包含 HAL 头文件

如果后面出现这些情况，就说明分层开始失控，需要及时收回。

---

## 13. 近期落地建议

基于当前仓库状态，建议最近几步按下面推进：

1. 保持 `camera_uart -> vision_service` 这条链稳定
2. 先把 `motor_service` 和 `chassis_service` 骨架补齐
3. 开始补 `pid` 和编码器速度闭环
4. 再把 `imu_service` 和姿态滤波接进来
5. 最后再正式设计 `APP` 状态机和 RTOS 任务映射

如果后续比赛逻辑调整，优先改：

- `APP/*`
- 少量 `Module/*`

不要轻易回头改底层 `Bsp` 和 `Core`，除非硬件接口或协议本身变化。

---

## 14. 最终结论

对于当前工程，后续最合理的推进方式不是“先把所有功能堆出来”，而是按分层边界逐步闭环：

1. 先稳住底层外设和模块服务
2. 再补纯算法
3. 再组织 APP 业务流程
4. 最后把 APP 映射到 RTOS 任务

只要坚持下面这条主线，后续功能增加时就不容易乱：

```text
硬件驱动放 Bsp
算法放 Algorithm
功能服务放 Module
业务流程放 APP
RTOS 只负责调度 APP
```
