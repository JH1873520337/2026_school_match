# OpenMV 综合脚本说明

当前 `openmv` 目录以 `main_all_in_one.py` 作为实际使用版本。

这份脚本把摄像头初始化、补光灯控制、圆点检测、火焰检测、目标滤波、串口打包和状态显示全部合在一个文件里，便于直接放到 OpenMV 上联调。

本文档只说明 `main_all_in_one.py`，不再描述目录里的拆分版脚本。

## 当前功能

- 识别橙色圆点目标
- 识别火焰图片目标
- 两类目标同时检测、同时发送
- 开机打开 RGB 补光灯
- 在图像上显示识别状态
- 通过串口向 STM32 发送双目标固定帧

## 使用文件

- 运行文件：`openmv/main_all_in_one.py`

如果后续继续维护 OpenMV 版本，默认以这个综合脚本为准。

## 摄像头配置

脚本里的当前默认配置如下：

- 分辨率：`QQVGA`
- 像素格式：`RGB565`
- 自动增益：关闭
- 自动白平衡：关闭
- 自动曝光：默认不锁定
- 串口：`UART(3)`
- 波特率：`115200`

补光灯当前配置为打开 RGB 三色灯：

- `LED(1)`
- `LED(2)`
- `LED(3)`

这样可以避免误开红外灯导致画面发紫。

## 识别逻辑

### 橙色圆点

橙色圆点检测流程：

1. 使用 LAB 阈值找橙色 blob
2. 用最小宽高过滤小噪声
3. 用长宽比过滤明显非圆形目标
4. 用密度、elongation、roundness 做圆形约束
5. 对候选打分，选择最优目标

当前圆点检测强调“接近圆形”的几何特征，主要是为了避免把近距离火焰误认成圆点。

### 火焰图片

火焰检测流程：

1. 使用两组暖色 LAB 阈值找 blob
2. 过滤过小候选
3. 过滤靠近画面边缘的候选
4. 过滤密度、长宽比和 elongation 异常的候选
5. 如果候选已经满足圆点形状约束，则直接排除
6. 对剩余候选打分，选择最优目标

当前火焰检测额外做了两层抑制：

1. 边缘屏蔽
2. 连续确认

目的是减少白纸边界外杂点、反光、单帧闪烁误判。

## 目标滤波

脚本里用了两种跟踪器：

### 普通跟踪器

橙色圆点使用普通跟踪器：

- 同类目标连续出现时做一阶平滑
- 目标短暂丢失时，短时间继续保留上一帧结果

### 连续确认跟踪器

火焰使用连续确认跟踪器：

- 新目标需要连续检测到若干帧才确认
- 确认过程中要求中心位置不能跳变过大
- 目标确认后再交给普通平滑逻辑处理

这样可以避免火焰检测对瞬时噪声过于敏感。

## 串口发送协议

当前脚本不再只发单目标，而是每个发送周期固定发送一帧双目标数据。

总长度：`27` 字节

```text
[0]  0xAA
[1]  0x55
[2]  seq
[3]  flags
[4]  orange_quality
[5]  orange_cx low
[6]  orange_cx high
[7]  orange_cy low
[8]  orange_cy high
[9]  orange_ex low
[10] orange_ex high
[11] orange_ey low
[12] orange_ey high
[13] orange_area low
[14] orange_area high
[15] flame_quality
[16] flame_cx low
[17] flame_cx high
[18] flame_cy low
[19] flame_cy high
[20] flame_ex low
[21] flame_ex high
[22] flame_ey low
[23] flame_ey high
[24] flame_area low
[25] flame_area high
[26] crc8
```

### flags 定义

- bit0：橙色圆点有效
- bit1：橙色圆点为保留帧
- bit2：火焰有效
- bit3：火焰为保留帧

### 无目标时的发送方式

如果两个目标都不存在：

- `flags = 0x00`
- 两段目标数据都为 `0`
- 仍然发送完整 27 字节固定帧

这样 STM32 端解析最简单，不需要处理变长协议。

## 图像状态显示

左上角会显示当前双目标状态，用于联调：

- `O:1` 表示圆点有效
- `O:S` 表示圆点为保留帧
- `O:0` 表示圆点无目标
- `F:1` 表示火焰有效
- `F:S` 表示火焰为保留帧
- `F:0` 表示火焰无目标

同时会显示：

- FPS
- 目标质量
- 目标坐标

## 调参重点

如果圆点和火焰仍然互相串类，优先调这些参数：

### 圆点相关

- `ORANGE_THRESHOLDS`
- `ORANGE_ASPECT_MIN`
- `ORANGE_ASPECT_MAX`
- `ORANGE_DENSITY_MIN`
- `ORANGE_ELONGATION_MAX`
- `ORANGE_ROUNDNESS_MIN`

### 火焰相关

- `FLAME_THRESHOLDS`
- `FLAME_EDGE_MARGIN_X`
- `FLAME_EDGE_MARGIN_Y`
- `FLAME_CONFIRM_FRAMES`
- `FLAME_CONFIRM_DISTANCE`

### 串口和输出节流

- `SEND_FRAME_INTERVAL_MS`

## 当前联调建议

1. 先确认 RGB 补光灯正常打开，画面没有发紫
2. 先只观察画框和状态栏，不急着联 STM32 控制
3. 分别单独测试圆点和火焰，确认基本识别稳定
4. 再测试两目标同时存在时的双目标串口输出
5. 最后再联调 STM32 端解析和控制逻辑

## 当前状态

`main_all_in_one.py` 已经完成：

- OpenMV 端综合识别逻辑
- 双目标固定帧发送
- 圆点/火焰互斥与滤波
- RGB 补光灯控制

当前更适合进入和 STM32 的整体联调与实物测试阶段。
