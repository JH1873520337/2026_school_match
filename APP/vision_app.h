#ifndef VISION_APP_H
#define VISION_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define VISION_APP_TARGET_NONE          ((uint8_t)0U)
#define VISION_APP_TARGET_ORANGE_CIRCLE ((uint8_t)1U)
#define VISION_APP_TARGET_FLAME         ((uint8_t)2U)

/**
 * @brief APP 层单个视觉目标。
 *
 * 该结构直接对应 OpenMV 双目标协议里单个目标的数据区：
 * quality + cx + cy + ex + ey + area。
 */
typedef struct
{
    uint8_t  valid;         /* 目标是否有效（检测到） */
    uint8_t  stale;         /* 跟踪保持标志（短暂丢失但沿用上一帧） */
    uint8_t  quality;       /* 检测/跟踪质量 0~100 */

    uint16_t cx;            /* 目标中心 X 像素坐标 */
    uint16_t cy;            /* 目标中心 Y 像素坐标 */

    int16_t  ex;            /* 归一化横向误差，大致 -1000~1000 */
    int16_t  ey;            /* 归一化纵向误差，大致 -1000~1000 */

    uint16_t area;          /* 目标像素面积 */
} vision_target_t;

/**
 * @brief APP 层视觉整帧结果。
 *
 * 当前 OpenMV 固件每帧同时发送橙色圆与火焰两个目标：
 * - orange 对应 flags bit0/bit1
 * - flame  对应 flags bit2/bit3
 */
typedef struct
{
    uint8_t online;         /* 视觉串口最近是否在线 */
    uint8_t sequence;       /* OpenMV 发包序号 */
    uint8_t crc_ok;         /* 最近一次发布到 APP 的帧校验是否通过 */
    uint32_t frame_count;   /* APP 层累计接收的有效帧数 */

    vision_target_t orange; /* 橙色圆目标 */
    vision_target_t flame;  /* 火焰目标 */
} vision_frame_t;

/**
 * @brief 初始化视觉 APP。
 *
 * 该函数只复位 APP/Service 层状态，不负责启动 UART4 DMA。
 * 典型调用顺序：
 * 1. CameraUart_Init()
 * 2. VisionApp_Init()
 * 3. 周期调用 VisionApp_Process()
 */
void VisionApp_Init(void);

/**
 * @brief 视觉 APP 周期处理。
 *
 * 从 CameraUart 环形缓冲区读取字节，交给 VisionService 完成协议解析，
 * 再把最新双目标结果发布到 APP 层缓存。
 * 建议在主循环或任务中以 100~200Hz 调用。
 */
void VisionApp_Process(void);

/**
 * @brief 获取最近一次发布到 APP 层的整帧视觉数据。
 * @return 只读指针，内容由 VisionApp_Process 更新。
 */
const vision_frame_t *VisionApp_GetFrame(void);

/**
 * @brief 获取最近一次发布的橙色圆目标。
 * @return 只读指针，等价于 VisionApp_GetFrame()->orange。
 */
const vision_target_t *VisionApp_GetOrangeTarget(void);

/**
 * @brief 获取最近一次发布的火焰目标。
 * @return 只读指针，等价于 VisionApp_GetFrame()->flame。
 */
const vision_target_t *VisionApp_GetFlameTarget(void);

/**
 * @brief 查询并清除新帧就绪标志。
 *
 * 当 VisionService 成功解析出一帧新序号数据后，该标志会被置位。
 *
 * @return 1 表示有新帧，0 表示无。
 */
uint8_t VisionApp_IsNewFrame(void);

#ifdef __cplusplus
}
#endif

#endif
