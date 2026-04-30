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
 * @brief OpenMV 视觉目标解析结果。
 *
 * 对应 OpenMV 端 VisionTarget 的全部字段，由 VisionApp_Process 更新。
 */
typedef struct
{
    uint8_t  target_kind;   /* 目标类型：0=NONE 1=橙色圆 2=火焰 */
    uint8_t  valid;         /* 目标是否有效（检测到） */
    uint8_t  stale;         /* 跟踪保持标志（超时未检测到但仍在保持） */
    uint8_t  quality;       /* 检测/跟踪质量 0~100 */

    uint16_t cx;            /* 目标中心 X 像素坐标 */
    uint16_t cy;            /* 目标中心 Y 像素坐标 */

    int16_t  ex;            /* 归一化横向误差 -1000~1000 */
    int16_t  ey;            /* 归一化纵向误差 -1000~1000 */

    uint16_t area;          /* 目标像素面积 */
    int16_t  angle;         /* 预留角度字段 */

    uint8_t  seq;           /* 帧序列号 */
    uint8_t  crc_ok;        /* CRC8 校验通过标志 */
    uint32_t frame_count;   /* 累计接收的有效帧数 */
} vision_target_t;

/**
 * @brief 初始化视觉 APP。
 *
 * 使能 UART4 NVIC 中断，启动单字节循环接收。
 * 调用后中断会逐字节填充环形缓冲区，无需在 ISR 中手动重启。
 */
void VisionApp_Init(void);

/**
 * @brief 视觉 APP 周期处理。
 *
 * 从环形缓冲区消费字节，通过状态机解析 OpenMV 帧协议。
 * 建议 100~200Hz 调用，例如在主循环或 VisionTask 中。
 */
void VisionApp_Process(void);

/**
 * @brief 获取最近一次成功解析的目标数据。
 * @return 只读指针，内容在每次 VisionApp_Process 有效帧时更新。
 */
const vision_target_t *VisionApp_GetTarget(void);

/**
 * @brief 查询并清除新帧就绪标志。
 *
 * 调用后标志归零，适合轮询模式：
 *   if (VisionApp_IsNewFrame()) { handle... }
 *
 * @return 1 表示有新帧，0 表示无。
 */
uint8_t VisionApp_IsNewFrame(void);

#ifdef __cplusplus
}
#endif

#endif
