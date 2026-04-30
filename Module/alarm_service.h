#ifndef ALARM_SERVICE_H
#define ALARM_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 初始化告警服务。
 *
 * 该函数会初始化蜂鸣器底层，并清空内部播放状态。
 */
void AlarmService_Init(void);

/**
 * @brief 周期处理告警服务。
 *
 * 采用非阻塞状态机驱动蜂鸣器节拍，建议在主循环或 RTOS 任务中周期调用。
 */
void AlarmService_Process(void);

/**
 * @brief 播放“火源检测到”三长鸣。
 *
 * 若当前已有蜂鸣器节拍正在播放，本函数会从头重新开始播放。
 */
void AlarmService_PlayFireDetectedPattern(void);

/**
 * @brief 立即停止蜂鸣器并清除当前节拍。
 */
void AlarmService_Stop(void);

/**
 * @brief 查询当前是否仍在播放节拍。
 * @return 1 表示忙，0 表示空闲。
 */
uint8_t AlarmService_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif
