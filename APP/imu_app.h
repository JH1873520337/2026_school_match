#ifndef IMU_APP_H
#define IMU_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_service.h"

#include <stdint.h>

/**
 * @brief IMU APP 初始化。
 *
 * 对 ImuService_Init 的薄封装：
 * - 初始化 ICM42688 传感器（SPI + 寄存器）
 * - 上电静态标定 500 次，计算零偏
 * - 初始化姿态融合器（互补滤波 @1kHz）
 *
 * 调用期间平台须静止，阻塞约 500ms。
 * 应在 RTOS 任务创建之前或首个任务初始化阶段调用。
 */
void ImuApp_Init(void);

/**
 * @brief IMU APP 周期处理。
 *
 * 对 ImuService_Update 的薄封装：
 * - 读取 ICM42688 传感器数据
 * - 零偏校正 + 低通滤波 + 互补滤波姿态融合
 * - 填充 imu_data_t 输出
 *
 * 调用频率应与 attitude 配置的 sample_hz 一致（当前 1kHz），
 * 建议在 RTOS SensorTask 中以 osDelay(1) 驱动。
 */
void ImuApp_Process(void);

/**
 * @brief 获取最新的 IMU 融合数据。
 * @return 只读 imu_data_t 指针，含姿态角、滤波后陀螺/加速度值。
 */
const imu_data_t *ImuApp_GetData(void);

/**
 * @brief 查询 IMU 标定是否完成。
 * @return 1 已标定且数据可用，0 未标定或标定失败。
 */
uint8_t ImuApp_IsCalibrated(void);

#ifdef __cplusplus
}
#endif

#endif
