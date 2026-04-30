#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief IMU 服务层输出数据结构。
 *
 * 由 ImuService_Update 周期更新，包含融合后的姿态角和滤波后的传感器值。
 */
typedef struct
{
    float roll_rad;      /* 融合后的横滚角 (rad) */
    float pitch_rad;     /* 融合后的俯仰角 (rad) */
    float yaw_rad;       /* 陀螺积分偏航角 (rad)，无磁力计修正，会漂移 */

    float gyro_x_dps;    /* 零偏校正+低通滤波后的陀螺 X 角速度 (dps) */
    float gyro_y_dps;    /* 零偏校正+低通滤波后的陀螺 Y 角速度 (dps) */
    float gyro_z_dps;    /* 零偏校正+低通滤波后的陀螺 Z 角速度 (dps) */

    float accel_x_g;     /* 零偏校正+低通滤波后的加速度 X (g) */
    float accel_y_g;     /* 零偏校正+低通滤波后的加速度 Y (g) */
    float accel_z_g;     /* 零偏校正+低通滤波后的加速度 Z (g) */
    float accel_norm_g;  /* 加速度模长 |a| (g)，用于可信度判断 */

    uint8_t calibrated;  /* 标定完成标志：1=已标定并可用 */
} imu_data_t;

/**
 * @brief 上电标定并初始化 IMU 服务。
 *
 * 平台必须静止。流程：
 * 1. 初始化 ICM42688 传感器
 * 2. 阻塞式采样 500 次计算陀螺/加速度计零偏（约 500ms）
 * 3. 初始化姿态融合器（互补滤波 @1kHz）
 *
 * 调用期间不得移动平台，建议在 RTOS 任务创建前调用。
 */
void ImuService_Init(void);

/**
 * @brief 周期更新 IMU 服务。
 *
 * 流程：
 * 1. 读取 ICM42688 传感器换算值
 * 2. 零偏校正 → 姿态融合 → 填充 imu_data_t
 *
 * 调用频率应与 attitude 配置的 sample_hz 一致（当前 1kHz）。
 * 建议在 RTOS SensorTask 中以 1ms 周期驱动。
 */
void ImuService_Update(void);

/**
 * @brief 获取最新 IMU 数据。
 * @return 只读数据指针，内容由 ImuService_Update 更新。
 */
const imu_data_t *ImuService_GetData(void);

/**
 * @brief 查询标定是否完成。
 * @return 1 已标定且数据可信，0 未标定或标定失败。
 */
uint8_t ImuService_IsCalibrated(void);

#ifdef __cplusplus
}
#endif

#endif
