#ifndef IMU_SERVICE_H
#define IMU_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief IMU 服务层输出数据结构。
 */
typedef struct
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;

    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;

    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float accel_norm_g;

    uint8_t calibrated;
} imu_data_t;

/**
 * @brief 上电标定并初始化 IMU 服务。
 *
 * 平台必须静止。阻塞式采样 500 次计算零偏，然后初始化姿态融合器。
 * 耗时约 500ms（1kHz ODR）。
 */
void ImuService_Init(void);

/**
 * @brief 周期更新 IMU 服务。
 *
 * 读取传感器 → 零偏校正 → 姿态融合 → 填充输出数据。
 * 调用频率应与 attitude 配置的 sample_hz 一致（1kHz）。
 */
void ImuService_Update(void);

/**
 * @brief 获取最新 IMU 数据。
 * @return 只读数据指针，由 ImuService_Update 更新。
 */
const imu_data_t *ImuService_GetData(void);

/**
 * @brief 查询标定是否完成。
 * @return 1 已标定，0 未标定。
 */
uint8_t ImuService_IsCalibrated(void);

#ifdef __cplusplus
}
#endif

#endif
