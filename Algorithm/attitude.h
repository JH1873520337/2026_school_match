#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 姿态融合器配置。
 */
typedef struct
{
    float gyro_lpf_hz;       /* 陀螺仪低通截止频率 (Hz) */
    float accel_lpf_hz;      /* 加速度计低通截止频率 (Hz) */
    float sample_hz;         /* 融合器调用频率 (Hz) */
    float compl_alpha;       /* 互补滤波系数 (0~1)，越大越信陀螺，典型 0.98~0.995 */
    float accel_trust_thr;   /* 加速度可信阈值 (g)，|a|-1.0 超过此值则降低 accel 权重 */
} attitude_config_t;

/**
 * @brief 姿态融合器句柄。
 */
typedef struct
{
    attitude_config_t config;

    /* 当前姿态，单位 rad */
    float roll;
    float pitch;
    float yaw;

    /* 内部低通滤波器状态 */
    float filt_gyro_x;
    float filt_gyro_y;
    float filt_gyro_z;
    float filt_accel_x;
    float filt_accel_y;
    float filt_accel_z;

    /* 低通 alpha 值 */
    float gyro_lpf_alpha;
    float accel_lpf_alpha;

    uint8_t initialized;
} attitude_t;

/**
 * @brief 用默认参数初始化姿态融合器。
 *
 * 默认参数适用于绳驱 X-Y 平面平台：
 * - gyro 低通 400Hz
 * - accel 低通 10Hz
 * - 互补系数 0.998
 * - 加速度可信阈值 0.3g
 *
 * @param att 姿态融合器句柄指针。
 * @param sample_hz 融合器调用频率，单位 Hz。
 */
void Attitude_Init(attitude_t *att, float sample_hz);

/**
 * @brief 用自定义参数初始化姿态融合器。
 * @param att 姿态融合器句柄指针。
 * @param config 自定义配置指针。
 */
void Attitude_InitConfig(attitude_t *att, const attitude_config_t *config);

/**
 * @brief 单步更新姿态融合器。
 *
 * 输入原始 gyro (dps) 和 accel (g)，输出融合后的 roll/pitch/yaw (rad)。
 * yaw 仅靠陀螺积分，无磁力计修正，会随时间漂移。
 *
 * @param att 姿态融合器句柄指针。
 * @param gyro_x 陀螺 X 轴角速度，单位 dps。
 * @param gyro_y 陀螺 Y 轴角速度，单位 dps。
 * @param gyro_z 陀螺 Z 轴角速度，单位 dps。
 * @param accel_x 加速度 X 轴，单位 g。
 * @param accel_y 加速度 Y 轴，单位 g。
 * @param accel_z 加速度 Z 轴，单位 g。
 */
void Attitude_Update(attitude_t *att,
                     float gyro_x, float gyro_y, float gyro_z,
                     float accel_x, float accel_y, float accel_z);

/**
 * @brief 重置融合器姿态到零位。
 * @param att 姿态融合器句柄指针。
 */
void Attitude_Reset(attitude_t *att);

#ifdef __cplusplus
}
#endif

#endif
