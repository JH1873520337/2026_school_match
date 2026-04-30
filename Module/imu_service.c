#include "imu_service.h"

#include "icm42688p.h"
#include "attitude.h"

#include <math.h>
#include <stddef.h>

#define IMU_CALIB_SAMPLES   ((uint32_t)500U)
#define IMU_SAMPLE_HZ       ((float)1000.0f)
#define IMU_CALIB_TIMEOUT   ((uint32_t)10000U)

static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;
static float s_accel_bias_x = 0.0f;
static float s_accel_bias_y = 0.0f;
static float s_accel_bias_z = 0.0f;

static attitude_t s_attitude;
static imu_data_t s_imu_data;
static uint8_t s_calibrated = 0U;

/**
 * @brief 阻塞式采集指定次数传感器数据，计算零偏均值。
 *
 * 平台必须静止。依靠 ICM42688 INT1 数据就绪标志同步，
 * 超时则跳过当前样本，避免死等。
 *
 * @param sum_gx / sum_gy / sum_gz   陀螺三轴累加器。
 * @param sum_ax / sum_ay / sum_az   加速度三轴累加器。
 * @param count                      有效采样计数输出。
 * @return 采集到的有效样本数。
 */
static uint32_t ImuService_CollectSamples(float *sum_gx, float *sum_gy, float *sum_gz,
                                          float *sum_ax, float *sum_ay, float *sum_az,
                                          uint32_t target)
{
    icm42688_scaled_t data;
    icm42688_status_t status;
    uint32_t collected = 0U;
    uint32_t timeout;

    while (collected < target)
    {
        timeout = IMU_CALIB_TIMEOUT;
        while ((ICM42688_GetDataReadyFlag() == 0U) && (timeout > 0U))
        {
            timeout--;
        }
        ICM42688_ClearDataReadyFlag();

        status = ICM42688_ReadScaled(&data);
        if (status != ICM42688_STATUS_OK)
        {
            continue;
        }

        *sum_gx += data.gyro_x_dps;
        *sum_gy += data.gyro_y_dps;
        *sum_gz += data.gyro_z_dps;
        *sum_ax += data.accel_x_g;
        *sum_ay += data.accel_y_g;
        *sum_az += data.accel_z_g;

        collected++;
    }

    return collected;
}

void ImuService_Init(void)
{
    icm42688_status_t status;
    float sum_gyro_x = 0.0f, sum_gyro_y = 0.0f, sum_gyro_z = 0.0f;
    float sum_accel_x = 0.0f, sum_accel_y = 0.0f, sum_accel_z = 0.0f;
    uint32_t count;
    float inv_count;

    /* 初始化 ICM42688 传感器 */
    status = ICM42688_Init(NULL, NULL);
    if (status != ICM42688_STATUS_OK)
    {
        s_calibrated = 0U;
        return;
    }

    /* 上电静态标定：采集 500 次，平台须静止 */
    count = ImuService_CollectSamples(&sum_gyro_x, &sum_gyro_y, &sum_gyro_z,
                                      &sum_accel_x, &sum_accel_y, &sum_accel_z,
                                      IMU_CALIB_SAMPLES);

    if (count == 0U)
    {
        s_calibrated = 0U;
        return;
    }

    inv_count = 1.0f / (float)count;

    s_gyro_bias_x = sum_gyro_x * inv_count;
    s_gyro_bias_y = sum_gyro_y * inv_count;
    s_gyro_bias_z = sum_gyro_z * inv_count;

    /* 加速度零偏：XY 轴理想值为 0，Z 轴理想值为 1g */
    s_accel_bias_x = sum_accel_x * inv_count;
    s_accel_bias_y = sum_accel_y * inv_count;
    s_accel_bias_z = sum_accel_z * inv_count - 1.0f;

    /* 初始化姿态融合器，1kHz */
    Attitude_Init(&s_attitude, IMU_SAMPLE_HZ);

    s_imu_data.roll_rad  = 0.0f;
    s_imu_data.pitch_rad = 0.0f;
    s_imu_data.yaw_rad   = 0.0f;

    s_calibrated = 1U;
}

void ImuService_Update(void)
{
    icm42688_scaled_t data;
    float gx, gy, gz;
    float ax, ay, az;
    icm42688_status_t status;

    if (s_calibrated == 0U)
    {
        return;
    }

    status = ICM42688_ReadScaled(&data);
    if (status != ICM42688_STATUS_OK)
    {
        return;
    }

    /* 零偏校正 */
    gx = data.gyro_x_dps - s_gyro_bias_x;
    gy = data.gyro_y_dps - s_gyro_bias_y;
    gz = data.gyro_z_dps - s_gyro_bias_z;
    ax = data.accel_x_g - s_accel_bias_x;
    ay = data.accel_y_g - s_accel_bias_y;
    az = data.accel_z_g - s_accel_bias_z;

    /* 姿态融合（内部完成 gyro/accel 低通 + 互补滤波） */
    Attitude_Update(&s_attitude, gx, gy, gz, ax, ay, az);

    /* 从 attitude 读取滤波后的值填充输出 */
    s_imu_data.roll_rad   = s_attitude.roll;
    s_imu_data.pitch_rad  = s_attitude.pitch;
    s_imu_data.yaw_rad    = s_attitude.yaw;
    s_imu_data.gyro_x_dps = s_attitude.filt_gyro_x;
    s_imu_data.gyro_y_dps = s_attitude.filt_gyro_y;
    s_imu_data.gyro_z_dps = s_attitude.filt_gyro_z;
    s_imu_data.accel_x_g  = s_attitude.filt_accel_x;
    s_imu_data.accel_y_g  = s_attitude.filt_accel_y;
    s_imu_data.accel_z_g  = s_attitude.filt_accel_z;
    s_imu_data.calibrated = 1U;

    /* 加速度模长，供外部可信度判断 */
    s_imu_data.accel_norm_g = sqrtf(ax * ax + ay * ay + az * az);
}

const imu_data_t *ImuService_GetData(void)
{
    return &s_imu_data;
}

uint8_t ImuService_IsCalibrated(void)
{
    return s_calibrated;
}
