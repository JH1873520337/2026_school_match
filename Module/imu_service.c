/**
 * @file imu_service.c
 * @brief IMU 服务层实现。
 *
 * 职责：
 * - 封装 ICM42688 传感器访问
 * - 上电静态标定（陀螺 + 加速度计零偏）
 * - 组合 Algorithm/attitude 完成姿态融合
 * - 对外输出 imu_data_t
 *
 * 依赖：Bsp/icm42688p、Algorithm/attitude
 * 被依赖：APP/imu_app
 */

#include "imu_service.h"

#include "icm42688p.h"
#include "attitude.h"

#include <math.h>
#include <stddef.h>

/* 标定参数 */
#define IMU_CALIB_SAMPLES   ((uint32_t)500U)    /* 上电标定采样次数 */
#define IMU_SAMPLE_HZ       ((float)1000.0f)    /* 姿态融合器调用频率 */
#define IMU_CALIB_TIMEOUT   ((uint32_t)10000U)  /* 等待数据就绪的超时计数 */

/* 零偏补偿值（标定时计算，运行时使用） */
static float s_gyro_bias_x = 0.0f;   /* 陀螺 X 零偏 (dps) */
static float s_gyro_bias_y = 0.0f;   /* 陀螺 Y 零偏 (dps) */
static float s_gyro_bias_z = 0.0f;   /* 陀螺 Z 零偏 (dps) */
static float s_accel_bias_x = 0.0f;  /* 加速度 X 零偏 (g) */
static float s_accel_bias_y = 0.0f;  /* 加速度 Y 零偏 (g) */
static float s_accel_bias_z = 0.0f;  /* 加速度 Z 零偏 (g)，校正后静置 Z 接近 1.0g */

/* 姿态融合器句柄 */
static attitude_t s_attitude;

/* 服务层输出数据 */
static imu_data_t s_imu_data;

/* 标定完成标志 */
static uint8_t s_calibrated = 0U;

/**
 * @brief 阻塞式采集指定次数传感器数据，计算零偏均值。
 *
 * 平台必须静止。依靠 ICM42688 INT1 数据就绪标志同步，
 * 超时则跳过当前样本，避免死等。
 *
 * @param sum_gx / sum_gy / sum_gz   陀螺三轴累加器 (dps)。
 * @param sum_ax / sum_ay / sum_az   加速度三轴累加器 (g)。
 * @param target                     期望采集的样本次数。
 * @return 实际采集到的有效样本数。
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
        /* 等待 INT1 数据就绪标志 */
        timeout = IMU_CALIB_TIMEOUT;
        while ((ICM42688_GetDataReadyFlag() == 0U) && (timeout > 0U))
        {
            timeout--;
        }
        ICM42688_ClearDataReadyFlag();

        /* 读取传感器换算值 */
        status = ICM42688_ReadScaled(&data);
        if (status != ICM42688_STATUS_OK)
        {
            continue;   /* 读取失败则跳过本次，不计入有效样本 */
        }

        /* 累加各轴原始值 */
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

    /* 步骤 1：初始化 ICM42688 传感器（SPI + 寄存器配置 + WHO_AM_I 校验） */
    status = ICM42688_Init(NULL, NULL);
    if (status != ICM42688_STATUS_OK)
    {
        s_calibrated = 0U;
        return;
    }

    /* 步骤 2：上电静态标定，采集 500 次原始数据计算零偏 */
    count = ImuService_CollectSamples(&sum_gyro_x, &sum_gyro_y, &sum_gyro_z,
                                      &sum_accel_x, &sum_accel_y, &sum_accel_z,
                                      IMU_CALIB_SAMPLES);

    if (count == 0U)
    {
        s_calibrated = 0U;
        return;
    }

    inv_count = 1.0f / (float)count;

    /* 步骤 3：计算陀螺各轴零偏 */
    s_gyro_bias_x = sum_gyro_x * inv_count;
    s_gyro_bias_y = sum_gyro_y * inv_count;
    s_gyro_bias_z = sum_gyro_z * inv_count;

    /* 步骤 4：计算加速度计零偏
       - X/Y 轴理想静态值为 0，直接取均值
       - Z 轴理想静态值为 1g，实际均值减去 1.0 得到零偏 */
    s_accel_bias_x = sum_accel_x * inv_count;
    s_accel_bias_y = sum_accel_y * inv_count;
    s_accel_bias_z = sum_accel_z * inv_count - 1.0f;

    /* 步骤 5：初始化姿态融合器（gyro LPF 400Hz, accel LPF 10Hz, 互补 α=0.998） */
    Attitude_Init(&s_attitude, IMU_SAMPLE_HZ);

    /* 步骤 6：清零输出结构体初始值 */
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

    /* 标定未完成则直接返回 */
    if (s_calibrated == 0U)
    {
        return;
    }

    /* 步骤 1：读取 ICM42688 传感器换算值 */
    status = ICM42688_ReadScaled(&data);
    if (status != ICM42688_STATUS_OK)
    {
        return;
    }

    /* 步骤 2：零偏校正 */
    gx = data.gyro_x_dps - s_gyro_bias_x;
    gy = data.gyro_y_dps - s_gyro_bias_y;
    gz = data.gyro_z_dps - s_gyro_bias_z;
    ax = data.accel_x_g - s_accel_bias_x;
    ay = data.accel_y_g - s_accel_bias_y;
    az = data.accel_z_g - s_accel_bias_z;

    /* 步骤 3：姿态融合
       内部完成 gyro LPF / accel LPF / 互补滤波 / 加速度可信度加权 */
    Attitude_Update(&s_attitude, gx, gy, gz, ax, ay, az);

    /* 步骤 4：从 attitude 读取滤波后的值填充输出结构体 */
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

    /* 步骤 5：计算加速度模长，供外部可信度判断（如倾斜保护） */
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
