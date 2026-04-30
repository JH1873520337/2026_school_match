#include "attitude.h"

#include <math.h>
#include <stddef.h>

#define ATTITUDE_PI       (3.14159265358979323846f)
#define ATTITUDE_D2R      (ATTITUDE_PI / 180.0f)
#define ATTITUDE_R2D      (180.0f / ATTITUDE_PI)
#define ATTITUDE_GRAVITY  (1.0f)

/* 默认配置，面向绳驱 X-Y 平面平台 */
#define DEFAULT_GYRO_LPF_HZ      (400.0f)
#define DEFAULT_ACCEL_LPF_HZ     (10.0f)
#define DEFAULT_COMPL_ALPHA      (0.998f)
#define DEFAULT_ACCEL_TRUST_THR  (0.3f)

static void Attitude_CalcLpfAlpha(float *alpha, float cutoff_hz, float sample_hz)
{
    float dt;
    float tau;

    if (alpha == NULL)
    {
        return;
    }

    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f)
    {
        *alpha = 1.0f;
        return;
    }

    dt = 1.0f / sample_hz;
    tau = 1.0f / (2.0f * ATTITUDE_PI * cutoff_hz);

    if (tau <= 0.0f)
    {
        *alpha = 1.0f;
    }
    else
    {
        *alpha = dt / (dt + tau);

        if (*alpha > 1.0f)
        {
            *alpha = 1.0f;
        }
        if (*alpha < 0.0f)
        {
            *alpha = 0.0f;
        }
    }
}

static float Attitude_LpfUpdate(float alpha, float prev, float input)
{
    return alpha * input + (1.0f - alpha) * prev;
}

/**
 * @brief 计算加速度计可信度。
 *
 * 当合成加速度模长接近 1g 时，加速度计能可靠反映重力方向。
 * 偏离越大说明平移加速度成分越多，可信度越低。
 *
 * @param ax 加速度 X (g)
 * @param ay 加速度 Y (g)
 * @param az 加速度 Z (g)
 * @param threshold 偏差阈值 (g)
 * @return 0~1 之间的可信度，1 表示完全可信。
 */
static float Attitude_CalcTrust(float ax, float ay, float az, float threshold)
{
    float norm;
    float err;

    norm = sqrtf(ax * ax + ay * ay + az * az);
    err = fabsf(norm - ATTITUDE_GRAVITY);

    if (err <= threshold)
    {
        return 1.0f;
    }

    if (err >= threshold * 2.0f)
    {
        return 0.0f;
    }

    return 1.0f - (err - threshold) / threshold;
}

void Attitude_Init(attitude_t *att, float sample_hz)
{
    attitude_config_t config;

    if (att == NULL)
    {
        return;
    }

    config.gyro_lpf_hz      = DEFAULT_GYRO_LPF_HZ;
    config.accel_lpf_hz     = DEFAULT_ACCEL_LPF_HZ;
    config.sample_hz        = sample_hz;
    config.compl_alpha      = DEFAULT_COMPL_ALPHA;
    config.accel_trust_thr  = DEFAULT_ACCEL_TRUST_THR;

    Attitude_InitConfig(att, &config);
}

void Attitude_InitConfig(attitude_t *att, const attitude_config_t *config)
{
    if (att == NULL || config == NULL)
    {
        return;
    }

    att->config = *config;

    att->roll  = 0.0f;
    att->pitch = 0.0f;
    att->yaw   = 0.0f;

    att->filt_gyro_x  = 0.0f;
    att->filt_gyro_y  = 0.0f;
    att->filt_gyro_z  = 0.0f;
    att->filt_accel_x = 0.0f;
    att->filt_accel_y = 0.0f;
    att->filt_accel_z = 0.0f;

    Attitude_CalcLpfAlpha(&att->gyro_lpf_alpha,  config->gyro_lpf_hz,  config->sample_hz);
    Attitude_CalcLpfAlpha(&att->accel_lpf_alpha, config->accel_lpf_hz, config->sample_hz);

    att->initialized = 1U;
}

void Attitude_Update(attitude_t *att,
                     float gyro_x, float gyro_y, float gyro_z,
                     float accel_x, float accel_y, float accel_z)
{
    float gx, gy, gz;
    float ax, ay, az;
    float accel_roll, accel_pitch;
    float trust;
    float scale;
    float dt;

    if (att == NULL || att->initialized == 0U)
    {
        return;
    }

    dt = 1.0f / att->config.sample_hz;

    /* 第一层：陀螺仪低通滤波 */
    att->filt_gyro_x = Attitude_LpfUpdate(att->gyro_lpf_alpha, att->filt_gyro_x, gyro_x);
    att->filt_gyro_y = Attitude_LpfUpdate(att->gyro_lpf_alpha, att->filt_gyro_y, gyro_y);
    att->filt_gyro_z = Attitude_LpfUpdate(att->gyro_lpf_alpha, att->filt_gyro_z, gyro_z);

    gx = att->filt_gyro_x;
    gy = att->filt_gyro_y;
    gz = att->filt_gyro_z;

    /* 第一层：加速度计低通滤波 */
    att->filt_accel_x = Attitude_LpfUpdate(att->accel_lpf_alpha, att->filt_accel_x, accel_x);
    att->filt_accel_y = Attitude_LpfUpdate(att->accel_lpf_alpha, att->filt_accel_y, accel_y);
    att->filt_accel_z = Attitude_LpfUpdate(att->accel_lpf_alpha, att->filt_accel_z, accel_z);

    ax = att->filt_accel_x;
    ay = att->filt_accel_y;
    az = att->filt_accel_z;

    /* 从加速度估计姿态角 */
    accel_roll  = atan2f(ay, az);
    accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    /* 加速度可信度：|a| 越接近 1g 越可信 */
    trust = Attitude_CalcTrust(ax, ay, az, att->config.accel_trust_thr);

    /* 第二层：互补滤波 */

    /* roll */
    att->roll += gx * ATTITUDE_D2R * dt;
    scale = (1.0f - att->config.compl_alpha) * trust;
    att->roll += scale * (accel_roll - att->roll);

    /* pitch */
    att->pitch += gy * ATTITUDE_D2R * dt;
    scale = (1.0f - att->config.compl_alpha) * trust;
    att->pitch += scale * (accel_pitch - att->pitch);

    /* yaw: 仅陀螺积分，无磁力计修正，会漂移 */
    att->yaw += gz * ATTITUDE_D2R * dt;
}

void Attitude_Reset(attitude_t *att)
{
    if (att == NULL)
    {
        return;
    }

    att->roll  = 0.0f;
    att->pitch = 0.0f;
    att->yaw   = 0.0f;

    att->filt_gyro_x  = 0.0f;
    att->filt_gyro_y  = 0.0f;
    att->filt_gyro_z  = 0.0f;
    att->filt_accel_x = 0.0f;
    att->filt_accel_y = 0.0f;
    att->filt_accel_z = 0.0f;
}
