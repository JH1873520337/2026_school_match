#include "app_platform_drive.h"

#include "TB6612.h"
#include "encoder.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 绳索编号到编码器编号的映射
 */
static const encoder_id_t s_encoder_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    ENCODER_MOTOR_1,
    ENCODER_MOTOR_2,
    ENCODER_MOTOR_3,
    ENCODER_MOTOR_4
};

/**
 * @brief 绳索编号到电机驱动编号的映射
 */
static const tb6612_motor_t s_motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    TB6612_MOTOR_1,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4
};

/**
 * @brief 限幅浮点数
 * @param value 待限幅的值
 * @param min_value 最小值
 * @param max_value 最大值
 * @return 限幅后的值
 */
static float AppPlatformDrive_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
 * @brief 把控制量限制到电机可接受的 PWM 范围
 * @param pwm_value 输入的浮点 PWM
 * @param pwm_limit 绝对最大 PWM
 * @param pwm_min_effective 克服静摩擦的最小有效 PWM
 * @return 限幅后的整数 PWM
 */
static int16_t AppPlatformDrive_ClampPwm(float pwm_value, int16_t pwm_limit, int16_t pwm_min_effective)
{
    if (pwm_value > (float)pwm_limit)
    {
        pwm_value = (float)pwm_limit;
    }
    else if (pwm_value < -(float)pwm_limit)
    {
        pwm_value = -(float)pwm_limit;
    }

    if ((pwm_value > 0.0f) && (pwm_value < (float)pwm_min_effective))
    {
        pwm_value = (float)pwm_min_effective;
    }
    else if ((pwm_value < 0.0f) && (pwm_value > -(float)pwm_min_effective))
    {
        pwm_value = -(float)pwm_min_effective;
    }

    return (int16_t)pwm_value;
}

/**
 * @brief 读取四路编码器当前累计计数
 * @param counts 输出计数数组
 * @return 驱动状态码
 */
static app_platform_drive_status_t AppPlatformDrive_ReadCounts(
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    uint32_t index;

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        if (Encoder_GetCount(s_encoder_map[index], &counts[index]) != ENCODER_STATUS_OK)
        {
            return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
        }
    }

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

/**
 * @brief 把解算器状态码转换成 APP 状态码
 * @param solver_status 解算器状态码
 * @return APP 层状态码
 */
static app_platform_drive_status_t AppPlatformDrive_StatusFromSolver(
    rope_platform_solver_status_t solver_status)
{
    if (solver_status == ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return APP_PLATFORM_DRIVE_STATUS_OK;
    }

    if (solver_status == ROPE_PLATFORM_SOLVER_STATUS_NOT_CALIBRATED)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_CALIBRATED;
    }

    if (solver_status == ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    if (solver_status == ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM)
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    return APP_PLATFORM_DRIVE_STATUS_SOLVER_ERROR;
}

void AppPlatformDrive_GetDefaultConfig(app_platform_drive_config_t *config)
{
    uint32_t index;

    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    RopePlatformSolver_GetDefaultConfig(&config->solver_config);

    config->solver_config.anchor[0].x_m = -0.30f;
    config->solver_config.anchor[0].y_m =  0.30f;
    config->solver_config.anchor[1].x_m =  0.30f;
    config->solver_config.anchor[1].y_m =  0.30f;
    config->solver_config.anchor[2].x_m = -0.30f;
    config->solver_config.anchor[2].y_m = -0.30f;
    config->solver_config.anchor[3].x_m =  0.30f;
    config->solver_config.anchor[3].y_m = -0.30f;

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        /* 下面这些值只是默认起步值，必须按你的实物重新标定。 */
        config->solver_config.counts_per_rev[index] = 1560.0f;
        config->solver_config.drum_radius_m[index] = 0.012f;
        config->solver_config.direction_sign[index] = 1;

        config->motor_ff_gain[index] = 3500.0f;
        config->motor_kp[index] = 1200.0f;
        config->motor_ki[index] = 4000.0f;
        config->motor_i_limit[index] = 350.0f;
    }

    config->max_platform_speed_mps = 0.15f;
    config->pwm_limit = TB6612_SPEED_MAX;
    config->pwm_min_effective = 120;
}

app_platform_drive_status_t AppPlatformDrive_Init(app_platform_drive_t *drive,
                                                  const app_platform_drive_config_t *config)
{
    rope_platform_solver_status_t solver_status;

    if ((drive == NULL) || (config == NULL))
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    (void)memset(drive, 0, sizeof(*drive));
    drive->config = *config;

    if (Encoder_Init() != ENCODER_STATUS_OK)
    {
        return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
    }

    if (TB6612_Init() != TB6612_STATUS_OK)
    {
        return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
    }

    solver_status = RopePlatformSolver_Init(&drive->solver, &drive->config.solver_config);
    if (solver_status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    (void)TB6612_StopAll();
    drive->initialized = 1U;
    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_CalibrateCurrentPosition(app_platform_drive_t *drive,
                                                                      float known_x_m,
                                                                      float known_y_m)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    app_platform_drive_status_t status;
    rope_platform_solver_status_t solver_status;

    if (drive == NULL)
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    status = AppPlatformDrive_ReadCounts(counts);
    if (status != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return status;
    }

    solver_status = RopePlatformSolver_SetZeroReference(&drive->solver, counts, known_x_m, known_y_m);
    if (solver_status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    (void)memset(drive->motor_i_term, 0, sizeof(drive->motor_i_term));
    (void)memset(drive->motor_pwm, 0, sizeof(drive->motor_pwm));
    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_Update(app_platform_drive_t *drive, float dt_s)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    app_platform_drive_status_t status;
    rope_platform_solver_status_t solver_status;
    uint32_t index;

    if ((drive == NULL) || (dt_s <= 0.0f))
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    status = AppPlatformDrive_ReadCounts(counts);
    if (status != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return status;
    }

    solver_status = RopePlatformSolver_Update(&drive->solver, counts, dt_s);
    if (solver_status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        (void)TB6612_StopAll();
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    solver_status = RopePlatformSolver_MapPlatformVelocityToRopeSpeeds(&drive->solver,
                                                                       drive->command_vx_mps,
                                                                       drive->command_vy_mps,
                                                                       rope_speed_refs_mps);
    if (solver_status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        (void)TB6612_StopAll();
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const float speed_error_mps = rope_speed_refs_mps[index] - drive->solver.rope_speeds_mps[index];
        float control_pwm;

        /* 这里采用“前馈 + P + I”的简化绳速控制，先满足平台基础移动。 */
        drive->motor_i_term[index] += drive->config.motor_ki[index] * speed_error_mps * dt_s;
        drive->motor_i_term[index] = AppPlatformDrive_ClampFloat(drive->motor_i_term[index],
                                                                 -drive->config.motor_i_limit[index],
                                                                 drive->config.motor_i_limit[index]);

        control_pwm = (drive->config.motor_ff_gain[index] * rope_speed_refs_mps[index]) +
                      (drive->config.motor_kp[index] * speed_error_mps) +
                      drive->motor_i_term[index];

        drive->motor_pwm[index] = AppPlatformDrive_ClampPwm(control_pwm,
                                                            drive->config.pwm_limit,
                                                            drive->config.pwm_min_effective);

        if (TB6612_SetMotorSpeed(s_motor_map[index], drive->motor_pwm[index]) != TB6612_STATUS_OK)
        {
            (void)TB6612_StopAll();
            return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
        }
    }

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_SetCommandVelocity(app_platform_drive_t *drive,
                                                                float vx_mps,
                                                                float vy_mps)
{
    const float max_speed_mps = (drive != NULL) ? drive->config.max_platform_speed_mps : 0.0f;

    if (drive == NULL)
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    drive->command_vx_mps = AppPlatformDrive_ClampFloat(vx_mps, -max_speed_mps, max_speed_mps);
    drive->command_vy_mps = AppPlatformDrive_ClampFloat(vy_mps, -max_speed_mps, max_speed_mps);

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_MoveForward(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, 0.0f, speed_mps);
}

app_platform_drive_status_t AppPlatformDrive_MoveBackward(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, 0.0f, -speed_mps);
}

app_platform_drive_status_t AppPlatformDrive_MoveLeft(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, -speed_mps, 0.0f);
}

app_platform_drive_status_t AppPlatformDrive_MoveRight(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, speed_mps, 0.0f);
}

app_platform_drive_status_t AppPlatformDrive_Stop(app_platform_drive_t *drive)
{
    uint32_t index;

    if (drive == NULL)
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    drive->command_vx_mps = 0.0f;
    drive->command_vy_mps = 0.0f;

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        drive->motor_i_term[index] = 0.0f;
        drive->motor_pwm[index] = 0;
    }

    if (TB6612_StopAll() != TB6612_STATUS_OK)
    {
        return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
    }

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

const rope_platform_solver_t *AppPlatformDrive_GetSolver(const app_platform_drive_t *drive)
{
    if (drive == NULL)
    {
        return NULL;
    }

    return &drive->solver;
}
