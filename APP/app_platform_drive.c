#include "app_platform_drive.h"

#include "TB6612.h"
#include "encoder.h"

#include <stddef.h>
#include <string.h>

static const encoder_id_t s_encoder_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    ENCODER_MOTOR_1,
    ENCODER_MOTOR_2,
    ENCODER_MOTOR_3,
    ENCODER_MOTOR_4
};

static const tb6612_motor_t s_motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    TB6612_MOTOR_1,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4
};

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
    const float motor_square_half_span_m = 0.30f;
    const float platform_half_size_m = 0.055f;
    const float motor_to_platform_center_m = 0.45f;
    const float effective_anchor_offset_m = motor_square_half_span_m - platform_half_size_m;
    const float anchor_height_m = 0.15000001f;
    uint32_t index;

    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    RopePlatformSolver_GetDefaultConfig(&config->solver_config);

    /* 按现场编号定义锚点:
     * 1: 左上, 2: 左下, 3: 右下, 4: 右上
     * x 正方向: 2,3 边 -> 1,4 边
     * y 正方向: 1,2 边 -> 3,4 边
     */
    /* 电机中心组成 60cm x 60cm 正方形，平台边长 11cm。
       状态量使用平台中心，因此锚点采用“电机位置 - 对应平台角点偏移”。
       已知电机到平台中心距离约 45cm，可得 z 方向高度差约 15cm。 */
    (void)motor_to_platform_center_m;
    config->solver_config.anchor[0].x_m = effective_anchor_offset_m;
    config->solver_config.anchor[0].y_m = -effective_anchor_offset_m;
    config->solver_config.anchor[0].z_m = anchor_height_m;
    config->solver_config.anchor[1].x_m = -effective_anchor_offset_m;
    config->solver_config.anchor[1].y_m = -effective_anchor_offset_m;
    config->solver_config.anchor[1].z_m = anchor_height_m;
    config->solver_config.anchor[2].x_m = -effective_anchor_offset_m;
    config->solver_config.anchor[2].y_m = effective_anchor_offset_m;
    config->solver_config.anchor[2].z_m = anchor_height_m;
    config->solver_config.anchor[3].x_m = effective_anchor_offset_m;
    config->solver_config.anchor[3].y_m = effective_anchor_offset_m;
    config->solver_config.anchor[3].z_m = anchor_height_m;

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        config->solver_config.counts_per_rev[index] = 1040.0f;
        config->solver_config.drum_radius_m[index] = 0.011f;
        config->solver_config.direction_sign[index] = 1;

        config->motor_ff_gain[index] = 3500.0f;
        Pid_GetDefaultConfig(&config->motor_speed_pid[index]);
        config->motor_speed_pid[index].kp = 1200.0f;
        config->motor_speed_pid[index].ki = 4000.0f;
        config->motor_speed_pid[index].kd = 0.0f;
        config->motor_speed_pid[index].integral_limit = 350.0f;
        config->motor_speed_pid[index].output_limit = (float)TB6612_SPEED_MAX;
        config->motor_speed_pid[index].derivative_lpf_alpha = 0.25f;
    }

    config->motor_speed_pid[0].kp = 6000.0f;
    config->motor_speed_pid[0].ki = 1700.0f;
    config->motor_speed_pid[0].integral_limit = 220.0f;
    config->pwm_min_effective[0] = 80;

    config->motor_speed_pid[1].kp = 2800.0f;
    config->motor_speed_pid[1].ki = 2800.0f;
    config->motor_speed_pid[1].integral_limit = 260.0f;
    config->pwm_min_effective[1] = 70;

    config->motor_speed_pid[2].kp = 4200.0f;
    config->motor_speed_pid[2].ki = 2950.0f;
    config->motor_speed_pid[2].integral_limit = 320.0f;
    config->pwm_min_effective[2] = 80;

    config->motor_speed_pid[3].kp = 4150.0f;
    config->motor_speed_pid[3].ki = 2950.0f;
    config->motor_speed_pid[3].integral_limit = 320.0f;
    config->pwm_min_effective[3] = 80;

    config->max_platform_speed_mps = 0.15f;
    config->pwm_limit = TB6612_SPEED_MAX;
}

app_platform_drive_status_t AppPlatformDrive_Init(app_platform_drive_t *drive,
                                                  const app_platform_drive_config_t *config)
{
    rope_platform_solver_status_t solver_status;
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint32_t index;

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

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        Pid_Init(&drive->motor_speed_pid[index], &drive->config.motor_speed_pid[index]);
    }

    if (AppPlatformDrive_ReadCounts(counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        drive->solver.last_counts[index] = counts[index];
    }

    (void)TB6612_StopAll();
    drive->initialized = 1U;
    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_CalibrateCurrentPosition(app_platform_drive_t *drive,
                                                                      float known_x_m,
                                                                      float known_y_m)
{
    return AppPlatformDrive_SetCurrentPoseAsOrigin(drive, known_x_m, known_y_m);
}

app_platform_drive_status_t AppPlatformDrive_SetCurrentPoseAsOrigin(app_platform_drive_t *drive,
                                                                    float known_x_m,
                                                                    float known_y_m)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    app_platform_drive_status_t status;
    rope_platform_solver_status_t solver_status;
    uint32_t index;

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

    solver_status = RopePlatformSolver_SetCurrentPoseAsReference(&drive->solver, counts, known_x_m, known_y_m);
    if (solver_status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        Pid_Reset(&drive->motor_speed_pid[index]);
    }

    (void)memset(drive->motor_pwm, 0, sizeof(drive->motor_pwm));
    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_UpdateMotorSpeedLoop(
    app_platform_drive_t *drive,
    const float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float dt_s)
{
    uint32_t index;

    if ((drive == NULL) || (rope_speed_refs_mps == NULL) || (dt_s <= 0.0f))
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const float speed_pid_output = Pid_Update(&drive->motor_speed_pid[index],
                                                  rope_speed_refs_mps[index],
                                                  drive->solver.rope_speeds_mps[index],
                                                  dt_s);
        const float control_pwm = (drive->config.motor_ff_gain[index] * rope_speed_refs_mps[index]) +
                                  speed_pid_output;

        drive->motor_pwm[index] = AppPlatformDrive_ClampPwm(control_pwm,
                                                            drive->config.pwm_limit,
                                                            drive->config.pwm_min_effective[index]);

        if (TB6612_SetMotorSpeed(s_motor_map[index], drive->motor_pwm[index]) != TB6612_STATUS_OK)
        {
            (void)TB6612_StopAll();
            return APP_PLATFORM_DRIVE_STATUS_BSP_ERROR;
        }
    }

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_UpdateStateOnly(app_platform_drive_t *drive, float dt_s)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    app_platform_drive_status_t status;
    rope_platform_solver_status_t solver_status;

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
        return AppPlatformDrive_StatusFromSolver(solver_status);
    }

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_Update(app_platform_drive_t *drive, float dt_s)
{
    float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    app_platform_drive_status_t status;
    rope_platform_solver_status_t solver_status;

    status = AppPlatformDrive_UpdateStateOnly(drive, dt_s);
    if (status != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)TB6612_StopAll();
        return status;
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

    return AppPlatformDrive_UpdateMotorSpeedLoop(drive, rope_speed_refs_mps, dt_s);
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

    /* 现场实物的电机正方向与几何模型中的正绳长变化方向相反，
       这里统一翻转平台速度指令方向，保证手动指令和位置环方向一致。 */
    drive->command_vx_mps = AppPlatformDrive_ClampFloat(-vx_mps, -max_speed_mps, max_speed_mps);
    drive->command_vy_mps = AppPlatformDrive_ClampFloat(-vy_mps, -max_speed_mps, max_speed_mps);

    return APP_PLATFORM_DRIVE_STATUS_OK;
}

app_platform_drive_status_t AppPlatformDrive_MoveForward(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, speed_mps, 0.0f);
}

app_platform_drive_status_t AppPlatformDrive_MoveBackward(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, -speed_mps, 0.0f);
}

app_platform_drive_status_t AppPlatformDrive_MoveLeft(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, 0.0f, -speed_mps);
}

app_platform_drive_status_t AppPlatformDrive_MoveRight(app_platform_drive_t *drive, float speed_mps)
{
    return AppPlatformDrive_SetCommandVelocity(drive, 0.0f, speed_mps);
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
        Pid_Reset(&drive->motor_speed_pid[index]);
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
