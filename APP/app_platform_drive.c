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
    const float effective_anchor_offset_m = motor_square_half_span_m - platform_half_size_m;
    const float anchor_height_m = 0.15000001f;
    uint32_t index;

    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));
    RopePlatformSolver_GetDefaultConfig(&config->solver_config);

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
    }
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

app_platform_drive_status_t AppPlatformDrive_GetCurrentCounts(
    app_platform_drive_t *drive,
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    if ((drive == NULL) || (counts == NULL))
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    return AppPlatformDrive_ReadCounts(counts);
}

app_platform_drive_status_t AppPlatformDrive_Stop(app_platform_drive_t *drive)
{
    if (drive == NULL)
    {
        return APP_PLATFORM_DRIVE_STATUS_BAD_PARAM;
    }

    if (drive->initialized == 0U)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
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
