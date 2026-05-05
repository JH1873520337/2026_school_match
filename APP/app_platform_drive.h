#ifndef APP_PLATFORM_DRIVE_H
#define APP_PLATFORM_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rope_platform_solver.h"

#include <stdint.h>

typedef enum
{
    APP_PLATFORM_DRIVE_STATUS_OK = 0,
    APP_PLATFORM_DRIVE_STATUS_BAD_PARAM,
    APP_PLATFORM_DRIVE_STATUS_NOT_INIT,
    APP_PLATFORM_DRIVE_STATUS_NOT_CALIBRATED,
    APP_PLATFORM_DRIVE_STATUS_BSP_ERROR,
    APP_PLATFORM_DRIVE_STATUS_SOLVER_ERROR
} app_platform_drive_status_t;

typedef struct
{
    rope_platform_solver_config_t solver_config;
} app_platform_drive_config_t;

typedef struct
{
    rope_platform_solver_t solver;
    app_platform_drive_config_t config;
    uint8_t initialized;
} app_platform_drive_t;

void AppPlatformDrive_GetDefaultConfig(app_platform_drive_config_t *config);

app_platform_drive_status_t AppPlatformDrive_Init(app_platform_drive_t *drive,
                                                  const app_platform_drive_config_t *config);

app_platform_drive_status_t AppPlatformDrive_CalibrateCurrentPosition(app_platform_drive_t *drive,
                                                                      float known_x_m,
                                                                      float known_y_m);

app_platform_drive_status_t AppPlatformDrive_SetCurrentPoseAsOrigin(app_platform_drive_t *drive,
                                                                    float known_x_m,
                                                                    float known_y_m);

app_platform_drive_status_t AppPlatformDrive_UpdateStateOnly(app_platform_drive_t *drive, float dt_s);

app_platform_drive_status_t AppPlatformDrive_GetCurrentCounts(
    app_platform_drive_t *drive,
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

app_platform_drive_status_t AppPlatformDrive_Stop(app_platform_drive_t *drive);

const rope_platform_solver_t *AppPlatformDrive_GetSolver(const app_platform_drive_t *drive);

#ifdef __cplusplus
}
#endif

#endif
