#ifndef ROPE_PLATFORM_SOLVER_H
#define ROPE_PLATFORM_SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ROPE_PLATFORM_SOLVER_CABLE_COUNT    (4U)

typedef enum
{
    ROPE_PLATFORM_SOLVER_STATUS_OK = 0,
    ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM,
    ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT,
    ROPE_PLATFORM_SOLVER_STATUS_NOT_CALIBRATED,
    ROPE_PLATFORM_SOLVER_STATUS_NUMERICAL_ERROR,
    ROPE_PLATFORM_SOLVER_STATUS_GEOMETRY_ERROR
} rope_platform_solver_status_t;

typedef struct
{
    float x_m;
    float y_m;
    float z_m;
} rope_anchor_point_t;

typedef struct
{
    rope_anchor_point_t anchor[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float counts_per_rev[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float drum_radius_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int8_t direction_sign[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float rope_speed_lpf_alpha;
    float platform_speed_lpf_alpha;
    float solver_tolerance_m;
    uint8_t max_iterations;
} rope_platform_solver_config_t;

typedef struct
{
    rope_platform_solver_config_t config;
    int32_t last_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int32_t zero_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float zero_lengths_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float rope_lengths_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float rope_speeds_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float position_x_m;
    float position_y_m;
    float velocity_x_mps;
    float velocity_y_mps;
    float residual_rms_m;
    uint8_t initialized;
    uint8_t zero_valid;
    uint8_t state_valid;
} rope_platform_solver_t;

void RopePlatformSolver_GetDefaultConfig(rope_platform_solver_config_t *config);

rope_platform_solver_status_t RopePlatformSolver_Init(rope_platform_solver_t *solver,
                                                      const rope_platform_solver_config_t *config);

rope_platform_solver_status_t RopePlatformSolver_UpdateRopeSpeeds(
    rope_platform_solver_t *solver,
    const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float dt_s);

rope_platform_solver_status_t RopePlatformSolver_SetZeroReference(
    rope_platform_solver_t *solver,
    const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float known_x_m,
    float known_y_m);

rope_platform_solver_status_t RopePlatformSolver_SetCurrentPoseAsReference(
    rope_platform_solver_t *solver,
    const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float known_x_m,
    float known_y_m);

rope_platform_solver_status_t RopePlatformSolver_Update(
    rope_platform_solver_t *solver,
    const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float dt_s);

rope_platform_solver_status_t RopePlatformSolver_MapPlatformVelocityToRopeSpeeds(
    const rope_platform_solver_t *solver,
    float vx_mps,
    float vy_mps,
    float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
