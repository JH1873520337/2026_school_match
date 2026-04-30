#include "rope_platform_solver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef ROPE_PLATFORM_PI
#define ROPE_PLATFORM_PI    (3.14159265358979323846f)
#endif

/**
 * @brief 非线性迭代过程的中间结果
 */
typedef struct
{
    float guess_x_m;
    float guess_y_m;
    float residual_rms_m;
} rope_platform_solver_iteration_t;

/**
 * @brief 把 alpha 限制到 [0, 1]
 * @param alpha 待限制的低通滤波系数
 * @return 限制后的 alpha
 */
static float RopePlatformSolver_ClampAlpha(float alpha)
{
    if (alpha < 0.0f)
    {
        return 0.0f;
    }

    if (alpha > 1.0f)
    {
        return 1.0f;
    }

    return alpha;
}

/**
 * @brief 一阶低通滤波
 * @param input 当前输入值
 * @param previous 上一次输出值
 * @param alpha 滤波系数
 * @return 当前滤波输出值
 */
static float RopePlatformSolver_LowPass(float input, float previous, float alpha)
{
    alpha = RopePlatformSolver_ClampAlpha(alpha);
    return previous + (alpha * (input - previous));
}

/**
 * @brief 判断一个值是否为有限正数
 * @param value 待判断的数值
 * @return 1 表示有效，0 表示无效
 */
static uint8_t RopePlatformSolver_IsFinitePositive(float value)
{
    return (uint8_t)((isfinite(value) != 0) && (value > 0.0f));
}

/**
 * @brief 校验配置参数是否合法
 * @param config 配置结构体指针
 * @return 解算状态码
 */
static rope_platform_solver_status_t RopePlatformSolver_ValidateConfig(
    const rope_platform_solver_config_t *config)
{
    uint32_t index;

    if (config == NULL)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    if ((config->max_iterations == 0U) || (config->solver_tolerance_m <= 0.0f))
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        if (!RopePlatformSolver_IsFinitePositive(config->counts_per_rev[index]) ||
            !RopePlatformSolver_IsFinitePositive(config->drum_radius_m[index]) ||
            !RopePlatformSolver_IsFinitePositive(config->anchor[index].z_m))
        {
            return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
        }

        if ((config->direction_sign[index] != 1) && (config->direction_sign[index] != -1))
        {
            return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
        }

        if ((isfinite(config->anchor[index].x_m) == 0) || (isfinite(config->anchor[index].y_m) == 0))
        {
            return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
        }
    }

    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}

/**
 * @brief 计算平台到某个锚点的绳长
 * @param anchor 锚点坐标
 * @param platform_x_m 平台 x 坐标
 * @param platform_y_m 平台 y 坐标
 * @return 当前绳长，单位 m
 */
static float RopePlatformSolver_DistanceToAnchor(const rope_anchor_point_t *anchor,
                                                 float platform_x_m,
                                                 float platform_y_m)
{
    const float dx = platform_x_m - anchor->x_m;
    const float dy = platform_y_m - anchor->y_m;
    return sqrtf((dx * dx) + (dy * dy) + (anchor->z_m * anchor->z_m));
}

/**
 * @brief 根据编码器累计计数重建四根绳的绝对长度
 * @param solver 解算器对象指针
 * @param counts 当前四路编码器累计计数
 * @param lengths_m 输出的四根绳长度
 */
static void RopePlatformSolver_BuildLengths(const rope_platform_solver_t *solver,
                                            const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
                                            float lengths_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    uint32_t index;

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t delta_count = counts[index] - solver->zero_counts[index];
        const float delta_theta_rad = (2.0f * ROPE_PLATFORM_PI * (float)delta_count) /
                                      solver->config.counts_per_rev[index];
        const float delta_length_m = (float)solver->config.direction_sign[index] *
                                     solver->config.drum_radius_m[index] *
                                     delta_theta_rad;

        lengths_m[index] = solver->zero_lengths_m[index] + delta_length_m;
    }
}

/**
 * @brief 用高斯牛顿法求解平台二维位置
 * @param solver 解算器对象指针
 * @param lengths_m 当前四根绳长度
 * @param initial_x_m x 方向初值
 * @param initial_y_m y 方向初值
 * @param result 输出迭代结果
 * @return 解算状态码
 */
static rope_platform_solver_status_t RopePlatformSolver_SolvePosition(
    const rope_platform_solver_t *solver,
    const float lengths_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float initial_x_m,
    float initial_y_m,
    rope_platform_solver_iteration_t *result)
{
    uint8_t iteration;
    float guess_x_m = initial_x_m;
    float guess_y_m = initial_y_m;
    float residual_sum = 0.0f;

    if (result == NULL)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    for (iteration = 0U; iteration < solver->config.max_iterations; iteration++)
    {
        float jtj_00 = 0.0f;
        float jtj_01 = 0.0f;
        float jtj_11 = 0.0f;
        float jtr_0 = 0.0f;
        float jtr_1 = 0.0f;
        uint32_t cable_index;

        residual_sum = 0.0f;

        for (cable_index = 0U; cable_index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; cable_index++)
        {
            const float dx = guess_x_m - solver->config.anchor[cable_index].x_m;
            const float dy = guess_y_m - solver->config.anchor[cable_index].y_m;
            const float dz = solver->config.anchor[cable_index].z_m;
            const float model_length_m = sqrtf((dx * dx) + (dy * dy) + (dz * dz));
            const float residual_m = model_length_m - lengths_m[cable_index];
            float jacobian_x;
            float jacobian_y;

            if (model_length_m <= 1.0e-6f)
            {
                return ROPE_PLATFORM_SOLVER_STATUS_NUMERICAL_ERROR;
            }

            jacobian_x = dx / model_length_m;
            jacobian_y = dy / model_length_m;

            jtj_00 += jacobian_x * jacobian_x;
            jtj_01 += jacobian_x * jacobian_y;
            jtj_11 += jacobian_y * jacobian_y;
            jtr_0 += jacobian_x * residual_m;
            jtr_1 += jacobian_y * residual_m;
            residual_sum += residual_m * residual_m;
        }

        {
            const float determinant = (jtj_00 * jtj_11) - (jtj_01 * jtj_01);
            float delta_x_m;
            float delta_y_m;

            if (fabsf(determinant) <= 1.0e-9f)
            {
                return ROPE_PLATFORM_SOLVER_STATUS_NUMERICAL_ERROR;
            }

            delta_x_m = ((-jtr_0 * jtj_11) - (-jtr_1 * jtj_01)) / determinant;
            delta_y_m = ((jtj_01 * jtr_0) - (jtj_00 * jtr_1)) / determinant;

            guess_x_m += delta_x_m;
            guess_y_m += delta_y_m;

            if ((delta_x_m * delta_x_m) + (delta_y_m * delta_y_m) <=
                (solver->config.solver_tolerance_m * solver->config.solver_tolerance_m))
            {
                break;
            }
        }
    }

    result->guess_x_m = guess_x_m;
    result->guess_y_m = guess_y_m;
    result->residual_rms_m = sqrtf(residual_sum / (float)ROPE_PLATFORM_SOLVER_CABLE_COUNT);
    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}

void RopePlatformSolver_GetDefaultConfig(rope_platform_solver_config_t *config)
{
    uint32_t index;

    if (config == NULL)
    {
        return;
    }

    (void)memset(config, 0, sizeof(*config));

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        config->counts_per_rev[index] = 1.0f;
        config->drum_radius_m[index] = 0.01f;
        config->direction_sign[index] = 1;
        config->anchor[index].z_m = 0.30f;
    }

    config->rope_speed_lpf_alpha = 0.2f;
    config->platform_speed_lpf_alpha = 0.2f;
    config->solver_tolerance_m = 1.0e-5f;
    config->max_iterations = 8U;
}

rope_platform_solver_status_t RopePlatformSolver_Init(rope_platform_solver_t *solver,
                                                      const rope_platform_solver_config_t *config)
{
    rope_platform_solver_status_t status;

    if (solver == NULL)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    status = RopePlatformSolver_ValidateConfig(config);
    if (status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return status;
    }

    (void)memset(solver, 0, sizeof(*solver));
    solver->config = *config;
    solver->initialized = 1U;
    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}

rope_platform_solver_status_t RopePlatformSolver_SetZeroReference(rope_platform_solver_t *solver,
                                                                  const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
                                                                  float known_x_m,
                                                                  float known_y_m)
{
    uint32_t index;

    if ((solver == NULL) || (counts == NULL))
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    if (solver->initialized == 0U)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT;
    }

    if ((isfinite(known_x_m) == 0) || (isfinite(known_y_m) == 0))
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const float base_length_m = RopePlatformSolver_DistanceToAnchor(&solver->config.anchor[index],
                                                                        known_x_m,
                                                                        known_y_m);

        if (!RopePlatformSolver_IsFinitePositive(base_length_m))
        {
            return ROPE_PLATFORM_SOLVER_STATUS_GEOMETRY_ERROR;
        }

        solver->zero_counts[index] = counts[index];
        solver->zero_lengths_m[index] = base_length_m;
        solver->rope_lengths_m[index] = base_length_m;
        solver->rope_speeds_mps[index] = 0.0f;
    }

    solver->position_x_m = known_x_m;
    solver->position_y_m = known_y_m;
    solver->velocity_x_mps = 0.0f;
    solver->velocity_y_mps = 0.0f;
    solver->residual_rms_m = 0.0f;
    solver->zero_valid = 1U;
    solver->state_valid = 1U;

    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}

rope_platform_solver_status_t RopePlatformSolver_Update(rope_platform_solver_t *solver,
                                                        const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
                                                        float dt_s)
{
    float new_lengths_m[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    rope_platform_solver_iteration_t solve_result;
    float initial_x_m;
    float initial_y_m;
    uint32_t index;
    rope_platform_solver_status_t status;

    if ((solver == NULL) || (counts == NULL) || (dt_s <= 0.0f))
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    if (solver->initialized == 0U)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT;
    }

    if (solver->zero_valid == 0U)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_NOT_CALIBRATED;
    }

    RopePlatformSolver_BuildLengths(solver, counts, new_lengths_m);

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const float rope_speed_raw_mps = (new_lengths_m[index] - solver->rope_lengths_m[index]) / dt_s;

        if (!RopePlatformSolver_IsFinitePositive(new_lengths_m[index]))
        {
            return ROPE_PLATFORM_SOLVER_STATUS_GEOMETRY_ERROR;
        }

        solver->rope_speeds_mps[index] = RopePlatformSolver_LowPass(rope_speed_raw_mps,
                                                                    solver->rope_speeds_mps[index],
                                                                    solver->config.rope_speed_lpf_alpha);
    }

    initial_x_m = solver->position_x_m;
    initial_y_m = solver->position_y_m;

    status = RopePlatformSolver_SolvePosition(solver,
                                              new_lengths_m,
                                              initial_x_m,
                                              initial_y_m,
                                              &solve_result);
    if (status != ROPE_PLATFORM_SOLVER_STATUS_OK)
    {
        return status;
    }

    {
        const float velocity_x_raw_mps = (solve_result.guess_x_m - solver->position_x_m) / dt_s;
        const float velocity_y_raw_mps = (solve_result.guess_y_m - solver->position_y_m) / dt_s;

        solver->velocity_x_mps = RopePlatformSolver_LowPass(velocity_x_raw_mps,
                                                            solver->velocity_x_mps,
                                                            solver->config.platform_speed_lpf_alpha);
        solver->velocity_y_mps = RopePlatformSolver_LowPass(velocity_y_raw_mps,
                                                            solver->velocity_y_mps,
                                                            solver->config.platform_speed_lpf_alpha);
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        solver->rope_lengths_m[index] = new_lengths_m[index];
    }

    solver->position_x_m = solve_result.guess_x_m;
    solver->position_y_m = solve_result.guess_y_m;
    solver->residual_rms_m = solve_result.residual_rms_m;
    solver->state_valid = 1U;

    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}

rope_platform_solver_status_t RopePlatformSolver_MapPlatformVelocityToRopeSpeeds(
    const rope_platform_solver_t *solver,
    float vx_mps,
    float vy_mps,
    float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    uint32_t index;

    if ((solver == NULL) || (rope_speed_refs_mps == NULL))
    {
        return ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM;
    }

    if (solver->initialized == 0U)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT;
    }

    if (solver->state_valid == 0U)
    {
        return ROPE_PLATFORM_SOLVER_STATUS_NOT_CALIBRATED;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const float dx = solver->position_x_m - solver->config.anchor[index].x_m;
        const float dy = solver->position_y_m - solver->config.anchor[index].y_m;
        const float rope_length_m = solver->rope_lengths_m[index];

        if (rope_length_m <= 1.0e-6f)
        {
            return ROPE_PLATFORM_SOLVER_STATUS_NUMERICAL_ERROR;
        }

        /* dL/dt = (dx * vx + dy * vy) / L */
        rope_speed_refs_mps[index] = ((dx * vx_mps) + (dy * vy_mps)) / rope_length_m;
    }

    return ROPE_PLATFORM_SOLVER_STATUS_OK;
}
