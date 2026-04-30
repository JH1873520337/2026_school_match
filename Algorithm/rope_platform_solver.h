#ifndef ROPE_PLATFORM_SOLVER_H
#define ROPE_PLATFORM_SOLVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define ROPE_PLATFORM_SOLVER_CABLE_COUNT    (4U)

/**
 * @brief 绳驱平台解算状态码
 */
typedef enum
{
    ROPE_PLATFORM_SOLVER_STATUS_OK = 0,
    ROPE_PLATFORM_SOLVER_STATUS_BAD_PARAM,
    ROPE_PLATFORM_SOLVER_STATUS_NOT_INIT,
    ROPE_PLATFORM_SOLVER_STATUS_NOT_CALIBRATED,
    ROPE_PLATFORM_SOLVER_STATUS_NUMERICAL_ERROR,
    ROPE_PLATFORM_SOLVER_STATUS_GEOMETRY_ERROR
} rope_platform_solver_status_t;

/**
 * @brief 单个锚点坐标
 * @note 平台运动平面定义为 z = 0，锚点 z_m 一般为正值
 */
typedef struct
{
    float x_m;
    float y_m;
    float z_m;
} rope_anchor_point_t;

/**
 * @brief 绳驱平台解算配置
 */
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

/**
 * @brief 绳驱平台解算器运行状态
 */
typedef struct
{
    rope_platform_solver_config_t config;
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

/**
 * @brief 获取一份默认配置
 * @param config 输出配置结构体指针
 * @note 默认值仅用于快速起步，锚点、计数当量、卷筒半径都必须结合实物标定
 */
void RopePlatformSolver_GetDefaultConfig(rope_platform_solver_config_t *config);

/**
 * @brief 初始化绳驱平台解算器
 * @param solver 解算器对象指针
 * @param config 配置参数指针
 * @return 解算状态码
 */
rope_platform_solver_status_t RopePlatformSolver_Init(rope_platform_solver_t *solver,
                                                      const rope_platform_solver_config_t *config);

/**
 * @brief 设置零点参考
 * @param solver 解算器对象指针
 * @param counts 当前四路编码器累计计数
 * @param known_x_m 当前已知平台 x 坐标，单位 m
 * @param known_y_m 当前已知平台 y 坐标，单位 m
 * @return 解算状态码
 * @note 常用于“平台放在中心点”后记录零点
 */
rope_platform_solver_status_t RopePlatformSolver_SetZeroReference(rope_platform_solver_t *solver,
                                                                  const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
                                                                  float known_x_m,
                                                                  float known_y_m);

/**
 * @brief 用最新编码器计数更新绳长、平台位置和平台速度
 * @param solver 解算器对象指针
 * @param counts 当前四路编码器累计计数
 * @param dt_s 控制周期，单位 s
 * @return 解算状态码
 */
rope_platform_solver_status_t RopePlatformSolver_Update(rope_platform_solver_t *solver,
                                                        const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
                                                        float dt_s);

/**
 * @brief 根据当前平台位置，把平台速度映射为四根绳的目标线速度
 * @param solver 解算器对象指针
 * @param vx_mps 平台 x 方向目标速度，单位 m/s
 * @param vy_mps 平台 y 方向目标速度，单位 m/s
 * @param rope_speed_refs_mps 输出的四根绳目标线速度，单位 m/s
 * @return 解算状态码
 */
rope_platform_solver_status_t RopePlatformSolver_MapPlatformVelocityToRopeSpeeds(
    const rope_platform_solver_t *solver,
    float vx_mps,
    float vy_mps,
    float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
