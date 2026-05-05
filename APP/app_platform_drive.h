#ifndef APP_PLATFORM_DRIVE_H
#define APP_PLATFORM_DRIVE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "pid.h"
#include "rope_platform_solver.h"

#include <stdint.h>

/**
 * @brief APP 层平台驱动状态码
 */
typedef enum
{
    APP_PLATFORM_DRIVE_STATUS_OK = 0,
    APP_PLATFORM_DRIVE_STATUS_BAD_PARAM,
    APP_PLATFORM_DRIVE_STATUS_NOT_INIT,
    APP_PLATFORM_DRIVE_STATUS_NOT_CALIBRATED,
    APP_PLATFORM_DRIVE_STATUS_BSP_ERROR,
    APP_PLATFORM_DRIVE_STATUS_SOLVER_ERROR
} app_platform_drive_status_t;

/**
 * @brief APP 层平台驱动配置
 * @note 这里同时包含了解算参数和简易控制器参数
 */
typedef struct
{
    rope_platform_solver_config_t solver_config;
    float motor_ff_gain[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    pid_config_t motor_speed_pid[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int16_t pwm_min_effective[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    float max_platform_speed_mps;
    int16_t pwm_limit;
} app_platform_drive_config_t;

/**
 * @brief APP 层平台驱动对象
 */
typedef struct
{
    rope_platform_solver_t solver;
    app_platform_drive_config_t config;
    float command_vx_mps;
    float command_vy_mps;
    pid_controller_t motor_speed_pid[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int16_t motor_pwm[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint8_t initialized;
} app_platform_drive_t;

/**
 * @brief 获取一份默认配置
 * @param config 输出配置结构体指针
 * @note 默认值仅为起步值，锚点、计数当量、卷筒半径和控制参数都需要结合实物调试
 */
void AppPlatformDrive_GetDefaultConfig(app_platform_drive_config_t *config);

/**
 * @brief 初始化平台简易运动驱动
 * @param drive 驱动对象指针
 * @param config 配置参数指针
 * @return 驱动状态码
 * @note 该函数会初始化 Encoder 和 TB6612 底层
 */
app_platform_drive_status_t AppPlatformDrive_Init(app_platform_drive_t *drive,
                                                  const app_platform_drive_config_t *config);

/**
 * @brief 以当前位置作为已知点进行零点标定
 * @param drive 驱动对象指针
 * @param known_x_m 当前已知平台 x 坐标，单位 m
 * @param known_y_m 当前已知平台 y 坐标，单位 m
 * @return 驱动状态码
 * @note 常见用法是把平台放到中心点后传入 (0.0f, 0.0f)
 */
app_platform_drive_status_t AppPlatformDrive_CalibrateCurrentPosition(app_platform_drive_t *drive,
                                                                      float known_x_m,
                                                                      float known_y_m);

app_platform_drive_status_t AppPlatformDrive_SetCurrentPoseAsOrigin(app_platform_drive_t *drive,
                                                                    float known_x_m,
                                                                    float known_y_m);

/**
 * @brief 周期调用的平台控制更新函数
 * @param drive 驱动对象指针
 * @param dt_s 控制周期，单位 s
 * @return 驱动状态码
 * @note 该函数会完成：读编码器 -> 解算位置/速度 -> 速度分配 -> 计算 PWM -> 下发电机
 */
app_platform_drive_status_t AppPlatformDrive_Update(app_platform_drive_t *drive, float dt_s);

/**
 * @brief 只更新编码器/绳长/平台位姿，不输出电机 PWM
 * @param drive 驱动对象指针
 * @param dt_s 控制周期，单位 s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_UpdateStateOnly(app_platform_drive_t *drive, float dt_s);

/**
 * @brief 设置平台目标速度
 * @param drive 驱动对象指针
 * @param vx_mps x 方向目标速度，单位 m/s
 * @param vy_mps y 方向目标速度，单位 m/s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_SetCommandVelocity(app_platform_drive_t *drive,
                                                                float vx_mps,
                                                                float vy_mps);

/**
 * @brief 让平台向前运动
 * @param drive 驱动对象指针
 * @param speed_mps 目标速度，单位 m/s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_MoveForward(app_platform_drive_t *drive, float speed_mps);

/**
 * @brief 让平台向后运动
 * @param drive 驱动对象指针
 * @param speed_mps 目标速度，单位 m/s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_MoveBackward(app_platform_drive_t *drive, float speed_mps);

/**
 * @brief 让平台向左运动
 * @param drive 驱动对象指针
 * @param speed_mps 目标速度，单位 m/s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_MoveLeft(app_platform_drive_t *drive, float speed_mps);

/**
 * @brief 让平台向右运动
 * @param drive 驱动对象指针
 * @param speed_mps 目标速度，单位 m/s
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_MoveRight(app_platform_drive_t *drive, float speed_mps);

/**
 * @brief 停止平台运动
 * @param drive 驱动对象指针
 * @return 驱动状态码
 */
app_platform_drive_status_t AppPlatformDrive_Stop(app_platform_drive_t *drive);

/**
 * @brief 获取内部解算器对象
 * @param drive 驱动对象指针
 * @return 解算器只读指针，失败时返回 NULL
 */
const rope_platform_solver_t *AppPlatformDrive_GetSolver(const app_platform_drive_t *drive);

app_platform_drive_status_t AppPlatformDrive_UpdateMotorSpeedLoop(
    app_platform_drive_t *drive,
    const float rope_speed_refs_mps[ROPE_PLATFORM_SOLVER_CABLE_COUNT],
    float dt_s);

#ifdef __cplusplus
}
#endif

#endif
