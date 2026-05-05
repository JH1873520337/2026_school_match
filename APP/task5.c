#include "task5.h"

#include "TB6612.h"
#include "screen_uart.h"

#include <stddef.h>
#include <string.h>

#define TASK5_GOTO_PWM_FAST_ABS          ((int16_t)360)
#define TASK5_GOTO_PWM_MID_ABS           ((int16_t)300)
#define TASK5_GOTO_COUNT_TOLERANCE       ((int32_t)15)
#define TASK5_GOTO_COUNT_NEAR_THRESHOLD  ((int32_t)120)
#define TASK5_GOTO_COUNT_MID_THRESHOLD   ((int32_t)500)
#define TASK5_GOTO_NEAR_BOOST_ABS        ((int16_t)20)
#define TASK5_GOTO_NEAR_PWM_LIMIT_ABS    ((int16_t)340)

static const tb6612_motor_t s_task5_motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    TB6612_MOTOR_1,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4
};

/*
 * 靠近目标点时，不同电机至少要给到最小有效 PWM，否则会出现“有误差但电机不转”的卡滞。
 * 如果后续发现某一路仍然偏弱，单独调大该路即可。
 */
static const int16_t s_task5_motor_near_pwm_abs[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    300,
    260,
    280,
    260
};

/*
 * 蛇形路径仅保留拐点/端点，按以下顺序运行：
 * 1.LT -> 2.RT -> 3.(20,10) -> 4.(-20,10) -> 5.(-20,0)
 * -> 6.(20,0) -> 7.(20,-10) -> 8.(-20,-10) -> 9.LB -> 10.RB
 *
 * 已知四个角点沿用 TASK2 的实测值。
 * 其余拐点先通过串口打印实测编码值，再手动填回下面数组。
 */
static const task5_waypoint_t s_task5_waypoints[(uint8_t)TASK5_POINT_COUNT] =
{
    { -20,  20, { 4742, -2230, -6082, -2091 }, 1U }, /* 1 LT */
    {  20,  20, { -2111, -6386, -1709, 5640 }, 1U }, /* 2 RT */
    {  20,  10, { -2343, -4616, 343, 4434 }, 1U },   /* 3 */
    { -20,  10, { 3432, -498, -4507, -2260 }, 1U },  /* 4 */
    { -20,   0, { 1884, 1626, -3430, -3441 }, 1U },  /* 5 */
    {  20,   0, { -3290, -3593, 1205, 2223 }, 1U },  /* 6 */
    {  20, -10, { -4302, -2399, 2836, 268 }, 1U },   /* 7 */
    { -20, -10, { -6, 3622, -2359, -4471 }, 1U },    /* 8 */
    { -20, -20, { -2004, 6081, -1670, -6413 }, 1U }, /* 9 LB */
    {  20, -20, { -6316, -623, 4675, -1865 }, 1U }   /* 10 RB */
};

static app_platform_drive_t *s_task5_drive = NULL;
static task5_state_t s_task5_state = TASK5_STATE_IDLE;
static task5_run_mode_t s_task5_run_mode = TASK5_RUN_MODE_NONE;
static task5_point_id_t s_task5_target_point = TASK5_POINT_LEFT_TOP;
static uint8_t s_task5_center_zero_ready = 0U;
static uint8_t s_task5_use_custom_target = 0U;
static int32_t s_task5_custom_target_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT] = { 0 };

static uint8_t Task5_IsValidPointId(task5_point_id_t point_id)
{
    return (uint8_t)((point_id >= TASK5_POINT_LEFT_TOP) &&
                     (point_id <= TASK5_POINT_RIGHT_BOTTOM));
}

static uint8_t Task5_PointToIndex(task5_point_id_t point_id)
{
    return (uint8_t)(point_id - 1U);
}

static uint8_t Task5_StartMoveInternal(task5_point_id_t point_id, task5_run_mode_t run_mode)
{
    if ((s_task5_drive == NULL) || (Task5_IsValidPointId(point_id) == 0U))
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    if (s_task5_center_zero_ready == 0U)
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    if (Task5_GetTargetCounts(point_id, s_task5_custom_target_counts) == 0U)
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    (void)AppPlatformDrive_Stop(s_task5_drive);
    s_task5_target_point = point_id;
    s_task5_run_mode = run_mode;
    s_task5_use_custom_target = 0U;
    s_task5_state = TASK5_STATE_RUNNING;
    return 1U;
}

static int16_t Task5_GetNearPwmWithPositionBoost(uint32_t motor_index)
{
    const rope_platform_solver_t *solver;
    int16_t pwm_abs;

    if (motor_index >= ROPE_PLATFORM_SOLVER_CABLE_COUNT)
    {
        return 0;
    }

    pwm_abs = s_task5_motor_near_pwm_abs[motor_index];
    solver = AppPlatformDrive_GetSolver(s_task5_drive);

    if ((solver == NULL) || (solver->state_valid == 0U))
    {
        return pwm_abs;
    }

    /* 电机角点映射：M1=LT, M2=LB, M3=RB, M4=RT */
    if ((solver->position_x_m <= -0.08f) && (solver->position_y_m >= 0.08f) && (motor_index == 0U))
    {
        pwm_abs = (int16_t)(pwm_abs + TASK5_GOTO_NEAR_BOOST_ABS);
    }
    else if ((solver->position_x_m <= -0.08f) && (solver->position_y_m <= -0.08f) && (motor_index == 1U))
    {
        pwm_abs = (int16_t)(pwm_abs + TASK5_GOTO_NEAR_BOOST_ABS);
    }
    else if ((solver->position_x_m >= 0.08f) && (solver->position_y_m <= -0.08f) && (motor_index == 2U))
    {
        pwm_abs = (int16_t)(pwm_abs + TASK5_GOTO_NEAR_BOOST_ABS);
    }
    else if ((solver->position_x_m >= 0.08f) && (solver->position_y_m >= 0.08f) && (motor_index == 3U))
    {
        pwm_abs = (int16_t)(pwm_abs + TASK5_GOTO_NEAR_BOOST_ABS);
    }

    if (pwm_abs > TASK5_GOTO_NEAR_PWM_LIMIT_ABS)
    {
        pwm_abs = TASK5_GOTO_NEAR_PWM_LIMIT_ABS;
    }

    return pwm_abs;
}

static int16_t Task5_ErrorToPwm(uint32_t motor_index, int32_t count_error)
{
    int32_t abs_error;
    int16_t pwm_abs;

    if (count_error == 0)
    {
        return 0;
    }

    abs_error = (count_error > 0) ? count_error : -count_error;
    if (abs_error <= TASK5_GOTO_COUNT_TOLERANCE)
    {
        return 0;
    }

    if (motor_index >= ROPE_PLATFORM_SOLVER_CABLE_COUNT)
    {
        return 0;
    }

    if (abs_error <= TASK5_GOTO_COUNT_NEAR_THRESHOLD)
    {
        pwm_abs = Task5_GetNearPwmWithPositionBoost(motor_index);
    }
    else if (abs_error <= TASK5_GOTO_COUNT_MID_THRESHOLD)
    {
        pwm_abs = TASK5_GOTO_PWM_MID_ABS;
    }
    else
    {
        pwm_abs = TASK5_GOTO_PWM_FAST_ABS;
    }

    return (count_error > 0) ? pwm_abs : (int16_t)(-pwm_abs);
}

void Task5_Init(app_platform_drive_t *drive)
{
    s_task5_drive = drive;
    Task5_Reset();
}

void Task5_Reset(void)
{
    s_task5_state = TASK5_STATE_IDLE;
    s_task5_run_mode = TASK5_RUN_MODE_NONE;
    s_task5_target_point = TASK5_POINT_LEFT_TOP;
}

uint8_t Task5_SetCenterZero(void)
{
    if (s_task5_drive == NULL)
    {
        s_task5_state = TASK5_STATE_ERROR;
        s_task5_center_zero_ready = 0U;
        return 0U;
    }

    if (AppPlatformDrive_SetCurrentPoseAsOrigin(s_task5_drive, 0.0f, 0.0f) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        s_task5_state = TASK5_STATE_ERROR;
        s_task5_center_zero_ready = 0U;
        return 0U;
    }

    s_task5_center_zero_ready = 1U;
    s_task5_state = TASK5_STATE_IDLE;
    return 1U;
}

uint8_t Task5_IsCenterZeroReady(void)
{
    return s_task5_center_zero_ready;
}

uint8_t Task5_StartGoto(task5_point_id_t point_id)
{
    return Task5_StartMoveInternal(point_id, TASK5_RUN_MODE_GOTO_ONE);
}

uint8_t Task5_StartGotoCounts(const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    if ((s_task5_drive == NULL) || (counts == NULL))
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    if (s_task5_center_zero_ready == 0U)
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    (void)memcpy(s_task5_custom_target_counts,
                 counts,
                 sizeof(s_task5_custom_target_counts));

    (void)AppPlatformDrive_Stop(s_task5_drive);
    s_task5_run_mode = TASK5_RUN_MODE_GOTO_ONE;
    s_task5_use_custom_target = 1U;
    s_task5_state = TASK5_STATE_RUNNING;
    return 1U;
}

uint8_t Task5_StartPatrol(void)
{
    if ((s_task5_drive == NULL) || (Task5_IsPathReady() == 0U))
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    return Task5_StartPatrolFrom(TASK5_POINT_LEFT_TOP);
}

uint8_t Task5_StartPatrolFrom(task5_point_id_t point_id)
{
    if ((s_task5_drive == NULL) || (Task5_IsPathReady() == 0U))
    {
        s_task5_state = TASK5_STATE_ERROR;
        return 0U;
    }

    return Task5_StartMoveInternal(point_id, TASK5_RUN_MODE_PATROL);
}

void TASK5(void)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int32_t target_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint32_t index;
    uint8_t all_arrived = 1U;

    if ((s_task5_drive == NULL) || (s_task5_state != TASK5_STATE_RUNNING))
    {
        return;
    }

    if (s_task5_use_custom_target != 0U)
    {
        (void)memcpy(target_counts,
                     s_task5_custom_target_counts,
                     sizeof(target_counts));
    }
    else if (Task5_GetTargetCounts(s_task5_target_point, target_counts) == 0U)
    {
        (void)AppPlatformDrive_Stop(s_task5_drive);
        s_task5_state = TASK5_STATE_ERROR;
        return;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task5_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)AppPlatformDrive_Stop(s_task5_drive);
        s_task5_state = TASK5_STATE_ERROR;
        return;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t error = target_counts[index] - counts[index];
        const int16_t pwm = Task5_ErrorToPwm(index, error);

        if (pwm != 0)
        {
            all_arrived = 0U;
        }

        (void)TB6612_SetMotorSpeed(s_task5_motor_map[index], pwm);
    }

    (void)AppPlatformDrive_UpdateStateOnly(s_task5_drive, 0.010f);

    if (all_arrived == 0U)
    {
        return;
    }

    (void)AppPlatformDrive_Stop(s_task5_drive);

    if (s_task5_run_mode == TASK5_RUN_MODE_GOTO_ONE)
    {
        s_task5_state = TASK5_STATE_FINISHED;
        return;
    }

    s_task5_state = TASK5_STATE_FINISHED;
}

void Task5_Stop(void)
{
    if (s_task5_drive != NULL)
    {
        (void)AppPlatformDrive_Stop(s_task5_drive);
    }

    s_task5_state = TASK5_STATE_IDLE;
    s_task5_run_mode = TASK5_RUN_MODE_NONE;
    s_task5_use_custom_target = 0U;
}

task5_state_t Task5_GetState(void)
{
    return s_task5_state;
}

task5_run_mode_t Task5_GetRunMode(void)
{
    return s_task5_run_mode;
}

uint8_t Task5_IsBusy(void)
{
    return (uint8_t)(s_task5_state == TASK5_STATE_RUNNING);
}

uint8_t Task5_IsFinished(void)
{
    return (uint8_t)(s_task5_state == TASK5_STATE_FINISHED);
}

task5_point_id_t Task5_GetCurrentTarget(void)
{
    return s_task5_target_point;
}

uint8_t Task5_IsPathReady(void)
{
    uint8_t index;

    for (index = 0U; index < (uint8_t)TASK5_POINT_COUNT; index++)
    {
        if (s_task5_waypoints[index].valid == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t Task5_PrintCalibrationPoint(task5_point_id_t point_id)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    const uint8_t index = Task5_PointToIndex(point_id);

    if ((s_task5_drive == NULL) || (Task5_IsValidPointId(point_id) == 0U))
    {
        return 0U;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task5_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return 0U;
    }

    (void)ScreenUart_Printf("task5_point[%u] = { %ld, %ld, %ld, %ld }; /* xy=(%d,%d) valid=1 */\r\n",
                            (unsigned int)point_id,
                            (long)counts[0],
                            (long)counts[1],
                            (long)counts[2],
                            (long)counts[3],
                            (int)s_task5_waypoints[index].x_cm,
                            (int)s_task5_waypoints[index].y_cm);
    return 1U;
}

uint8_t Task5_GetWaypoint(task5_point_id_t point_id, task5_waypoint_t *waypoint)
{
    if ((waypoint == NULL) || (Task5_IsValidPointId(point_id) == 0U))
    {
        return 0U;
    }

    (void)memcpy(waypoint,
                 &s_task5_waypoints[Task5_PointToIndex(point_id)],
                 sizeof(task5_waypoint_t));
    return 1U;
}

uint8_t Task5_GetTargetCounts(task5_point_id_t point_id,
                              int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    const uint8_t index = Task5_PointToIndex(point_id);

    if ((counts == NULL) || (Task5_IsValidPointId(point_id) == 0U))
    {
        return 0U;
    }

    if (s_task5_waypoints[index].valid == 0U)
    {
        return 0U;
    }

    (void)memcpy(counts,
                 s_task5_waypoints[index].counts,
                 sizeof(s_task5_waypoints[index].counts));
    return 1U;
}
