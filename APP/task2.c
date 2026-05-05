#include "task2.h"

#include "TB6612.h"
#include "screen_uart.h"

#include <stddef.h>
#include <string.h>

#define TASK2_GOTO_PWM_ABS              ((int16_t)360)
#define TASK2_GOTO_PWM_SLOW_ABS         ((int16_t)260)
#define TASK2_GOTO_COUNT_TOLERANCE      ((int32_t)8)
#define TASK2_GOTO_COUNT_NEAR_THRESHOLD ((int32_t)80)

typedef struct
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
} task2_fixed_point_t;

static const task2_fixed_point_t s_task2_fixed_points[TASK2_POINT_COUNT] =
{
    { { 5367, -2061, -6074, -1950 } }, /* LT */
    { { -1953, -6180, -1686, 6158 } }, /* RT */
    { { -5875, -1295, 5255, -1218 } }, /* RB */
    { { -1542, 6308, -1730, -5878 } }  /* LB */
};

static const tb6612_motor_t s_task2_motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    TB6612_MOTOR_1,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4
};

static app_platform_drive_t *s_task2_drive = NULL;
static task2_state_t s_task2_state = TASK2_STATE_IDLE;
static task2_point_id_t s_task2_target_point = TASK2_POINT_LEFT_TOP;
static uint8_t s_task2_center_zero_ready = 0U;

static uint8_t Task2_IsValidPointId(task2_point_id_t point_id)
{
    return (uint8_t)(point_id < TASK2_POINT_COUNT);
}

static int16_t Task2_ErrorToPwm(int32_t count_error)
{
    int32_t abs_error;

    if (count_error == 0)
    {
        return 0;
    }

    abs_error = (count_error > 0) ? count_error : -count_error;
    if (abs_error <= TASK2_GOTO_COUNT_TOLERANCE)
    {
        return 0;
    }

    if (abs_error <= TASK2_GOTO_COUNT_NEAR_THRESHOLD)
    {
        return (count_error > 0) ? TASK2_GOTO_PWM_SLOW_ABS : -TASK2_GOTO_PWM_SLOW_ABS;
    }

    return (count_error > 0) ? TASK2_GOTO_PWM_ABS : -TASK2_GOTO_PWM_ABS;
}

static void Task2_AdvanceTarget(void)
{
    if (s_task2_target_point < TASK2_POINT_LEFT_BOTTOM)
    {
        s_task2_target_point = (task2_point_id_t)(s_task2_target_point + 1);
    }
    else
    {
        s_task2_state = TASK2_STATE_FINISHED;
    }
}

void Task2_Init(app_platform_drive_t *drive)
{
    s_task2_drive = drive;
    Task2_Reset();
}

void Task2_Reset(void)
{
    s_task2_state = TASK2_STATE_IDLE;
    s_task2_target_point = TASK2_POINT_LEFT_TOP;
}

uint8_t Task2_SetCenterZero(void)
{
    if (s_task2_drive == NULL)
    {
        s_task2_state = TASK2_STATE_ERROR;
        s_task2_center_zero_ready = 0U;
        return 0U;
    }

    if (AppPlatformDrive_SetCurrentPoseAsOrigin(s_task2_drive, 0.0f, 0.0f) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        s_task2_state = TASK2_STATE_ERROR;
        s_task2_center_zero_ready = 0U;
        return 0U;
    }

    s_task2_center_zero_ready = 1U;
    s_task2_state = TASK2_STATE_IDLE;
    return 1U;
}

uint8_t Task2_IsCenterZeroReady(void)
{
    return s_task2_center_zero_ready;
}

uint8_t Task2_StartBasicPatrol(void)
{
    if (s_task2_drive == NULL)
    {
        s_task2_state = TASK2_STATE_ERROR;
        return 0U;
    }

    if (s_task2_center_zero_ready == 0U)
    {
        s_task2_state = TASK2_STATE_ERROR;
        return 0U;
    }

    (void)AppPlatformDrive_Stop(s_task2_drive);
    s_task2_target_point = TASK2_POINT_LEFT_TOP;
    s_task2_state = TASK2_STATE_RUNNING;
    return 1U;
}

void TASK2(void)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint32_t index;
    uint8_t all_arrived = 1U;

    if ((s_task2_drive == NULL) || (s_task2_state != TASK2_STATE_RUNNING))
    {
        return;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task2_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)AppPlatformDrive_Stop(s_task2_drive);
        s_task2_state = TASK2_STATE_ERROR;
        return;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t error = s_task2_fixed_points[s_task2_target_point].counts[index] - counts[index];
        const int16_t pwm = Task2_ErrorToPwm(error);

        if (pwm != 0)
        {
            all_arrived = 0U;
        }

        (void)TB6612_SetMotorSpeed(s_task2_motor_map[index], pwm);
    }

    (void)AppPlatformDrive_UpdateStateOnly(s_task2_drive, 0.010f);

    if (all_arrived == 0U)
    {
        return;
    }

    (void)AppPlatformDrive_Stop(s_task2_drive);
    Task2_AdvanceTarget();
}

void Task2_Stop(void)
{
    if (s_task2_drive != NULL)
    {
        (void)AppPlatformDrive_Stop(s_task2_drive);
    }

    s_task2_state = TASK2_STATE_IDLE;
}

task2_state_t Task2_GetState(void)
{
    return s_task2_state;
}

uint8_t Task2_IsBusy(void)
{
    return (uint8_t)(s_task2_state == TASK2_STATE_RUNNING);
}

uint8_t Task2_IsFinished(void)
{
    return (uint8_t)(s_task2_state == TASK2_STATE_FINISHED);
}

task2_point_id_t Task2_GetCurrentTarget(void)
{
    return s_task2_target_point;
}

uint8_t Task2_PrintCalibrationPoint(task2_point_id_t point_id)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];

    if ((s_task2_drive == NULL) || (Task2_IsValidPointId(point_id) == 0U))
    {
        return 0U;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task2_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return 0U;
    }

    (void)ScreenUart_Printf("task2_point[%u] = { %ld, %ld, %ld, %ld }\r\n",
                            (unsigned int)point_id,
                            (long)counts[0],
                            (long)counts[1],
                            (long)counts[2],
                            (long)counts[3]);
    return 1U;
}

uint8_t Task2_GetFixedPointCounts(task2_point_id_t point_id,
                                  int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    if ((counts == NULL) || (Task2_IsValidPointId(point_id) == 0U))
    {
        return 0U;
    }

    (void)memcpy(counts,
                 s_task2_fixed_points[point_id].counts,
                 sizeof(s_task2_fixed_points[point_id].counts));
    return 1U;
}
