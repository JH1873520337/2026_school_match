#include "task3.h"

#include "TB6612.h"
#include "task2.h"

#include <stddef.h>
#include <string.h>

#define TASK3_GOTO_PWM_ABS              ((int16_t)360)
#define TASK3_GOTO_PWM_SLOW_ABS         ((int16_t)260)
#define TASK3_GOTO_COUNT_TOLERANCE      ((int32_t)8)
#define TASK3_GOTO_COUNT_NEAR_THRESHOLD ((int32_t)80)

static const tb6612_motor_t s_task3_motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
{
    TB6612_MOTOR_1,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4
};

static app_platform_drive_t *s_task3_drive = NULL;
static task3_state_t s_task3_state = TASK3_STATE_IDLE;
static task3_point_id_t s_task3_target_point = TASK3_POINT_CENTER;
static uint8_t s_task3_center_zero_ready = 0U;

static int16_t Task3_ErrorToPwm(int32_t count_error)
{
    int32_t abs_error;

    if (count_error == 0)
    {
        return 0;
    }

    abs_error = (count_error > 0) ? count_error : -count_error;
    if (abs_error <= TASK3_GOTO_COUNT_TOLERANCE)
    {
        return 0;
    }

    if (abs_error <= TASK3_GOTO_COUNT_NEAR_THRESHOLD)
    {
        return (count_error > 0) ? TASK3_GOTO_PWM_SLOW_ABS : -TASK3_GOTO_PWM_SLOW_ABS;
    }

    return (count_error > 0) ? TASK3_GOTO_PWM_ABS : -TASK3_GOTO_PWM_ABS;
}

void Task3_Init(app_platform_drive_t *drive)
{
    s_task3_drive = drive;
    Task3_Reset();
}

void Task3_Reset(void)
{
    s_task3_state = TASK3_STATE_IDLE;
    s_task3_target_point = TASK3_POINT_CENTER;
}

uint8_t Task3_SetCenterZero(void)
{
    if (s_task3_drive == NULL)
    {
        s_task3_state = TASK3_STATE_ERROR;
        s_task3_center_zero_ready = 0U;
        return 0U;
    }

    if (AppPlatformDrive_SetCurrentPoseAsOrigin(s_task3_drive, 0.0f, 0.0f) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        s_task3_state = TASK3_STATE_ERROR;
        s_task3_center_zero_ready = 0U;
        return 0U;
    }

    s_task3_center_zero_ready = 1U;
    s_task3_state = TASK3_STATE_IDLE;
    return 1U;
}

uint8_t Task3_IsCenterZeroReady(void)
{
    return s_task3_center_zero_ready;
}

uint8_t Task3_StartGoto(task3_point_id_t point_id)
{
    int32_t target_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];

    if ((s_task3_drive == NULL) ||
        (point_id < TASK3_POINT_LEFT_TOP) ||
        (point_id > TASK3_POINT_CENTER))
    {
        s_task3_state = TASK3_STATE_ERROR;
        return 0U;
    }

    if (s_task3_center_zero_ready == 0U)
    {
        s_task3_state = TASK3_STATE_ERROR;
        return 0U;
    }

    if (Task3_GetTargetCounts(point_id, target_counts) == 0U)
    {
        s_task3_state = TASK3_STATE_ERROR;
        return 0U;
    }

    (void)AppPlatformDrive_Stop(s_task3_drive);
    s_task3_target_point = point_id;
    s_task3_state = TASK3_STATE_RUNNING;
    return 1U;
}

void TASK3(void)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    int32_t target_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint32_t index;
    uint8_t all_arrived = 1U;

    if ((s_task3_drive == NULL) || (s_task3_state != TASK3_STATE_RUNNING))
    {
        return;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task3_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)AppPlatformDrive_Stop(s_task3_drive);
        s_task3_state = TASK3_STATE_ERROR;
        return;
    }

    if (Task3_GetTargetCounts(s_task3_target_point, target_counts) == 0U)
    {
        (void)AppPlatformDrive_Stop(s_task3_drive);
        s_task3_state = TASK3_STATE_ERROR;
        return;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t error = target_counts[index] - counts[index];
        const int16_t pwm = Task3_ErrorToPwm(error);

        if (pwm != 0)
        {
            all_arrived = 0U;
        }

        (void)TB6612_SetMotorSpeed(s_task3_motor_map[index], pwm);
    }

    (void)AppPlatformDrive_UpdateStateOnly(s_task3_drive, 0.010f);

    if (all_arrived == 0U)
    {
        return;
    }

    (void)AppPlatformDrive_Stop(s_task3_drive);
    s_task3_state = TASK3_STATE_FINISHED;
}

void Task3_Stop(void)
{
    if (s_task3_drive != NULL)
    {
        (void)AppPlatformDrive_Stop(s_task3_drive);
    }

    s_task3_state = TASK3_STATE_IDLE;
}

task3_state_t Task3_GetState(void)
{
    return s_task3_state;
}

uint8_t Task3_IsBusy(void)
{
    return (uint8_t)(s_task3_state == TASK3_STATE_RUNNING);
}

uint8_t Task3_IsFinished(void)
{
    return (uint8_t)(s_task3_state == TASK3_STATE_FINISHED);
}

task3_point_id_t Task3_GetCurrentTarget(void)
{
    return s_task3_target_point;
}

uint8_t Task3_GetTargetCounts(task3_point_id_t point_id,
                              int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    int32_t fixed_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];

    if ((counts == NULL) ||
        (point_id < TASK3_POINT_LEFT_TOP) ||
        (point_id > TASK3_POINT_CENTER))
    {
        return 0U;
    }

    if (point_id == TASK3_POINT_CENTER)
    {
        (void)memset(counts, 0, sizeof(int32_t) * ROPE_PLATFORM_SOLVER_CABLE_COUNT);
        return 1U;
    }

    if (Task2_GetFixedPointCounts((task2_point_id_t)(point_id - 1U), fixed_counts) == 0U)
    {
        return 0U;
    }

    (void)memcpy(counts, fixed_counts, sizeof(fixed_counts));
    return 1U;
}
