#include "task2.h"

#include "TB6612.h"

#include <stddef.h>
#include <string.h>

#define TASK2_GOTO_PWM_ABS              ((int16_t)360)
#define TASK2_GOTO_PWM_SLOW_ABS         ((int16_t)260)
#define TASK2_GOTO_COUNT_TOLERANCE      ((int32_t)8)
#define TASK2_GOTO_COUNT_NEAR_THRESHOLD ((int32_t)80)

typedef struct
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint8_t valid;
} task2_point_t;

static app_platform_drive_t *s_task2_drive = NULL;
static task2_point_t s_task2_points[TASK2_POINT_COUNT];
static uint8_t s_task2_busy = 0U;
static uint8_t s_task2_target_index = 0U;

static uint8_t Task2_IsValidPointIndex(uint8_t point_index)
{
    return (uint8_t)((point_index >= 1U) && (point_index <= TASK2_POINT_COUNT));
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

void Task2_Init(app_platform_drive_t *drive)
{
    s_task2_drive = drive;
    s_task2_busy = 0U;
    s_task2_target_index = 0U;
    (void)memset(s_task2_points, 0, sizeof(s_task2_points));
}

uint8_t Task2_SavePoint(uint8_t point_index)
{
    if ((s_task2_drive == NULL) || (Task2_IsValidPointIndex(point_index) == 0U))
    {
        return 0U;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task2_drive,
                                          s_task2_points[point_index - 1U].counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return 0U;
    }

    s_task2_points[point_index - 1U].valid = 1U;
    return 1U;
}

uint8_t Task2_StartGoto(uint8_t point_index)
{
    if ((s_task2_drive == NULL) || (Task2_IsValidPointIndex(point_index) == 0U))
    {
        return 0U;
    }

    if (s_task2_points[point_index - 1U].valid == 0U)
    {
        return 0U;
    }

    (void)AppPlatformDrive_Stop(s_task2_drive);
    s_task2_target_index = point_index - 1U;
    s_task2_busy = 1U;
    return 1U;
}

void TASK2(void)
{
    static const tb6612_motor_t motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
    {
        TB6612_MOTOR_1,
        TB6612_MOTOR_2,
        TB6612_MOTOR_3,
        TB6612_MOTOR_4
    };
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint8_t all_arrived = 1U;
    uint32_t index;

    if ((s_task2_drive == NULL) || (s_task2_busy == 0U))
    {
        return;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task2_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)AppPlatformDrive_Stop(s_task2_drive);
        s_task2_busy = 0U;
        return;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t error = s_task2_points[s_task2_target_index].counts[index] - counts[index];
        const int16_t pwm = Task2_ErrorToPwm(error);

        if (pwm != 0)
        {
            all_arrived = 0U;
        }

        (void)TB6612_SetMotorSpeed(motor_map[index], pwm);
    }

    (void)AppPlatformDrive_UpdateStateOnly(s_task2_drive, 0.010f);

    if (all_arrived == 0U)
    {
        return;
    }

    (void)AppPlatformDrive_Stop(s_task2_drive);
    s_task2_busy = 0U;
}

void Task2_Stop(void)
{
    if (s_task2_drive != NULL)
    {
        (void)AppPlatformDrive_Stop(s_task2_drive);
    }

    s_task2_busy = 0U;
}

uint8_t Task2_IsBusy(void)
{
    return s_task2_busy;
}

uint8_t Task2_HasPoint(uint8_t point_index)
{
    if (Task2_IsValidPointIndex(point_index) == 0U)
    {
        return 0U;
    }

    return s_task2_points[point_index - 1U].valid;
}

uint8_t Task2_GetPointCounts(uint8_t point_index,
                             int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT])
{
    if ((counts == NULL) || (Task2_IsValidPointIndex(point_index) == 0U))
    {
        return 0U;
    }

    if (s_task2_points[point_index - 1U].valid == 0U)
    {
        return 0U;
    }

    (void)memcpy(counts,
                 s_task2_points[point_index - 1U].counts,
                 sizeof(s_task2_points[point_index - 1U].counts));
    return 1U;
}
