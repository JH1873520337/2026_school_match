#include "task1.h"

#include "TB6612.h"
#include <stddef.h>
#include <string.h>

#define TASK1_HOME_PWM_ABS              ((int16_t)360)
#define TASK1_HOME_PWM_SLOW_ABS         ((int16_t)260)
#define TASK1_HOME_COUNT_TOLERANCE      ((int32_t)8)
#define TASK1_HOME_COUNT_NEAR_THRESHOLD ((int32_t)80)

typedef enum
{
    TASK1_STATE_IDLE = 0,
    TASK1_STATE_HOMING
} task1_state_t;

static app_platform_drive_t *s_task1_drive = NULL;
static task1_state_t s_task1_state = TASK1_STATE_IDLE;
static int32_t s_task1_origin_counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
static uint8_t s_task1_origin_valid = 0U;

static int16_t Task1_ErrorToPwm(int32_t count_error)
{
    int32_t abs_error;

    if (count_error == 0)
    {
        return 0;
    }

    abs_error = (count_error > 0) ? count_error : -count_error;
    if (abs_error <= TASK1_HOME_COUNT_TOLERANCE)
    {
        return 0;
    }

    if (abs_error <= TASK1_HOME_COUNT_NEAR_THRESHOLD)
    {
        return (count_error > 0) ? TASK1_HOME_PWM_SLOW_ABS : -TASK1_HOME_PWM_SLOW_ABS;
    }

    return (count_error > 0) ? TASK1_HOME_PWM_ABS : -TASK1_HOME_PWM_ABS;
}

void Task1_Init(app_platform_drive_t *drive)
{
    s_task1_drive = drive;
    s_task1_state = TASK1_STATE_IDLE;
    s_task1_origin_valid = 0U;
    (void)memset(s_task1_origin_counts, 0, sizeof(s_task1_origin_counts));
}

app_platform_drive_status_t Task1_SetCenterOrigin(void)
{
    app_platform_drive_status_t status;

    if (s_task1_drive == NULL)
    {
        return APP_PLATFORM_DRIVE_STATUS_NOT_INIT;
    }

    status = AppPlatformDrive_SetCurrentPoseAsOrigin(s_task1_drive, 0.0f, 0.0f);
    if (status != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        s_task1_origin_valid = 0U;
        return status;
    }

    status = AppPlatformDrive_GetCurrentCounts(s_task1_drive, s_task1_origin_counts);
    if (status != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        s_task1_origin_valid = 0U;
        return status;
    }

    s_task1_origin_valid = 1U;
    return APP_PLATFORM_DRIVE_STATUS_OK;
}

uint8_t Task1_Start(void)
{
    if ((s_task1_drive == NULL) || (s_task1_origin_valid == 0U))
    {
        return 0U;
    }

    (void)AppPlatformDrive_Stop(s_task1_drive);
    s_task1_state = TASK1_STATE_HOMING;
    return 1U;
}

void TASK1(void)
{
    static const tb6612_motor_t motor_map[ROPE_PLATFORM_SOLVER_CABLE_COUNT] =
    {
        TB6612_MOTOR_1,
        TB6612_MOTOR_2,
        TB6612_MOTOR_3,
        TB6612_MOTOR_4
    };
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint32_t index;
    uint8_t all_arrived = 1U;

    if ((s_task1_drive == NULL) || (s_task1_origin_valid == 0U))
    {
        return;
    }

    if (s_task1_state == TASK1_STATE_IDLE)
    {
        return;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task1_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        (void)AppPlatformDrive_Stop(s_task1_drive);
        s_task1_state = TASK1_STATE_IDLE;
        return;
    }

    for (index = 0U; index < ROPE_PLATFORM_SOLVER_CABLE_COUNT; index++)
    {
        const int32_t error = s_task1_origin_counts[index] - counts[index];
        const int16_t pwm = Task1_ErrorToPwm(error);

        if (pwm != 0)
        {
            all_arrived = 0U;
        }

        (void)TB6612_SetMotorSpeed(motor_map[index], pwm);
    }

    (void)AppPlatformDrive_UpdateStateOnly(s_task1_drive, 0.010f);

    if (all_arrived == 0U)
    {
        return;
    }

    (void)AppPlatformDrive_Stop(s_task1_drive);
    (void)Task1_SetCenterOrigin();
    s_task1_state = TASK1_STATE_IDLE;
}

void Task1_Stop(void)
{
    if (s_task1_drive != NULL)
    {
        (void)AppPlatformDrive_Stop(s_task1_drive);
    }

    s_task1_state = TASK1_STATE_IDLE;
}

uint8_t Task1_IsBusy(void)
{
    return (s_task1_state == TASK1_STATE_IDLE) ? 0U : 1U;
}
