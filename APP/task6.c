#include "task6.h"

#include "alarm_service.h"
#include "screen_uart.h"

#include <string.h>

typedef enum
{
    TASK6_STEP_PATH_POINT = 0,
    TASK6_STEP_FIRE_POINT
} task6_step_type_t;

typedef struct
{
    task6_step_type_t type;
    uint8_t id;
} task6_step_t;

static const task6_fire_point_t s_task6_fire_points[TASK6_FIRE_POINT_COUNT] =
{
    { { 2974, -779, -3501, -1322 }, 1U }, /* fire 1 */
    { { -3488, -701, 2613, -1323 }, 1U }  /* fire 2 */
};

/*
 * 固定火源版本路径：
 * p1 -> p2 -> p3 -> fire1 -> p4 -> p5 -> p6 -> p7 -> fire2 -> p8 -> p9 -> p10
 */
static const task6_step_t s_task6_steps[] =
{
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_LEFT_TOP },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_RIGHT_TOP },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_RIGHT_Y_POS_10 },
    { TASK6_STEP_FIRE_POINT, 1U },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_LEFT_Y_POS_10 },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_LEFT_CENTER_ROW },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_RIGHT_CENTER_ROW },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_RIGHT_Y_NEG_10 },
    { TASK6_STEP_FIRE_POINT, 2U },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_LEFT_Y_NEG_10 },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_LEFT_BOTTOM },
    { TASK6_STEP_PATH_POINT, (uint8_t)TASK5_POINT_RIGHT_BOTTOM }
};

static app_platform_drive_t *s_task6_drive = NULL;
static task6_state_t s_task6_state = TASK6_STATE_IDLE;
static uint8_t s_task6_step_index = 0U;
static uint8_t s_task6_alarm_fire_id = 0U;

static uint8_t Task6_IsValidFirePointId(uint8_t fire_point_id)
{
    return (uint8_t)((fire_point_id >= 1U) && (fire_point_id <= TASK6_FIRE_POINT_COUNT));
}

static uint8_t Task6_StartCurrentStep(void)
{
    const task6_step_t *step;

    if (s_task6_step_index >= (uint8_t)(sizeof(s_task6_steps) / sizeof(s_task6_steps[0])))
    {
        s_task6_state = TASK6_STATE_FINISHED;
        return 0U;
    }

    step = &s_task6_steps[s_task6_step_index];

    if (step->type == TASK6_STEP_PATH_POINT)
    {
        if (Task5_StartGoto((task5_point_id_t)step->id) == 0U)
        {
            s_task6_state = TASK6_STATE_ERROR;
            return 0U;
        }
    }
    else
    {
        if ((Task6_IsValidFirePointId(step->id) == 0U) ||
            (s_task6_fire_points[step->id - 1U].valid == 0U) ||
            (Task5_StartGotoCounts(s_task6_fire_points[step->id - 1U].counts) == 0U))
        {
            s_task6_state = TASK6_STATE_ERROR;
            return 0U;
        }
    }

    s_task6_state = TASK6_STATE_RUNNING;
    return 1U;
}

void Task6_Init(app_platform_drive_t *drive)
{
    s_task6_drive = drive;
    Task5_Init(drive);
    Task6_Reset();
}

void Task6_Reset(void)
{
    s_task6_state = TASK6_STATE_IDLE;
    s_task6_step_index = 0U;
    s_task6_alarm_fire_id = 0U;
    Task5_Reset();
}

uint8_t Task6_SetCenterZero(void)
{
    s_task6_step_index = 0U;
    s_task6_alarm_fire_id = 0U;
    s_task6_state = TASK6_STATE_IDLE;
    return Task5_SetCenterZero();
}

uint8_t Task6_IsCenterZeroReady(void)
{
    return Task5_IsCenterZeroReady();
}

uint8_t Task6_StartPatrol(void)
{
    if ((s_task6_drive == NULL) || (Task5_IsPathReady() == 0U) || (Task6_IsFirePointsReady() == 0U))
    {
        s_task6_state = TASK6_STATE_ERROR;
        return 0U;
    }

    if (Task5_IsCenterZeroReady() == 0U)
    {
        s_task6_state = TASK6_STATE_ERROR;
        return 0U;
    }

    s_task6_step_index = 0U;
    s_task6_alarm_fire_id = 0U;
    AlarmService_Stop();
    Task5_Stop();
    return Task6_StartCurrentStep();
}

void TASK6(void)
{
    const task6_step_t *step;

    AlarmService_Process();

    if ((s_task6_state == TASK6_STATE_IDLE) ||
        (s_task6_state == TASK6_STATE_FINISHED) ||
        (s_task6_state == TASK6_STATE_ERROR))
    {
        return;
    }

    if (s_task6_state == TASK6_STATE_FIRE_ALARM)
    {
        if (AlarmService_IsBusy() != 0U)
        {
            return;
        }

        s_task6_step_index++;
        s_task6_alarm_fire_id = 0U;

        if (s_task6_step_index >= (uint8_t)(sizeof(s_task6_steps) / sizeof(s_task6_steps[0])))
        {
            s_task6_state = TASK6_STATE_FINISHED;
            return;
        }

        (void)Task6_StartCurrentStep();
        return;
    }

    TASK5();

    if (Task5_GetState() == TASK5_STATE_ERROR)
    {
        s_task6_state = TASK6_STATE_ERROR;
        return;
    }

    if (Task5_GetState() != TASK5_STATE_FINISHED)
    {
        return;
    }

    step = &s_task6_steps[s_task6_step_index];

    if (step->type == TASK6_STEP_FIRE_POINT)
    {
        s_task6_alarm_fire_id = step->id;
        AlarmService_PlayFireDetectedPattern();
        s_task6_state = TASK6_STATE_FIRE_ALARM;
        return;
    }

    s_task6_step_index++;
    if (s_task6_step_index >= (uint8_t)(sizeof(s_task6_steps) / sizeof(s_task6_steps[0])))
    {
        s_task6_state = TASK6_STATE_FINISHED;
        return;
    }

    (void)Task6_StartCurrentStep();
}

void Task6_Stop(void)
{
    Task5_Stop();
    AlarmService_Stop();
    s_task6_state = TASK6_STATE_IDLE;
    s_task6_alarm_fire_id = 0U;
}

task6_state_t Task6_GetState(void)
{
    return s_task6_state;
}

uint8_t Task6_IsBusy(void)
{
    return (uint8_t)((s_task6_state == TASK6_STATE_RUNNING) ||
                     (s_task6_state == TASK6_STATE_FIRE_ALARM));
}

uint8_t Task6_IsFinished(void)
{
    return (uint8_t)(s_task6_state == TASK6_STATE_FINISHED);
}

uint8_t Task6_IsFireAlarmActive(void)
{
    return (uint8_t)(s_task6_state == TASK6_STATE_FIRE_ALARM);
}

task5_point_id_t Task6_GetCurrentTarget(void)
{
    const task6_step_t *step;
    int32_t index;

    if (s_task6_step_index >= (uint8_t)(sizeof(s_task6_steps) / sizeof(s_task6_steps[0])))
    {
        return TASK5_POINT_RIGHT_BOTTOM;
    }

    step = &s_task6_steps[s_task6_step_index];
    if (step->type == TASK6_STEP_PATH_POINT)
    {
        return (task5_point_id_t)step->id;
    }

    for (index = (int32_t)s_task6_step_index - 1; index >= 0; index--)
    {
        if (s_task6_steps[index].type == TASK6_STEP_PATH_POINT)
        {
            return (task5_point_id_t)s_task6_steps[index].id;
        }
    }

    return TASK5_POINT_LEFT_TOP;
}

uint8_t Task6_GetCurrentPathIndex(void)
{
    return s_task6_step_index;
}

uint8_t Task6_GetCurrentFireId(void)
{
    const task6_step_t *step;

    if (s_task6_step_index >= (uint8_t)(sizeof(s_task6_steps) / sizeof(s_task6_steps[0])))
    {
        return 0U;
    }

    step = &s_task6_steps[s_task6_step_index];
    if (step->type == TASK6_STEP_FIRE_POINT)
    {
        return step->id;
    }

    return 0U;
}

uint8_t Task6_PrintFireCalibrationPoint(uint8_t fire_point_id)
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];

    if ((s_task6_drive == NULL) || (Task6_IsValidFirePointId(fire_point_id) == 0U))
    {
        return 0U;
    }

    if (AppPlatformDrive_GetCurrentCounts(s_task6_drive, counts) != APP_PLATFORM_DRIVE_STATUS_OK)
    {
        return 0U;
    }

    (void)ScreenUart_Printf("task6_fire_point[%u] = { %ld, %ld, %ld, %ld }; /* valid=1 */\r\n",
                            (unsigned int)fire_point_id,
                            (long)counts[0],
                            (long)counts[1],
                            (long)counts[2],
                            (long)counts[3]);
    (void)ScreenUart_Printf("task6 mark fire%u ok\r\n", (unsigned int)fire_point_id);
    return 1U;
}

uint8_t Task6_GetFirePoint(uint8_t fire_point_id, task6_fire_point_t *fire_point)
{
    if ((fire_point == NULL) || (Task6_IsValidFirePointId(fire_point_id) == 0U))
    {
        return 0U;
    }

    (void)memcpy(fire_point,
                 &s_task6_fire_points[fire_point_id - 1U],
                 sizeof(task6_fire_point_t));
    return 1U;
}

uint8_t Task6_IsFirePointsReady(void)
{
    uint8_t index;

    for (index = 0U; index < TASK6_FIRE_POINT_COUNT; index++)
    {
        if (s_task6_fire_points[index].valid == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}
