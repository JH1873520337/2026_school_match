#ifndef TASK3_H
#define TASK3_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"

#include <stdint.h>

typedef enum
{
    TASK3_POINT_LEFT_TOP = 1,
    TASK3_POINT_RIGHT_TOP,
    TASK3_POINT_RIGHT_BOTTOM,
    TASK3_POINT_LEFT_BOTTOM,
    TASK3_POINT_CENTER
} task3_point_id_t;

typedef enum
{
    TASK3_STATE_IDLE = 0,
    TASK3_STATE_RUNNING,
    TASK3_STATE_FINISHED,
    TASK3_STATE_ERROR
} task3_state_t;

void Task3_Init(app_platform_drive_t *drive);
void Task3_Reset(void);
uint8_t Task3_SetCenterZero(void);
uint8_t Task3_IsCenterZeroReady(void);
uint8_t Task3_StartGoto(task3_point_id_t point_id);
void TASK3(void);
void Task3_Stop(void);
task3_state_t Task3_GetState(void);
uint8_t Task3_IsBusy(void);
uint8_t Task3_IsFinished(void);
task3_point_id_t Task3_GetCurrentTarget(void);
uint8_t Task3_GetTargetCounts(task3_point_id_t point_id,
                              int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
