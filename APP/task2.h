#ifndef TASK2_H
#define TASK2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"

#include <stdint.h>

typedef enum
{
    TASK2_POINT_LEFT_TOP = 0,
    TASK2_POINT_RIGHT_TOP,
    TASK2_POINT_RIGHT_BOTTOM,
    TASK2_POINT_LEFT_BOTTOM,
    TASK2_POINT_COUNT
} task2_point_id_t;

typedef enum
{
    TASK2_STATE_IDLE = 0,
    TASK2_STATE_RUNNING,
    TASK2_STATE_FINISHED,
    TASK2_STATE_ERROR
} task2_state_t;

void Task2_Init(app_platform_drive_t *drive);
void Task2_Reset(void);
uint8_t Task2_SetCenterZero(void);
uint8_t Task2_IsCenterZeroReady(void);

uint8_t Task2_StartBasicPatrol(void);
void TASK2(void);
void Task2_Stop(void);

task2_state_t Task2_GetState(void);
uint8_t Task2_IsBusy(void);
uint8_t Task2_IsFinished(void);
task2_point_id_t Task2_GetCurrentTarget(void);

uint8_t Task2_PrintCalibrationPoint(task2_point_id_t point_id);
uint8_t Task2_GetFixedPointCounts(task2_point_id_t point_id,
                                  int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
