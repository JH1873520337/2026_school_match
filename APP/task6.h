#ifndef TASK6_H
#define TASK6_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"
#include "task5.h"

#include <stdint.h>

#define TASK6_FIRE_POINT_COUNT   ((uint8_t)2U)

typedef enum
{
    TASK6_STATE_IDLE = 0,
    TASK6_STATE_RUNNING,
    TASK6_STATE_FIRE_ALARM,
    TASK6_STATE_FINISHED,
    TASK6_STATE_ERROR
} task6_state_t;

typedef struct
{
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint8_t valid;
} task6_fire_point_t;

void Task6_Init(app_platform_drive_t *drive);
void Task6_Reset(void);
uint8_t Task6_SetCenterZero(void);
uint8_t Task6_IsCenterZeroReady(void);

uint8_t Task6_StartPatrol(void);
void TASK6(void);
void Task6_Stop(void);

task6_state_t Task6_GetState(void);
uint8_t Task6_IsBusy(void);
uint8_t Task6_IsFinished(void);
uint8_t Task6_IsFireAlarmActive(void);
task5_point_id_t Task6_GetCurrentTarget(void);
uint8_t Task6_GetCurrentPathIndex(void);
uint8_t Task6_GetCurrentFireId(void);

uint8_t Task6_PrintFireCalibrationPoint(uint8_t fire_point_id);
uint8_t Task6_GetFirePoint(uint8_t fire_point_id, task6_fire_point_t *fire_point);
uint8_t Task6_IsFirePointsReady(void);

#ifdef __cplusplus
}
#endif

#endif
