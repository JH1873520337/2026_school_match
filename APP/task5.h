#ifndef TASK5_H
#define TASK5_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"

#include <stdint.h>

typedef enum
{
    TASK5_POINT_LEFT_TOP = 1,
    TASK5_POINT_RIGHT_TOP,
    TASK5_POINT_RIGHT_Y_POS_10,
    TASK5_POINT_LEFT_Y_POS_10,
    TASK5_POINT_LEFT_CENTER_ROW,
    TASK5_POINT_RIGHT_CENTER_ROW,
    TASK5_POINT_RIGHT_Y_NEG_10,
    TASK5_POINT_LEFT_Y_NEG_10,
    TASK5_POINT_LEFT_BOTTOM,
    TASK5_POINT_RIGHT_BOTTOM,
    TASK5_POINT_COUNT = 10
} task5_point_id_t;

typedef enum
{
    TASK5_STATE_IDLE = 0,
    TASK5_STATE_RUNNING,
    TASK5_STATE_FINISHED,
    TASK5_STATE_ERROR
} task5_state_t;

typedef enum
{
    TASK5_RUN_MODE_NONE = 0,
    TASK5_RUN_MODE_GOTO_ONE,
    TASK5_RUN_MODE_PATROL
} task5_run_mode_t;

typedef struct
{
    int16_t x_cm;
    int16_t y_cm;
    int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT];
    uint8_t valid;
} task5_waypoint_t;

void Task5_Init(app_platform_drive_t *drive);
void Task5_Reset(void);
uint8_t Task5_SetCenterZero(void);
uint8_t Task5_IsCenterZeroReady(void);

uint8_t Task5_StartGoto(task5_point_id_t point_id);
uint8_t Task5_StartGotoCounts(const int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);
uint8_t Task5_StartPatrol(void);
uint8_t Task5_StartPatrolFrom(task5_point_id_t point_id);
void TASK5(void);
void Task5_Stop(void);

task5_state_t Task5_GetState(void);
task5_run_mode_t Task5_GetRunMode(void);
uint8_t Task5_IsBusy(void);
uint8_t Task5_IsFinished(void);
task5_point_id_t Task5_GetCurrentTarget(void);

uint8_t Task5_IsPathReady(void);
uint8_t Task5_PrintCalibrationPoint(task5_point_id_t point_id);
uint8_t Task5_GetWaypoint(task5_point_id_t point_id, task5_waypoint_t *waypoint);
uint8_t Task5_GetTargetCounts(task5_point_id_t point_id,
                              int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
