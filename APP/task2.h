#ifndef TASK2_H
#define TASK2_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"

#include <stdint.h>

#define TASK2_POINT_COUNT   ((uint8_t)5U)

void Task2_Init(app_platform_drive_t *drive);
uint8_t Task2_SavePoint(uint8_t point_index);
uint8_t Task2_StartGoto(uint8_t point_index);
void TASK2(void);
void Task2_Stop(void);
uint8_t Task2_IsBusy(void);
uint8_t Task2_HasPoint(uint8_t point_index);
uint8_t Task2_GetPointCounts(uint8_t point_index,
                             int32_t counts[ROPE_PLATFORM_SOLVER_CABLE_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
