#ifndef TASK1_H
#define TASK1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "app_platform_drive.h"

#include <stdint.h>

void Task1_Init(app_platform_drive_t *drive);
app_platform_drive_status_t Task1_SetCenterOrigin(void);
uint8_t Task1_Start(void);
void TASK1(void);
void Task1_Stop(void);
uint8_t Task1_IsBusy(void);

#ifdef __cplusplus
}
#endif

#endif
