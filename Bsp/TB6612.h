#ifndef TB6612_H
#define TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#include <stdint.h>

#define TB6612_SPEED_MAX   ((int16_t)1000)

typedef enum
{
    TB6612_STATUS_OK = 0,
    TB6612_STATUS_ERROR,
    TB6612_STATUS_BAD_PARAM,
    TB6612_STATUS_NOT_INIT
} tb6612_status_t;

typedef enum
{
    TB6612_MOTOR_1 = 0,
    TB6612_MOTOR_2,
    TB6612_MOTOR_3,
    TB6612_MOTOR_4,
    TB6612_MOTOR_COUNT
} tb6612_motor_t;

tb6612_status_t TB6612_Init(void);
uint8_t TB6612_IsReady(void);
tb6612_status_t TB6612_SetMotorSpeed(tb6612_motor_t motor, int16_t speed);
tb6612_status_t TB6612_StopMotor(tb6612_motor_t motor);
tb6612_status_t TB6612_BrakeMotor(tb6612_motor_t motor);
tb6612_status_t TB6612_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif
