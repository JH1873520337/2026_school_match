#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#include <stdint.h>

typedef enum
{
    ENCODER_STATUS_OK = 0,
    ENCODER_STATUS_ERROR,
    ENCODER_STATUS_BAD_PARAM,
    ENCODER_STATUS_NOT_INIT
} encoder_status_t;

typedef enum
{
    ENCODER_MOTOR_1 = 0,
    ENCODER_MOTOR_2,
    ENCODER_MOTOR_3,
    ENCODER_MOTOR_4,
    ENCODER_COUNT
} encoder_id_t;

encoder_status_t Encoder_Init(void);
uint8_t Encoder_IsReady(void);
encoder_status_t Encoder_GetCount(encoder_id_t encoder, int32_t *count);
encoder_status_t Encoder_SetCount(encoder_id_t encoder, int32_t count);
encoder_status_t Encoder_ResetCount(encoder_id_t encoder);
encoder_status_t Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif
