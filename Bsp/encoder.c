#include "encoder.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
} encoder_hw_t;

static const encoder_hw_t s_encoder_hw[ENCODER_COUNT] =
{
    {&htim1},
    {&htim2},
    {&htim4},
    {&htim5}
};

static uint8_t s_encoder_ready = 0U;

static encoder_status_t Encoder_CheckId(encoder_id_t encoder)
{
    if (encoder >= ENCODER_COUNT)
    {
        return ENCODER_STATUS_BAD_PARAM;
    }

    return ENCODER_STATUS_OK;
}

static int32_t Encoder_ReadCounter(const TIM_HandleTypeDef *htim)
{
    uint32_t counter = __HAL_TIM_GET_COUNTER(htim);

    if ((htim->Instance == TIM2) || (htim->Instance == TIM5))
    {
        return (int32_t)counter;
    }

    return (int32_t)((int16_t)counter);
}

static uint32_t Encoder_WriteCounterValue(const TIM_HandleTypeDef *htim, int32_t count)
{
    if ((htim->Instance == TIM2) || (htim->Instance == TIM5))
    {
        return (uint32_t)count;
    }

    return (uint32_t)((uint16_t)count);
}

encoder_status_t Encoder_Init(void)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    if (hal_status != HAL_OK)
    {
        return ENCODER_STATUS_ERROR;
    }

    hal_status = HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    if (hal_status != HAL_OK)
    {
        return ENCODER_STATUS_ERROR;
    }

    hal_status = HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    if (hal_status != HAL_OK)
    {
        return ENCODER_STATUS_ERROR;
    }

    hal_status = HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    if (hal_status != HAL_OK)
    {
        return ENCODER_STATUS_ERROR;
    }

    for (uint32_t index = 0U; index < (uint32_t)ENCODER_COUNT; index++)
    {
        __HAL_TIM_SET_COUNTER(s_encoder_hw[index].htim, 0U);
    }

    s_encoder_ready = 1U;
    return ENCODER_STATUS_OK;
}

uint8_t Encoder_IsReady(void)
{
    return s_encoder_ready;
}

encoder_status_t Encoder_GetCount(encoder_id_t encoder, int32_t *count)
{
    encoder_status_t status;

    if (!s_encoder_ready)
    {
        return ENCODER_STATUS_NOT_INIT;
    }

    if (count == NULL)
    {
        return ENCODER_STATUS_BAD_PARAM;
    }

    status = Encoder_CheckId(encoder);
    if (status != ENCODER_STATUS_OK)
    {
        return status;
    }

    *count = Encoder_ReadCounter(s_encoder_hw[encoder].htim);
    return ENCODER_STATUS_OK;
}

encoder_status_t Encoder_SetCount(encoder_id_t encoder, int32_t count)
{
    encoder_status_t status;

    if (!s_encoder_ready)
    {
        return ENCODER_STATUS_NOT_INIT;
    }

    status = Encoder_CheckId(encoder);
    if (status != ENCODER_STATUS_OK)
    {
        return status;
    }

    __HAL_TIM_SET_COUNTER(s_encoder_hw[encoder].htim,
                          Encoder_WriteCounterValue(s_encoder_hw[encoder].htim, count));
    return ENCODER_STATUS_OK;
}

encoder_status_t Encoder_ResetCount(encoder_id_t encoder)
{
    return Encoder_SetCount(encoder, 0);
}

encoder_status_t Encoder_ResetAll(void)
{
    if (!s_encoder_ready)
    {
        return ENCODER_STATUS_NOT_INIT;
    }

    for (uint32_t index = 0U; index < (uint32_t)ENCODER_COUNT; index++)
    {
        __HAL_TIM_SET_COUNTER(s_encoder_hw[index].htim, 0U);
    }

    return ENCODER_STATUS_OK;
}
