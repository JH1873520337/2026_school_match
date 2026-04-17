#include "TB6612.h"

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
} tb6612_motor_hw_t;

static const tb6612_motor_hw_t s_tb6612_motor_hw[TB6612_MOTOR_COUNT] =
{
    {&htim9, TIM_CHANNEL_1, TB6612_1_AIN1_GPIO_Port, TB6612_1_AIN1_Pin, TB6612_1_AIN2_GPIO_Port, TB6612_1_AIN2_Pin},
    {&htim9, TIM_CHANNEL_2, TB6612_1_BIN1_GPIO_Port, TB6612_1_BIN1_Pin, TB6612_1_BIN2_GPIO_Port, TB6612_1_BIN2_Pin},
    {&htim12, TIM_CHANNEL_2, TB6612_2_AIN1_GPIO_Port, TB6612_2_AIN1_Pin, TB6612_2_AIN2_GPIO_Port, TB6612_2_AIN2_Pin},
    {&htim12, TIM_CHANNEL_1, TB6612_2_BIN1_GPIO_Port, TB6612_2_BIN1_Pin, TB6612_2_BIN2_GPIO_Port, TB6612_2_BIN2_Pin}
};

static uint8_t s_tb6612_ready = 0U;

static tb6612_status_t TB6612_CheckMotor(tb6612_motor_t motor)
{
    if (motor >= TB6612_MOTOR_COUNT)
    {
        return TB6612_STATUS_BAD_PARAM;
    }

    return TB6612_STATUS_OK;
}

static void TB6612_WriteDirection(const tb6612_motor_hw_t *motor_hw, GPIO_PinState in1_state, GPIO_PinState in2_state)
{
    HAL_GPIO_WritePin(motor_hw->in1_port, motor_hw->in1_pin, in1_state);
    HAL_GPIO_WritePin(motor_hw->in2_port, motor_hw->in2_pin, in2_state);
}

static uint16_t TB6612_SpeedToPulse(const tb6612_motor_hw_t *motor_hw, uint16_t speed)
{
    uint32_t period;

    if (speed >= (uint16_t)TB6612_SPEED_MAX)
    {
        return (uint16_t)__HAL_TIM_GET_AUTORELOAD(motor_hw->htim);
    }

    period = __HAL_TIM_GET_AUTORELOAD(motor_hw->htim);
    return (uint16_t)((period * speed) / (uint32_t)TB6612_SPEED_MAX);
}

static void TB6612_SetPulse(const tb6612_motor_hw_t *motor_hw, uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(motor_hw->htim, motor_hw->channel, pulse);
}

tb6612_status_t TB6612_Init(void)
{
    HAL_StatusTypeDef hal_status;

    hal_status = HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    if (hal_status != HAL_OK)
    {
        return TB6612_STATUS_ERROR;
    }

    hal_status = HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
    if (hal_status != HAL_OK)
    {
        return TB6612_STATUS_ERROR;
    }

    hal_status = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
    if (hal_status != HAL_OK)
    {
        return TB6612_STATUS_ERROR;
    }

    hal_status = HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    if (hal_status != HAL_OK)
    {
        return TB6612_STATUS_ERROR;
    }

    for (uint32_t index = 0U; index < (uint32_t)TB6612_MOTOR_COUNT; index++)
    {
        TB6612_WriteDirection(&s_tb6612_motor_hw[index], GPIO_PIN_RESET, GPIO_PIN_RESET);
        TB6612_SetPulse(&s_tb6612_motor_hw[index], 0U);
    }

    s_tb6612_ready = 1U;
    return TB6612_STATUS_OK;
}

uint8_t TB6612_IsReady(void)
{
    return s_tb6612_ready;
}

tb6612_status_t TB6612_SetMotorSpeed(tb6612_motor_t motor, int16_t speed)
{
    const tb6612_motor_hw_t *motor_hw;
    uint16_t pulse;
    tb6612_status_t status;

    if (!s_tb6612_ready)
    {
        return TB6612_STATUS_NOT_INIT;
    }

    status = TB6612_CheckMotor(motor);
    if (status != TB6612_STATUS_OK)
    {
        return status;
    }

    if ((speed < -TB6612_SPEED_MAX) || (speed > TB6612_SPEED_MAX))
    {
        return TB6612_STATUS_BAD_PARAM;
    }

    motor_hw = &s_tb6612_motor_hw[motor];

    if (speed > 0)
    {
        pulse = TB6612_SpeedToPulse(motor_hw, (uint16_t)speed);
        TB6612_WriteDirection(motor_hw, GPIO_PIN_SET, GPIO_PIN_RESET);
        TB6612_SetPulse(motor_hw, pulse);
        return TB6612_STATUS_OK;
    }

    if (speed < 0)
    {
        pulse = TB6612_SpeedToPulse(motor_hw, (uint16_t)(-speed));
        TB6612_WriteDirection(motor_hw, GPIO_PIN_RESET, GPIO_PIN_SET);
        TB6612_SetPulse(motor_hw, pulse);
        return TB6612_STATUS_OK;
    }

    return TB6612_StopMotor(motor);
}

tb6612_status_t TB6612_StopMotor(tb6612_motor_t motor)
{
    tb6612_status_t status;

    if (!s_tb6612_ready)
    {
        return TB6612_STATUS_NOT_INIT;
    }

    status = TB6612_CheckMotor(motor);
    if (status != TB6612_STATUS_OK)
    {
        return status;
    }

    TB6612_WriteDirection(&s_tb6612_motor_hw[motor], GPIO_PIN_RESET, GPIO_PIN_RESET);
    TB6612_SetPulse(&s_tb6612_motor_hw[motor], 0U);
    return TB6612_STATUS_OK;
}

tb6612_status_t TB6612_BrakeMotor(tb6612_motor_t motor)
{
    tb6612_status_t status;

    if (!s_tb6612_ready)
    {
        return TB6612_STATUS_NOT_INIT;
    }

    status = TB6612_CheckMotor(motor);
    if (status != TB6612_STATUS_OK)
    {
        return status;
    }

    TB6612_WriteDirection(&s_tb6612_motor_hw[motor], GPIO_PIN_SET, GPIO_PIN_SET);
    TB6612_SetPulse(&s_tb6612_motor_hw[motor], (uint16_t)__HAL_TIM_GET_AUTORELOAD(s_tb6612_motor_hw[motor].htim));
    return TB6612_STATUS_OK;
}

tb6612_status_t TB6612_StopAll(void)
{
    if (!s_tb6612_ready)
    {
        return TB6612_STATUS_NOT_INIT;
    }

    for (uint32_t index = 0U; index < (uint32_t)TB6612_MOTOR_COUNT; index++)
    {
        TB6612_WriteDirection(&s_tb6612_motor_hw[index], GPIO_PIN_RESET, GPIO_PIN_RESET);
        TB6612_SetPulse(&s_tb6612_motor_hw[index], 0U);
    }

    return TB6612_STATUS_OK;
}
