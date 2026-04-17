#include "buzzer.h"

void Buzzer_Init(void)
{
    Buzzer_Off();
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZ_control_GPIO_Port, BUZZ_control_Pin, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZ_control_GPIO_Port, BUZZ_control_Pin, GPIO_PIN_RESET);
}
