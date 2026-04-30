/**
 * @file alarm_service.c
 * @brief 蜂鸣器告警服务层。
 *
 * 当前仅实现题目需要的“检测到火源后蜂鸣器长响三声”。
 * 该模块不阻塞，通过状态机在 AlarmService_Process 中推进节拍。
 */

#include "alarm_service.h"

#include "buzzer.h"

#include <stddef.h>

#define ALARM_FIRE_BEEP_COUNT        ((uint8_t)3U)
#define ALARM_FIRE_BEEP_ON_MS        ((uint32_t)500U)
#define ALARM_FIRE_BEEP_OFF_MS       ((uint32_t)250U)

typedef enum
{
    ALARM_STATE_IDLE = 0,
    ALARM_STATE_BEEP_ON,
    ALARM_STATE_BEEP_OFF
} alarm_service_state_t;

static alarm_service_state_t s_alarm_state = ALARM_STATE_IDLE;
static uint32_t s_alarm_stage_start_ms = 0U;
static uint8_t s_alarm_beep_index = 0U;

void AlarmService_Init(void)
{
    Buzzer_Init();
    s_alarm_state = ALARM_STATE_IDLE;
    s_alarm_stage_start_ms = 0U;
    s_alarm_beep_index = 0U;
}

void AlarmService_Process(void)
{
    uint32_t now_ms;

    if (s_alarm_state == ALARM_STATE_IDLE)
    {
        return;
    }

    now_ms = HAL_GetTick();

    if (s_alarm_state == ALARM_STATE_BEEP_ON)
    {
        if ((now_ms - s_alarm_stage_start_ms) < ALARM_FIRE_BEEP_ON_MS)
        {
            return;
        }

        Buzzer_Off();
        s_alarm_beep_index++;

        if (s_alarm_beep_index >= ALARM_FIRE_BEEP_COUNT)
        {
            s_alarm_state = ALARM_STATE_IDLE;
            return;
        }

        s_alarm_state = ALARM_STATE_BEEP_OFF;
        s_alarm_stage_start_ms = now_ms;
        return;
    }

    if ((now_ms - s_alarm_stage_start_ms) < ALARM_FIRE_BEEP_OFF_MS)
    {
        return;
    }

    Buzzer_On();
    s_alarm_state = ALARM_STATE_BEEP_ON;
    s_alarm_stage_start_ms = now_ms;
}

void AlarmService_PlayFireDetectedPattern(void)
{
    s_alarm_beep_index = 0U;
    s_alarm_stage_start_ms = HAL_GetTick();
    s_alarm_state = ALARM_STATE_BEEP_ON;
    Buzzer_On();
}

void AlarmService_Stop(void)
{
    Buzzer_Off();
    s_alarm_state = ALARM_STATE_IDLE;
    s_alarm_stage_start_ms = 0U;
    s_alarm_beep_index = 0U;
}

uint8_t AlarmService_IsBusy(void)
{
    return (s_alarm_state == ALARM_STATE_IDLE) ? 0U : 1U;
}
