#include "pid_tune_app.h"

#include "TB6612.h"
#include "encoder.h"
#include "pid.h"
#include "screen_uart.h"

#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define PID_TUNE_SAMPLE_PERIOD_MS        ((uint32_t)10U)
#define PID_TUNE_SAMPLE_PERIOD_S         (0.010f)
#define PID_TUNE_PRINT_PERIOD_MS         ((uint32_t)100U)
#define PID_TUNE_STEP_PERIOD_MS          ((uint32_t)3000U)
#define PID_TUNE_COUNTS_PER_REV          (1040.0f)
#define PID_TUNE_DRUM_RADIUS_M           (0.012f)
#define PID_TUNE_TARGET_SPEED_COUNT      (4U)
#define PID_TUNE_TEST_MOTOR              (TB6612_MOTOR_4)
#define PID_TUNE_TEST_ENCODER            (ENCODER_MOTOR_4)

static const float s_target_speed_seq_mps[PID_TUNE_TARGET_SPEED_COUNT] =
{
    0.05f,
    0.10f,
    0.20f,
    -0.10f
};

static pid_controller_t s_speed_pid;
static float s_target_speed_mps = 0.0f;
static float s_measured_speed_mps = 0.0f;
static float s_speed_error_mps = 0.0f;
static int16_t s_pwm_output = 0;
static int32_t s_prev_count = 0;
static int32_t s_encoder_count = 0;
static int32_t s_delta_count = 0;
static uint32_t s_last_update_ms = 0U;
static uint32_t s_last_print_ms = 0U;
static uint32_t s_last_step_ms = 0U;
static uint32_t s_step_index = 0U;
static uint8_t s_initialized = 0U;
static uint8_t s_auto_step_enable = 1U;

static int16_t PidTuneApp_ClampPwm(float pwm_value)
{
    if (pwm_value > (float)TB6612_SPEED_MAX)
    {
        pwm_value = (float)TB6612_SPEED_MAX;
    }
    else if (pwm_value < -(float)TB6612_SPEED_MAX)
    {
        pwm_value = -(float)TB6612_SPEED_MAX;
    }

    if ((pwm_value > 0.0f) && (pwm_value < 80.0f))
    {
        pwm_value = 80.0f;
    }
    else if ((pwm_value < 0.0f) && (pwm_value > -80.0f))
    {
        pwm_value = -80.0f;
    }

    return (int16_t)pwm_value;
}

static float PidTuneApp_CountDeltaToSpeedMps(int32_t delta_count)
{
    const float rev = (float)delta_count / PID_TUNE_COUNTS_PER_REV;
    const float meters = rev * (2.0f * 3.14159265358979323846f * PID_TUNE_DRUM_RADIUS_M);
    return meters / PID_TUNE_SAMPLE_PERIOD_S;
}

static int32_t PidTuneApp_GetCountDelta(int32_t current_count, int32_t previous_count)
{
    return (int32_t)(current_count - previous_count);
}

static void PidTuneApp_PrintHelp(void)
{
    (void)ScreenUart_Printf("cmd: kp=, ki=, kd=, ilim=, tgt=, auto=0/1, stop, stat\r\n");
}

static void PidTuneApp_PrintStatus(void)
{
    (void)ScreenUart_Printf("kp=%.3f ki=%.3f kd=%.3f ilim=%.3f tgt=%.3f auto=%u\r\n",
                            (double)s_speed_pid.config.kp,
                            (double)s_speed_pid.config.ki,
                            (double)s_speed_pid.config.kd,
                            (double)s_speed_pid.config.integral_limit,
                            (double)s_target_speed_mps,
                            (unsigned int)s_auto_step_enable);
}

static uint8_t PidTuneApp_ParseFloatArg(const char *line, const char *prefix, float *value)
{
    char *end_ptr;

    if ((line == NULL) || (prefix == NULL) || (value == NULL))
    {
        return 0U;
    }

    if (strncmp(line, prefix, strlen(prefix)) != 0)
    {
        return 0U;
    }

    *value = strtof(line + strlen(prefix), &end_ptr);
    if ((end_ptr == (line + strlen(prefix))) || (*end_ptr != '\0'))
    {
        return 0U;
    }

    return 1U;
}

static void PidTuneApp_UpdateTarget(uint32_t now_ms)
{
    if (s_auto_step_enable == 0U)
    {
        return;
    }

    if ((now_ms - s_last_step_ms) < PID_TUNE_STEP_PERIOD_MS)
    {
        return;
    }

    s_last_step_ms = now_ms;
    s_step_index++;
    if (s_step_index >= PID_TUNE_TARGET_SPEED_COUNT)
    {
        s_step_index = 0U;
    }

    s_target_speed_mps = s_target_speed_seq_mps[s_step_index];
    Pid_Reset(&s_speed_pid);
}

static void PidTuneApp_HandleCommand(char *line)
{
    float value;

    if (line == NULL)
    {
        return;
    }

    if (PidTuneApp_ParseFloatArg(line, "kp=", &value) != 0U)
    {
        s_speed_pid.config.kp = value;
        (void)ScreenUart_Printf("set kp=%.3f\r\n", (double)value);
        return;
    }

    if (PidTuneApp_ParseFloatArg(line, "ki=", &value) != 0U)
    {
        s_speed_pid.config.ki = value;
        (void)ScreenUart_Printf("set ki=%.3f\r\n", (double)value);
        return;
    }

    if (PidTuneApp_ParseFloatArg(line, "kd=", &value) != 0U)
    {
        s_speed_pid.config.kd = value;
        (void)ScreenUart_Printf("set kd=%.3f\r\n", (double)value);
        return;
    }

    if (PidTuneApp_ParseFloatArg(line, "ilim=", &value) != 0U)
    {
        s_speed_pid.config.integral_limit = value;
        (void)ScreenUart_Printf("set ilim=%.3f\r\n", (double)value);
        return;
    }

    if (PidTuneApp_ParseFloatArg(line, "tgt=", &value) != 0U)
    {
        s_auto_step_enable = 0U;
        s_target_speed_mps = value;
        Pid_Reset(&s_speed_pid);
        (void)ScreenUart_Printf("set target=%.3f\r\n", (double)value);
        return;
    }

    if (strcmp(line, "auto=1") == 0)
    {
        s_auto_step_enable = 1U;
        s_last_step_ms = HAL_GetTick();
        (void)ScreenUart_Printf("auto step on\r\n");
        return;
    }

    if (strcmp(line, "auto=0") == 0)
    {
        s_auto_step_enable = 0U;
        (void)ScreenUart_Printf("auto step off\r\n");
        return;
    }

    if (strcmp(line, "stop") == 0)
    {
        s_auto_step_enable = 0U;
        s_target_speed_mps = 0.0f;
        Pid_Reset(&s_speed_pid);
        (void)ScreenUart_Printf("motor stop\r\n");
        return;
    }

    if (strcmp(line, "stat") == 0)
    {
        PidTuneApp_PrintStatus();
        return;
    }

    if (strcmp(line, "help") == 0)
    {
        PidTuneApp_PrintHelp();
        return;
    }

    (void)ScreenUart_Printf("unknown: %s\r\n", line);
}

static void PidTuneApp_ProcessCommands(void)
{
    char line[SCREEN_UART_LINE_BUFFER_SIZE];

    while (ScreenUart_ReadLine(line, sizeof(line)) != 0U)
    {
        PidTuneApp_HandleCommand(line);
    }
}

void PidTuneApp_Init(void)
{
    pid_config_t pid_config;
    int32_t count = 0;

    if (Encoder_Init() != ENCODER_STATUS_OK)
    {
        return;
    }

    if (TB6612_Init() != TB6612_STATUS_OK)
    {
        return;
    }

    Pid_GetDefaultConfig(&pid_config);
    pid_config.kp = 4150.0f;
    pid_config.ki = 2950.0f;
    pid_config.kd = 0.0f;
    pid_config.integral_limit = 320.0f;
    pid_config.output_limit = (float)TB6612_SPEED_MAX;
    pid_config.derivative_lpf_alpha = 0.25f;
    Pid_Init(&s_speed_pid, &pid_config);

    (void)Encoder_GetCount(PID_TUNE_TEST_ENCODER, &count);
    s_prev_count = count;
    s_encoder_count = count;
    s_delta_count = 0;
    s_target_speed_mps = s_target_speed_seq_mps[0];
    s_measured_speed_mps = 0.0f;
    s_speed_error_mps = 0.0f;
    s_pwm_output = 0;
    s_last_update_ms = HAL_GetTick();
    s_last_print_ms = s_last_update_ms;
    s_last_step_ms = s_last_update_ms;
    s_step_index = 0U;
    s_auto_step_enable = 1U;
    s_initialized = 1U;
}

void PidTuneApp_Process(void)
{
    int32_t count = 0;
    int32_t delta_count;
    float pid_output;
    uint32_t now_ms;

    if (s_initialized == 0U)
    {
        return;
    }

    PidTuneApp_ProcessCommands();

    now_ms = HAL_GetTick();
    if ((now_ms - s_last_update_ms) < PID_TUNE_SAMPLE_PERIOD_MS)
    {
        return;
    }

    s_last_update_ms = now_ms;
    PidTuneApp_UpdateTarget(now_ms);

    if (Encoder_GetCount(PID_TUNE_TEST_ENCODER, &count) != ENCODER_STATUS_OK)
    {
        return;
    }

    delta_count = PidTuneApp_GetCountDelta(count, s_prev_count);
    s_prev_count = count;
    s_encoder_count = count;
    s_delta_count = delta_count;
    s_measured_speed_mps = PidTuneApp_CountDeltaToSpeedMps(delta_count);
    s_speed_error_mps = s_target_speed_mps - s_measured_speed_mps;

    pid_output = Pid_Update(&s_speed_pid,
                            s_target_speed_mps,
                            s_measured_speed_mps,
                            PID_TUNE_SAMPLE_PERIOD_S);
    s_pwm_output = PidTuneApp_ClampPwm(pid_output);

    if (s_target_speed_mps == 0.0f)
    {
        Pid_Reset(&s_speed_pid);
        s_pwm_output = 0;
    }

    (void)TB6612_StopMotor(TB6612_MOTOR_1);
    (void)TB6612_StopMotor(TB6612_MOTOR_2);
    (void)TB6612_StopMotor(TB6612_MOTOR_3);
    (void)TB6612_SetMotorSpeed(PID_TUNE_TEST_MOTOR, s_pwm_output);

    if ((now_ms - s_last_print_ms) >= PID_TUNE_PRINT_PERIOD_MS)
    {
        s_last_print_ms = now_ms;

        (void)ScreenUart_Printf("%.3f,%.3f,%.3f,%d,%ld,%ld,%.2f\n",
                                (double)s_target_speed_mps,
                                (double)s_measured_speed_mps,
                                (double)s_speed_error_mps,
                                (int)s_pwm_output,
                                (long)s_encoder_count,
                                (long)s_delta_count,
                                (double)s_speed_pid.integral);
    }
}
