#include "pid.h"

#include <stddef.h>

static float Pid_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

void Pid_GetDefaultConfig(pid_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    config->kp = 0.0f;
    config->ki = 0.0f;
    config->kd = 0.0f;
    config->integral_limit = 0.0f;
    config->output_limit = 0.0f;
    config->derivative_lpf_alpha = 1.0f;
}

void Pid_Init(pid_controller_t *controller, const pid_config_t *config)
{
    if ((controller == NULL) || (config == NULL))
    {
        return;
    }

    controller->config = *config;
    controller->integral = 0.0f;
    controller->prev_error = 0.0f;
    controller->prev_measurement = 0.0f;
    controller->derivative = 0.0f;
    controller->initialized = 1U;
}

void Pid_Reset(pid_controller_t *controller)
{
    if (controller == NULL)
    {
        return;
    }

    controller->integral = 0.0f;
    controller->prev_error = 0.0f;
    controller->prev_measurement = 0.0f;
    controller->derivative = 0.0f;
}

float Pid_Update(pid_controller_t *controller, float setpoint, float measurement, float dt_s)
{
    float error;
    float proportional;
    float integral_term;
    float derivative_raw;
    float output;
    float alpha;

    if ((controller == NULL) || (controller->initialized == 0U) || (dt_s <= 0.0f))
    {
        return 0.0f;
    }

    error = setpoint - measurement;
    proportional = controller->config.kp * error;

    controller->integral += controller->config.ki * error * dt_s;
    if (controller->config.integral_limit > 0.0f)
    {
        controller->integral = Pid_Clamp(controller->integral,
                                         -controller->config.integral_limit,
                                         controller->config.integral_limit);
    }
    integral_term = controller->integral;

    derivative_raw = -(measurement - controller->prev_measurement) / dt_s;
    alpha = controller->config.derivative_lpf_alpha;
    alpha = Pid_Clamp(alpha, 0.0f, 1.0f);
    controller->derivative += alpha * (derivative_raw - controller->derivative);

    output = proportional + integral_term + (controller->config.kd * controller->derivative);
    if (controller->config.output_limit > 0.0f)
    {
        output = Pid_Clamp(output,
                           -controller->config.output_limit,
                           controller->config.output_limit);
    }

    controller->prev_error = error;
    controller->prev_measurement = measurement;
    return output;
}
