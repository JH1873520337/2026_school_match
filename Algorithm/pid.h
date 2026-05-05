#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
    float derivative_lpf_alpha;
} pid_config_t;

typedef struct
{
    pid_config_t config;
    float integral;
    float prev_error;
    float prev_measurement;
    float derivative;
    unsigned char initialized;
} pid_controller_t;

void Pid_GetDefaultConfig(pid_config_t *config);
void Pid_Init(pid_controller_t *controller, const pid_config_t *config);
void Pid_Reset(pid_controller_t *controller);
float Pid_Update(pid_controller_t *controller, float setpoint, float measurement, float dt_s);

#ifdef __cplusplus
}
#endif

#endif
