# Motor PID Tuning Notes

## Motor 1

Status: current active speed-loop test target on 2026-05-04

Test target:
- Motor: `TB6612_MOTOR_1`
- Encoder: `ENCODER_MOTOR_1`

Current PID tune parameters in `APP/pid_tune_app.c`:

```c
kp = 6800.0f;
ki = 1500.0f;
kd = 0.0f;
integral_limit = 220.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Other related test parameters:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Target sequence used during tuning:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

Retest setup restored on 2026-05-05 from current `APP/pid_tune_app.c`:

```c
PID_TUNE_TEST_MOTOR = TB6612_MOTOR_4;
PID_TUNE_TEST_ENCODER = ENCODER_MOTOR_4;
kp = 4150.0f;
ki = 2950.0f;
kd = 0.0f;
integral_limit = 320.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Retest related settings:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Retest target sequence:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

Latest retest record on 2026-05-05 from current `APP/pid_tune_app.c`:

```c
kp = 6000.0f;
ki = 1700.0f;
kd = 0.0f;
integral_limit = 220.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Latest related test settings:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Latest target sequence:

```c
0.05f, 0.10f, 0.15f, -0.10f
```

## Motor 2

Status: temporarily finalized on 2026-05-04

Test target:
- Motor: `TB6612_MOTOR_2`
- Encoder: `ENCODER_MOTOR_2`

Current PID tune parameters from `APP/pid_tune_app.c` before switching to motor 3:

```c
kp = 2800.0f;
ki = 2800.0f;
kd = 0.0f;
integral_limit = 260.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Other related test parameters:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 70;
```

Target sequence used during tuning:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

Retest setup restored on 2026-05-05 from current `APP/pid_tune_app.c`:

```c
PID_TUNE_TEST_MOTOR = TB6612_MOTOR_2;
PID_TUNE_TEST_ENCODER = ENCODER_MOTOR_2;
kp = 2800.0f;
ki = 2800.0f;
kd = 0.0f;
integral_limit = 260.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Retest related settings:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 70;
```

Retest target sequence:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

## Motor 3

Status: temporarily finalized on 2026-05-04

Test target:
- Motor: `TB6612_MOTOR_3`
- Encoder: `ENCODER_MOTOR_3`

Current PID tune parameters from `APP/pid_tune_app.c` before switching to motor 4:

```c
kp = 4200.0f;
ki = 2950.0f;
kd = 0.0f;
integral_limit = 320.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Other related test parameters:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Target sequence used during tuning:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

Retest setup restored on 2026-05-05 from current `APP/pid_tune_app.c`:

```c
PID_TUNE_TEST_MOTOR = TB6612_MOTOR_3;
PID_TUNE_TEST_ENCODER = ENCODER_MOTOR_3;
kp = 4200.0f;
ki = 2950.0f;
kd = 0.0f;
integral_limit = 320.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Retest related settings:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Retest target sequence:

```c
0.05f, 0.10f, 0.20f, -0.10f
```

## Motor 4

Status: temporarily finalized on 2026-05-04

Test target:
- Motor: `TB6612_MOTOR_4`
- Encoder: `ENCODER_MOTOR_4`

Current PID tune parameters in `APP/pid_tune_app.c`:

```c
kp = 4150.0f;
ki = 2950.0f;
kd = 0.0f;
integral_limit = 320.0f;
output_limit = (float)TB6612_SPEED_MAX;
derivative_lpf_alpha = 0.25f;
```

Other related test parameters:

```c
PID_TUNE_SAMPLE_PERIOD_MS = 10;
PID_TUNE_PRINT_PERIOD_MS = 100;
PID_TUNE_STEP_PERIOD_MS = 3000;
PID_TUNE_COUNTS_PER_REV = 1040.0f;
PID_TUNE_DRUM_RADIUS_M = 0.012f;
PWM minimum effective value = 80;
```

Target sequence used during tuning:

```c
0.05f, 0.10f, 0.20f, -0.10f
```
