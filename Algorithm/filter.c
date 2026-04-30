#include "filter.h"

#include <stddef.h>

#define FILTER_PI (3.14159265358979323846f)

void Filter_Lp1_Init(filter_lp1_t *filter, float cutoff_hz, float sample_hz)
{
    float tau;
    float dt;

    if (filter == NULL)
    {
        return;
    }

    if (cutoff_hz <= 0.0f || sample_hz <= 0.0f)
    {
        filter->alpha = 1.0f;
        filter->output = 0.0f;
        return;
    }

    tau = 1.0f / (2.0f * FILTER_PI * cutoff_hz);
    dt = 1.0f / sample_hz;

    if (tau <= 0.0f)
    {
        filter->alpha = 1.0f;
    }
    else
    {
        filter->alpha = dt / (dt + tau);

        if (filter->alpha > 1.0f)
        {
            filter->alpha = 1.0f;
        }
        if (filter->alpha < 0.0f)
        {
            filter->alpha = 0.0f;
        }
    }

    filter->output = 0.0f;
}

void Filter_Lp1_InitTau(filter_lp1_t *filter, float tau)
{
    if (filter == NULL)
    {
        return;
    }

    /* tau 仅用于存储，实际 alpha 依赖调用方在 Update 前通过采样周期计算。
       这里只记录 tau 并重置状态，供后续手动设置 alpha 时参考。 */
    if (tau <= 0.0f)
    {
        filter->alpha = 1.0f;
    }
    else
    {
        filter->alpha = 0.0f;
    }

    filter->output = 0.0f;

    (void)tau;
}

float Filter_Lp1_Update(filter_lp1_t *filter, float input)
{
    if (filter == NULL)
    {
        return input;
    }

    filter->output = filter->alpha * input + (1.0f - filter->alpha) * filter->output;

    return filter->output;
}

void Filter_Lp1_Reset(filter_lp1_t *filter, float value)
{
    if (filter == NULL)
    {
        return;
    }

    filter->output = value;
}
