#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 一阶低通滤波器句柄。
 */
typedef struct
{
    float alpha;
    float output;
} filter_lp1_t;

/**
 * @brief 用截止频率初始化一阶低通滤波器。
 * @param filter 滤波器句柄指针。
 * @param cutoff_hz 截止频率，单位 Hz。
 * @param sample_hz 采样频率，单位 Hz。
 */
void Filter_Lp1_Init(filter_lp1_t *filter, float cutoff_hz, float sample_hz);

/**
 * @brief 用时间常数初始化一阶低通滤波器。
 * @param filter 滤波器句柄指针。
 * @param tau 时间常数，tau = 1/(2*pi*cutoff_hz)。
 */
void Filter_Lp1_InitTau(filter_lp1_t *filter, float tau);

/**
 * @brief 更新一阶低通滤波器并返回滤波后的值。
 * @param filter 滤波器句柄指针。
 * @param input 当前采样值。
 * @return 滤波后的值。
 */
float Filter_Lp1_Update(filter_lp1_t *filter, float input);

/**
 * @brief 重置滤波器内部状态。
 * @param filter 滤波器句柄指针。
 * @param value 重置值。
 */
void Filter_Lp1_Reset(filter_lp1_t *filter, float value);

#ifdef __cplusplus
}
#endif

#endif
