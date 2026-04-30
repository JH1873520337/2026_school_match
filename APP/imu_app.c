/**
 * @file imu_app.c
 * @brief IMU APP 层实现，封装 Module/imu_service 为 RTOS 可调度的 Init/Process 模式。
 *
 * 本层不包含任何算法或硬件操作，所有逻辑委托给 imu_service。
 */

#include "imu_app.h"

#include <stddef.h>

void ImuApp_Init(void)
{
    /* 委托给服务层完成传感器初始化 + 上电标定 + 姿态融合器初始化 */
    ImuService_Init();
}

void ImuApp_Process(void)
{
    /* 委托给服务层完成传感器读取 + 零偏校正 + 姿态融合更新 */
    ImuService_Update();
}

const imu_data_t *ImuApp_GetData(void)
{
    /* 透传服务层的输出数据指针 */
    return ImuService_GetData();
}

uint8_t ImuApp_IsCalibrated(void)
{
    /* 透传服务层的标定状态 */
    return ImuService_IsCalibrated();
}
