#ifndef CAMERA_UART_H
#define CAMERA_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include <stdint.h>

#define CAMERA_UART_DEFAULT_TIMEOUT_MS   ((uint32_t)100U)
#define CAMERA_UART_RX_BUFFER_SIZE       ((uint16_t)256U)
#define CAMERA_UART_DMA_BUFFER_SIZE      ((uint16_t)64U)

HAL_StatusTypeDef CameraUart_Init(void);
HAL_StatusTypeDef CameraUart_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef CameraUart_SendString(const char *string);
HAL_StatusTypeDef CameraUart_Printf(const char *format, ...);
HAL_StatusTypeDef CameraUart_ReadByte(uint8_t *data);
uint16_t CameraUart_GetRxCount(void);
void CameraUart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void CameraUart_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
