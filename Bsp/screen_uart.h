#ifndef SCREEN_UART_H
#define SCREEN_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include <stdint.h>

#define SCREEN_UART_DEFAULT_TIMEOUT_MS   ((uint32_t)100U)
#define SCREEN_UART_RX_BUFFER_SIZE       ((uint16_t)256U)
#define SCREEN_UART_LINE_BUFFER_SIZE     ((uint16_t)128U)

HAL_StatusTypeDef ScreenUart_Init(void);
HAL_StatusTypeDef ScreenUart_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef ScreenUart_SendString(const char *string);
HAL_StatusTypeDef ScreenUart_Printf(const char *format, ...);
HAL_StatusTypeDef ScreenUart_ReadByte(uint8_t *data);
uint16_t ScreenUart_GetRxCount(void);
uint8_t ScreenUart_ReadLine(char *buffer, uint16_t buffer_size);
void ScreenUart_RxCpltCallback(UART_HandleTypeDef *huart);
void ScreenUart_ErrorCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
