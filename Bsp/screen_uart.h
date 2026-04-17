#ifndef SCREEN_UART_H
#define SCREEN_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include <stdint.h>

#define SCREEN_UART_DEFAULT_TIMEOUT_MS   ((uint32_t)100U)

HAL_StatusTypeDef ScreenUart_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef ScreenUart_SendString(const char *string);
HAL_StatusTypeDef ScreenUart_Printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
