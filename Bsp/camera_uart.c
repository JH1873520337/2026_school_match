#include "camera_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

HAL_StatusTypeDef CameraUart_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart4, (uint8_t *)data, length, timeout_ms);
}

HAL_StatusTypeDef CameraUart_SendString(const char *string)
{
    size_t length;

    if (string == NULL)
    {
        return HAL_ERROR;
    }

    length = strlen(string);
    if (length == 0U)
    {
        return HAL_OK;
    }

    if (length > UINT16_MAX)
    {
        return HAL_ERROR;
    }

    return CameraUart_SendBytes((const uint8_t *)string, (uint16_t)length, CAMERA_UART_DEFAULT_TIMEOUT_MS);
}

HAL_StatusTypeDef CameraUart_Printf(const char *format, ...)
{
    va_list args;
    int length;
    char buffer[128];

    if (format == NULL)
    {
        return HAL_ERROR;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0)
    {
        return HAL_ERROR;
    }

    if (length >= (int)sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    return CameraUart_SendBytes((const uint8_t *)buffer, (uint16_t)length, CAMERA_UART_DEFAULT_TIMEOUT_MS);
}
