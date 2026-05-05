#include "screen_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_screen_uart_rx_byte = 0U;
static uint8_t s_screen_uart_rx_buffer[SCREEN_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_screen_uart_rx_head = 0U;
static volatile uint16_t s_screen_uart_rx_tail = 0U;
static char s_screen_uart_line_assembly[SCREEN_UART_LINE_BUFFER_SIZE];
static uint16_t s_screen_uart_line_length = 0U;

static uint16_t ScreenUart_AdvanceIndex(uint16_t index)
{
    index++;
    if (index >= SCREEN_UART_RX_BUFFER_SIZE)
    {
        index = 0U;
    }

    return index;
}

static void ScreenUart_Lock(uint32_t *primask)
{
    if (primask == NULL)
    {
        return;
    }

    *primask = __get_PRIMASK();
    __disable_irq();
}

static void ScreenUart_Unlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void ScreenUart_PushRxByte(uint8_t byte)
{
    uint16_t next_head = ScreenUart_AdvanceIndex(s_screen_uart_rx_head);

    if (next_head == s_screen_uart_rx_tail)
    {
        s_screen_uart_rx_tail = ScreenUart_AdvanceIndex(s_screen_uart_rx_tail);
    }

    s_screen_uart_rx_buffer[s_screen_uart_rx_head] = byte;
    s_screen_uart_rx_head = next_head;
}

static HAL_StatusTypeDef ScreenUart_StartReceiveIt(void)
{
    return HAL_UART_Receive_IT(&huart5, &s_screen_uart_rx_byte, 1U);
}

HAL_StatusTypeDef ScreenUart_Init(void)
{
    uint32_t primask;

    ScreenUart_Lock(&primask);
    s_screen_uart_rx_head = 0U;
    s_screen_uart_rx_tail = 0U;
    ScreenUart_Unlock(primask);

    return ScreenUart_StartReceiveIt();
}

HAL_StatusTypeDef ScreenUart_SendBytes(const uint8_t *data, uint16_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart5, (uint8_t *)data, length, timeout_ms);
}

HAL_StatusTypeDef ScreenUart_SendString(const char *string)
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

    return ScreenUart_SendBytes((const uint8_t *)string, (uint16_t)length, SCREEN_UART_DEFAULT_TIMEOUT_MS);
}

HAL_StatusTypeDef ScreenUart_Printf(const char *format, ...)
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

    return ScreenUart_SendBytes((const uint8_t *)buffer, (uint16_t)length, SCREEN_UART_DEFAULT_TIMEOUT_MS);
}

HAL_StatusTypeDef ScreenUart_ReadByte(uint8_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    ScreenUart_Lock(&primask);

    if (s_screen_uart_rx_head == s_screen_uart_rx_tail)
    {
        ScreenUart_Unlock(primask);
        return HAL_ERROR;
    }

    *data = s_screen_uart_rx_buffer[s_screen_uart_rx_tail];
    s_screen_uart_rx_tail = ScreenUart_AdvanceIndex(s_screen_uart_rx_tail);

    ScreenUart_Unlock(primask);
    return HAL_OK;
}

uint16_t ScreenUart_GetRxCount(void)
{
    uint16_t head;
    uint16_t tail;
    uint32_t primask;

    ScreenUart_Lock(&primask);
    head = s_screen_uart_rx_head;
    tail = s_screen_uart_rx_tail;
    ScreenUart_Unlock(primask);

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(SCREEN_UART_RX_BUFFER_SIZE - tail + head);
}

uint8_t ScreenUart_ReadLine(char *buffer, uint16_t buffer_size)
{
    uint8_t byte;

    if ((buffer == NULL) || (buffer_size < 2U))
    {
        return 0U;
    }

    while (ScreenUart_ReadByte(&byte) == HAL_OK)
    {
        if ((byte == '\r') || (byte == '\n'))
        {
            if (s_screen_uart_line_length == 0U)
            {
                continue;
            }

            if (s_screen_uart_line_length >= buffer_size)
            {
                s_screen_uart_line_length = (uint16_t)(buffer_size - 1U);
            }

            memcpy(buffer, s_screen_uart_line_assembly, s_screen_uart_line_length);
            buffer[s_screen_uart_line_length] = '\0';
            s_screen_uart_line_length = 0U;
            return 1U;
        }

        if (s_screen_uart_line_length < (uint16_t)(sizeof(s_screen_uart_line_assembly) - 1U))
        {
            s_screen_uart_line_assembly[s_screen_uart_line_length++] = (char)byte;
        }
        else
        {
            s_screen_uart_line_length = 0U;
        }
    }

    return 0U;
}

void ScreenUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART5))
    {
        return;
    }

    ScreenUart_PushRxByte(s_screen_uart_rx_byte);
    (void)ScreenUart_StartReceiveIt();
}

void ScreenUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART5))
    {
        return;
    }

    (void)ScreenUart_StartReceiveIt();
}
