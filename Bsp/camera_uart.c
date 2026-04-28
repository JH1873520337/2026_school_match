#include "camera_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static uint8_t s_camera_uart_dma_buffer[CAMERA_UART_DMA_BUFFER_SIZE];
static uint8_t s_camera_uart_rx_buffer[CAMERA_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_camera_uart_rx_head;
static volatile uint16_t s_camera_uart_rx_tail;
static uint8_t s_camera_uart_rx_started;

static uint16_t CameraUart_AdvanceIndex(uint16_t index)
{
    index++;
    if (index >= CAMERA_UART_RX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

static HAL_StatusTypeDef CameraUart_StartReceiveDma(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart4,
                                          s_camera_uart_dma_buffer,
                                          CAMERA_UART_DMA_BUFFER_SIZE);
    if (status == HAL_OK)
    {
        s_camera_uart_rx_started = 1U;
        if (huart4.hdmarx != NULL)
        {
            __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
        }
    }

    return status;
}

static void CameraUart_PushRxByte(uint8_t byte)
{
    uint16_t next_head = CameraUart_AdvanceIndex(s_camera_uart_rx_head);

    if (next_head == s_camera_uart_rx_tail)
    {
        s_camera_uart_rx_tail = CameraUart_AdvanceIndex(s_camera_uart_rx_tail);
    }

    s_camera_uart_rx_buffer[s_camera_uart_rx_head] = byte;
    s_camera_uart_rx_head = next_head;
}

HAL_StatusTypeDef CameraUart_Init(void)
{
    s_camera_uart_rx_head = 0U;
    s_camera_uart_rx_tail = 0U;
    s_camera_uart_rx_started = 0U;
    memset(s_camera_uart_dma_buffer, 0, sizeof(s_camera_uart_dma_buffer));

    return CameraUart_StartReceiveDma();
}

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

HAL_StatusTypeDef CameraUart_ReadByte(uint8_t *data)
{
    uint32_t primask;

    if (data == NULL)
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (s_camera_uart_rx_head == s_camera_uart_rx_tail)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HAL_ERROR;
    }

    *data = s_camera_uart_rx_buffer[s_camera_uart_rx_tail];
    s_camera_uart_rx_tail = CameraUart_AdvanceIndex(s_camera_uart_rx_tail);

    if (primask == 0U)
    {
        __enable_irq();
    }

    return HAL_OK;
}

uint16_t CameraUart_GetRxCount(void)
{
    uint16_t head;
    uint16_t tail;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    head = s_camera_uart_rx_head;
    tail = s_camera_uart_rx_tail;
    if (primask == 0U)
    {
        __enable_irq();
    }

    if (head >= tail)
    {
        return (uint16_t)(head - tail);
    }

    return (uint16_t)(CAMERA_UART_RX_BUFFER_SIZE - tail + head);
}

void CameraUart_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    uint16_t index;

    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    if (size > CAMERA_UART_DMA_BUFFER_SIZE)
    {
        size = CAMERA_UART_DMA_BUFFER_SIZE;
    }

    for (index = 0U; index < size; index++)
    {
        CameraUart_PushRxByte(s_camera_uart_dma_buffer[index]);
    }

    (void)CameraUart_StartReceiveDma();
}

void CameraUart_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != UART4))
    {
        return;
    }

    s_camera_uart_rx_started = 0U;
    (void)CameraUart_StartReceiveDma();
}
