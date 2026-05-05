#include "vision_app.h"

#include "main.h"
#include "screen_uart.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>

#define VISION_PROTO_HEADER0  ((uint8_t)0xAAU)
#define VISION_PROTO_HEADER1  ((uint8_t)0x55U)
#define VISION_PROTO_LEN      ((uint16_t)19U)
#define VISION_RING_SIZE      ((uint16_t)128U)

typedef enum
{
    VISION_STATE_IDLE = 0,
    VISION_STATE_GOT_AA,
    VISION_STATE_COLLECTING
} vision_parse_state_t;

static uint8_t s_rx_byte = 0U;
static uint8_t s_ring[VISION_RING_SIZE];
static volatile uint16_t s_ring_wr = 0U;
static uint16_t s_ring_rd = 0U;
static vision_target_t s_target;
static volatile uint8_t s_frame_ready = 0U;

static uint8_t VisionProto_Crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0U;
    uint16_t i;

    for (i = 0U; i < len; i++)
    {
        crc ^= buf[i];
    }

    return crc;
}

static void VisionApp_ParseByte(uint8_t byte)
{
    static uint8_t frame[VISION_PROTO_LEN];
    static uint8_t index = 0U;
    static vision_parse_state_t state = VISION_STATE_IDLE;

    switch (state)
    {
        case VISION_STATE_IDLE:
            if (byte == VISION_PROTO_HEADER0)
            {
                frame[0] = byte;
                state = VISION_STATE_GOT_AA;
            }
            break;

        case VISION_STATE_GOT_AA:
            if (byte == VISION_PROTO_HEADER1)
            {
                frame[1] = byte;
                index = 2U;
                state = VISION_STATE_COLLECTING;
            }
            else if (byte != VISION_PROTO_HEADER0)
            {
                state = VISION_STATE_IDLE;
            }
            break;

        case VISION_STATE_COLLECTING:
            frame[index++] = byte;
            if (index >= VISION_PROTO_LEN)
            {
                uint8_t crc;

                state = VISION_STATE_IDLE;
                crc = VisionProto_Crc8(&frame[2U], 16U);
                if (crc != frame[18U])
                {
                    break;
                }

                s_target.seq = frame[2];
                s_target.target_kind = frame[3];
                s_target.valid = (frame[4] & 0x01U) ? 1U : 0U;
                s_target.stale = (frame[4] & 0x02U) ? 1U : 0U;
                s_target.quality = frame[5];
                s_target.cx = (uint16_t)frame[6] | ((uint16_t)frame[7] << 8);
                s_target.cy = (uint16_t)frame[8] | ((uint16_t)frame[9] << 8);
                s_target.ex = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
                s_target.ey = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
                s_target.area = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8);
                s_target.angle = (int16_t)((uint16_t)frame[16] | ((uint16_t)frame[17] << 8));
                s_target.crc_ok = 1U;
                s_target.frame_count++;
                s_frame_ready = 1U;
            }
            break;

        default:
            state = VISION_STATE_IDLE;
            break;
    }
}

void VisionApp_Init(void)
{
    (void)memset(&s_target, 0, sizeof(s_target));
    s_ring_wr = 0U;
    s_ring_rd = 0U;
    s_frame_ready = 0U;

    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

void VisionApp_Process(void)
{
    uint16_t wr = s_ring_wr;

    while (s_ring_rd != wr)
    {
        VisionApp_ParseByte(s_ring[s_ring_rd]);
        s_ring_rd = (s_ring_rd + 1U) % VISION_RING_SIZE;
    }
}

const vision_target_t *VisionApp_GetTarget(void)
{
    return &s_target;
}

uint8_t VisionApp_IsNewFrame(void)
{
    uint8_t ready = s_frame_ready;
    s_frame_ready = 0U;
    return ready;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        uint16_t next = (s_ring_wr + 1U) % VISION_RING_SIZE;

        if (next != s_ring_rd)
        {
            s_ring[s_ring_wr] = s_rx_byte;
            s_ring_wr = next;
        }

        HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
        return;
    }

    if (huart == &huart5)
    {
        ScreenUart_RxCpltCallback(huart);
    }
}
