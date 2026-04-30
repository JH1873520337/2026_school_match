#include "vision_app.h"
#include "main.h"
#include "usart.h"

#include <stddef.h>
#include <string.h>

/* OpenMV 串口帧协议常量 */
#define VISION_PROTO_HEADER0  ((uint8_t)0xAAU)   /* 帧头 0 */
#define VISION_PROTO_HEADER1  ((uint8_t)0x55U)   /* 帧头 1 */
#define VISION_PROTO_LEN      ((uint16_t)19U)    /* 完整帧字节数 */
#define VISION_RING_SIZE      ((uint16_t)128U)   /* 环形缓冲区容量 */

/* 帧解析状态机 */
typedef enum
{
    VISION_STATE_IDLE = 0,       /* 等待帧头 0xAA */
    VISION_STATE_GOT_AA,         /* 已收到 0xAA，等待 0x55 */
    VISION_STATE_COLLECTING      /* 正在收集剩余 17 字节 */
} vision_parse_state_t;

/* UART4 中断接收单字节缓冲 */
static uint8_t s_rx_byte = 0U;

/* 环形缓冲区：ISR 写入，Process 读取 */
static uint8_t s_ring[VISION_RING_SIZE];
static volatile uint16_t s_ring_wr = 0U;   /* ISR 写入指针 */
static uint16_t s_ring_rd = 0U;            /* Process 读取指针 */

/* 最新一帧的解析结果 */
static vision_target_t s_target;

/* 新帧就绪标志：ISR 置 1，调用方通过 VisionApp_IsNewFrame 查询并清除 */
static volatile uint8_t s_frame_ready = 0U;

/**
 * @brief OpenMV 帧 CRC8 异或校验。
 * @param buf 数据起始指针（跳过两字节帧头）。
 * @param len 校验字节数（16 字节）。
 * @return 8 位异或校验值。
 */
static uint8_t VisionProto_Crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= buf[i];
    }
    return crc;
}

/**
 * @brief 逐字节帧解析状态机。
 *
 * 由 VisionApp_Process 调用，不在 ISR 中执行，
 * 遵循 "ISR 做最小动作" 原则。
 *
 * @param byte 当前消费的字节。
 */
static void VisionApp_ParseByte(uint8_t byte)
{
    static uint8_t frame[VISION_PROTO_LEN];
    static uint8_t index = 0U;
    static vision_parse_state_t state = VISION_STATE_IDLE;

    switch (state)
    {
    case VISION_STATE_IDLE:
        /* 等待 0xAA */
        if (byte == VISION_PROTO_HEADER0)
        {
            frame[0] = byte;
            state = VISION_STATE_GOT_AA;
        }
        break;

    case VISION_STATE_GOT_AA:
        /* 等待 0x55，若不是则回到 IDLE（若仍是 AA 则保持 GOT_AA） */
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
            state = VISION_STATE_IDLE;

            /* CRC8 校验：对 [2..17] 共 16 字节做异或 */
            uint8_t crc = VisionProto_Crc8(&frame[2U], 16U);
            if (crc != frame[18U]) break;   /* 校验失败则丢弃本帧 */

            /* 校验通过，提取各字段（Little-Endian） */
            s_target.seq         = frame[2];
            s_target.target_kind = frame[3];
            s_target.valid       = (frame[4] & 0x01U) ? 1U : 0U;
            s_target.stale       = (frame[4] & 0x02U) ? 1U : 0U;
            s_target.quality     = frame[5];
            s_target.cx          = (uint16_t)frame[6]  | ((uint16_t)frame[7]  << 8);
            s_target.cy          = (uint16_t)frame[8]  | ((uint16_t)frame[9]  << 8);
            s_target.ex          = (int16_t)((uint16_t)frame[10] | ((uint16_t)frame[11] << 8));
            s_target.ey          = (int16_t)((uint16_t)frame[12] | ((uint16_t)frame[13] << 8));
            s_target.area        = (uint16_t)frame[14] | ((uint16_t)frame[15] << 8);
            s_target.angle       = (int16_t)((uint16_t)frame[16] | ((uint16_t)frame[17] << 8));
            s_target.crc_ok      = 1U;
            s_target.frame_count++;
            s_frame_ready        = 1U;
        }
        break;
    }
}

void VisionApp_Init(void)
{
    memset(&s_target, 0, sizeof(s_target));
    s_ring_wr    = 0U;
    s_ring_rd    = 0U;
    s_frame_ready = 0U;

    /* 使能 UART4 中断并启动单字节循环接收 */
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
}

void VisionApp_Process(void)
{
    uint16_t wr;

    /* 快照写入指针，避免 ISR 修改导致判断错误 */
    wr = s_ring_wr;

    /* 消费环形缓冲区中所有未处理字节 */
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

/**
 * @brief UART4 接收完成中断回调。
 *
 * ISR 只做字节搬运到环形缓冲区：
 * 1. 将收到的字节写入 ring[wr]
 * 2. wr 指针前进
 * 3. 启动下一次单字节接收
 *
 * 帧解析放在 VisionApp_Process 中执行，不在 ISR 内。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart4)
    {
        uint16_t next = (s_ring_wr + 1U) % VISION_RING_SIZE;

        /* 环形缓冲区未满则写入，满则丢弃（覆盖保护） */
        if (next != s_ring_rd)
        {
            s_ring[s_ring_wr] = s_rx_byte;
            s_ring_wr = next;
        }

        /* 重新启动单字节接收，保持中断链不断 */
        HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1U);
    }
}
