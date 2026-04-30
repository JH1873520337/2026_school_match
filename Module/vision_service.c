#include "vision_service.h"

#include "camera_uart.h"

#include <string.h>

#define VISION_PACKET_HEADER_0                   ((uint8_t)0xAAU)
#define VISION_PACKET_HEADER_1                   ((uint8_t)0x55U)
#define VISION_PROCESS_MAX_BYTES_PER_CALL        ((uint16_t)64U)

static uint8_t s_packet_buffer[VISION_SERVICE_PACKET_SIZE];
static uint16_t s_packet_index;
static VisionServiceFrame s_latest_frame;

static void VisionService_Lock(uint32_t *primask)
{
    if (primask == NULL)
    {
        return;
    }

    *primask = __get_PRIMASK();
    __disable_irq();
}

static void VisionService_Unlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

static void VisionService_ResetTarget(VisionServiceTarget *target)
{
    if (target == NULL)
    {
        return;
    }

    target->valid = false;
    target->stale = false;
    target->quality = 0U;
    target->cx = 0U;
    target->cy = 0U;
    target->ex = 0;
    target->ey = 0;
    target->area = 0U;
}

static void VisionService_ResetFrame(VisionServiceFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    frame->online = false;
    frame->sequence = 0U;
    frame->last_update_ms = 0U;
    VisionService_ResetTarget(&frame->orange);
    VisionService_ResetTarget(&frame->flame);
}

static uint16_t VisionService_ParseU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t VisionService_ParseI16(const uint8_t *data)
{
    return (int16_t)VisionService_ParseU16(data);
}

static uint8_t VisionService_Crc8Xor(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    uint8_t crc = 0U;

    for (index = 0U; index < length; index++)
    {
        crc ^= data[index];
    }

    return crc;
}

static void VisionService_DecodeTarget(VisionServiceTarget *target,
                                       const uint8_t *payload,
                                       bool valid,
                                       bool stale)
{
    if ((target == NULL) || (payload == NULL))
    {
        return;
    }

    target->valid = valid;
    target->stale = stale;
    target->quality = payload[0];
    target->cx = VisionService_ParseU16(&payload[1]);
    target->cy = VisionService_ParseU16(&payload[3]);
    target->ex = VisionService_ParseI16(&payload[5]);
    target->ey = VisionService_ParseI16(&payload[7]);
    target->area = VisionService_ParseU16(&payload[9]);

    if (!valid)
    {
        target->quality = 0U;
        target->cx = 0U;
        target->cy = 0U;
        target->ex = 0;
        target->ey = 0;
        target->area = 0U;
    }
}

static void VisionService_PublishPacket(const uint8_t *packet)
{
    VisionServiceFrame frame;
    uint8_t flags;

    if (packet == NULL)
    {
        return;
    }

    VisionService_ResetFrame(&frame);

    flags = packet[3];
    frame.online = true;
    frame.sequence = packet[2];
    frame.last_update_ms = HAL_GetTick();

    VisionService_DecodeTarget(&frame.orange,
                               &packet[4],
                               ((flags & 0x01U) != 0U),
                               ((flags & 0x02U) != 0U));

    VisionService_DecodeTarget(&frame.flame,
                               &packet[15],
                               ((flags & 0x04U) != 0U),
                               ((flags & 0x08U) != 0U));

    {
        uint32_t primask;
        VisionService_Lock(&primask);
    s_latest_frame = frame;
        VisionService_Unlock(primask);
    }
}

static void VisionService_HandlePacketByte(uint8_t byte)
{
    if (s_packet_index == 0U)
    {
        if (byte == VISION_PACKET_HEADER_0)
        {
            s_packet_buffer[s_packet_index++] = byte;
        }
        return;
    }

    if (s_packet_index == 1U)
    {
        if (byte == VISION_PACKET_HEADER_1)
        {
            s_packet_buffer[s_packet_index++] = byte;
            return;
        }

        s_packet_index = (byte == VISION_PACKET_HEADER_0) ? 1U : 0U;
        if (s_packet_index == 1U)
        {
            s_packet_buffer[0] = byte;
        }
        return;
    }

    s_packet_buffer[s_packet_index++] = byte;
    if (s_packet_index < VISION_SERVICE_PACKET_SIZE)
    {
        return;
    }

    s_packet_index = 0U;
    if (VisionService_Crc8Xor(&s_packet_buffer[2], VISION_SERVICE_PACKET_SIZE - 3U) !=
        s_packet_buffer[VISION_SERVICE_PACKET_SIZE - 1U])
    {
        return;
    }

    VisionService_PublishPacket(s_packet_buffer);
}

static void VisionService_UpdateOfflineState(void)
{
    uint32_t now_ms;

    {
        uint32_t primask;
        VisionService_Lock(&primask);
    if (!s_latest_frame.online)
    {
        VisionService_Unlock(primask);
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - s_latest_frame.last_update_ms) <= VISION_SERVICE_OFFLINE_TIMEOUT_MS)
    {
        VisionService_Unlock(primask);
        return;
    }

    VisionService_ResetFrame(&s_latest_frame);
        VisionService_Unlock(primask);
    }
}

void VisionService_Init(void)
{
    s_packet_index = 0U;
    memset(s_packet_buffer, 0, sizeof(s_packet_buffer));

    {
        uint32_t primask;
        VisionService_Lock(&primask);
    VisionService_ResetFrame(&s_latest_frame);
        VisionService_Unlock(primask);
    }
}

void VisionService_Process(void)
{
    uint8_t byte;
    uint16_t processed_count = 0U;

    while (processed_count < VISION_PROCESS_MAX_BYTES_PER_CALL)
    {
        if (CameraUart_ReadByte(&byte) != HAL_OK)
        {
            break;
        }

        VisionService_HandlePacketByte(byte);
        processed_count++;
    }

    VisionService_UpdateOfflineState();
}

void VisionService_GetLatestFrame(VisionServiceFrame *frame)
{
    if (frame == NULL)
    {
        return;
    }

    {
        uint32_t primask;
        VisionService_Lock(&primask);
    *frame = s_latest_frame;
        VisionService_Unlock(primask);
    }
}
