#include "vision_app.h"

#include "vision_service.h"

#include <string.h>

static vision_frame_t s_frame;
static uint8_t s_frame_ready;
static uint8_t s_last_online;
static uint8_t s_last_sequence;

static void VisionApp_CopyTarget(vision_target_t *dst, const VisionServiceTarget *src)
{
    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    dst->valid = src->valid ? 1U : 0U;
    dst->stale = src->stale ? 1U : 0U;
    dst->quality = src->quality;
    dst->cx = src->cx;
    dst->cy = src->cy;
    dst->ex = src->ex;
    dst->ey = src->ey;
    dst->area = src->area;
}

static void VisionApp_CopyFrame(vision_frame_t *dst, const VisionServiceFrame *src)
{
    if ((dst == NULL) || (src == NULL))
    {
        return;
    }

    dst->online = src->online ? 1U : 0U;
    dst->sequence = src->sequence;
    dst->crc_ok = src->online ? 1U : 0U;

    VisionApp_CopyTarget(&dst->orange, &src->orange);
    VisionApp_CopyTarget(&dst->flame, &src->flame);
}

static uint8_t VisionApp_HasNewPacket(const VisionServiceFrame *frame)
{
    if ((frame == NULL) || (!frame->online))
    {
        return 0U;
    }

    if (s_last_online == 0U)
    {
        return 1U;
    }

    return (frame->sequence != s_last_sequence) ? 1U : 0U;
}

void VisionApp_Init(void)
{
    memset(&s_frame, 0, sizeof(s_frame));
    s_frame_ready = 0U;
    s_last_online = 0U;
    s_last_sequence = 0U;

    VisionService_Init();
}

void VisionApp_Process(void)
{
    VisionServiceFrame service_frame;
    uint32_t frame_count_snapshot;
    uint8_t has_new_packet;

    VisionService_Process();
    VisionService_GetLatestFrame(&service_frame);

    has_new_packet = VisionApp_HasNewPacket(&service_frame);
    frame_count_snapshot = s_frame.frame_count;

    VisionApp_CopyFrame(&s_frame, &service_frame);
    s_frame.frame_count = frame_count_snapshot;

    if (has_new_packet != 0U)
    {
        s_frame.frame_count++;
        s_frame_ready = 1U;
    }

    s_last_online = s_frame.online;
    s_last_sequence = s_frame.sequence;
}

const vision_frame_t *VisionApp_GetFrame(void)
{
    return &s_frame;
}

const vision_target_t *VisionApp_GetOrangeTarget(void)
{
    return &s_frame.orange;
}

const vision_target_t *VisionApp_GetFlameTarget(void)
{
    return &s_frame.flame;
}

uint8_t VisionApp_IsNewFrame(void)
{
    uint8_t ready = s_frame_ready;
    s_frame_ready = 0U;
    return ready;
}
