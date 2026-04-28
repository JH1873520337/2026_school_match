#ifndef VISION_SERVICE_H
#define VISION_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define VISION_SERVICE_PACKET_SIZE                ((uint16_t)27U)
#define VISION_SERVICE_OFFLINE_TIMEOUT_MS         ((uint32_t)150U)

typedef struct
{
    bool valid;
    bool stale;
    uint8_t quality;
    uint16_t cx;
    uint16_t cy;
    int16_t ex;
    int16_t ey;
    uint16_t area;
} VisionServiceTarget;

typedef struct
{
    bool online;
    uint8_t sequence;
    uint32_t last_update_ms;
    VisionServiceTarget orange;
    VisionServiceTarget flame;
} VisionServiceFrame;

void VisionService_Init(void);
void VisionService_Process(void);
void VisionService_GetLatestFrame(VisionServiceFrame *frame);

#ifdef __cplusplus
}
#endif

#endif
