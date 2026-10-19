#include "rtt_debug.h"

#include <stddef.h>
#include "SEGGER_RTT.h"
#include "bsp_dwt.h"

#define RTT_DEBUG_JSCOPE_DEFAULT_CH 1
#define RTT_DEBUG_JSCOPE_DEFAULT_NAME "JScope_t4f4f4"
#define RTT_DEBUG_JSCOPE_BUF_SIZE 1024

static uint8_t jscope_channel = RTT_DEBUG_JSCOPE_DEFAULT_CH;
static uint8_t csv_enable = 0;
static uint16_t csv_div = 1;
static uint16_t csv_count = 0;
static char jscope_buf[RTT_DEBUG_JSCOPE_BUF_SIZE];

#pragma pack(push, 1)
typedef struct
{
    uint32_t time_us;
    float value0;
    float value1;
} RTTDebugJScope2FPacket_t;
#pragma pack(pop)

void RTTDebugJScope2FInit(const RTTDebugJScope2FConfig_t *config)
{
    const char *buffer_name = RTT_DEBUG_JSCOPE_DEFAULT_NAME;

    jscope_channel = RTT_DEBUG_JSCOPE_DEFAULT_CH;
    csv_enable = 0;
    csv_div = 1;
    csv_count = 0;

    if (config != NULL)
    {
        jscope_channel = config->channel;
        csv_enable = config->csv_enable;
        csv_div = config->csv_div == 0 ? 1 : config->csv_div;
        if (config->buffer_name != NULL)
            buffer_name = config->buffer_name;
    }

    SEGGER_RTT_ConfigUpBuffer(jscope_channel,
                              buffer_name,
                              jscope_buf,
                              sizeof(jscope_buf),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    if (csv_enable && config != NULL && config->csv_header != NULL)
        SEGGER_RTT_printf(0, "%s\r\n", config->csv_header);
}

void RTTDebugJScope2FSend(float value0, float value1)
{
    RTTDebugJScope2FPacket_t pkt;

    pkt.time_us = (uint32_t)DWT_GetTimeline_us();
    pkt.value0 = value0;
    pkt.value1 = value1;

    SEGGER_RTT_Write(jscope_channel, &pkt, sizeof(pkt));

    if (csv_enable && ++csv_count >= csv_div)
    {
        csv_count = 0;
        SEGGER_RTT_printf(0, "%u,%d,%d\r\n",
                          (unsigned int)pkt.time_us,
                          (int)(value0 * 100.0f),
                          (int)(value1 * 100.0f));
    }
}

#define RTT_DEBUG_JSCOPE6_DEFAULT_CH 2
#define RTT_DEBUG_JSCOPE6_DEFAULT_NAME "JScope_t4f4f4f4f4f4f4"
#define RTT_DEBUG_JSCOPE6_BUF_SIZE 1024

static uint8_t jscope6_channel = RTT_DEBUG_JSCOPE6_DEFAULT_CH;
static uint8_t jscope6_csv_enable = 0;
static uint16_t jscope6_csv_div = 1;
static uint16_t jscope6_csv_count = 0;
static char jscope6_buf[RTT_DEBUG_JSCOPE6_BUF_SIZE];

#pragma pack(push, 1)
typedef struct
{
    uint32_t time_us;
    float v0;
    float v1;
    float v2;
    float v3;
    float v4;
    float v5;
} RTTDebugJScope6FPacket_t;
#pragma pack(pop)

void RTTDebugJScope6FInit(const RTTDebugJScope6FConfig_t *config)
{
    const char *buffer_name = RTT_DEBUG_JSCOPE6_DEFAULT_NAME;

    jscope6_channel = RTT_DEBUG_JSCOPE6_DEFAULT_CH;
    jscope6_csv_enable = 0;
    jscope6_csv_div = 1;
    jscope6_csv_count = 0;

    if (config != NULL)
    {
        jscope6_channel = config->channel;
        jscope6_csv_enable = config->csv_enable;
        jscope6_csv_div = config->csv_div == 0 ? 1 : config->csv_div;
        if (config->buffer_name != NULL)
            buffer_name = config->buffer_name;
    }

    SEGGER_RTT_ConfigUpBuffer(jscope6_channel,
                              buffer_name,
                              jscope6_buf,
                              sizeof(jscope6_buf),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    if (jscope6_csv_enable && config != NULL && config->csv_header != NULL)
        SEGGER_RTT_printf(0, "%s\r\n", config->csv_header);
}

void RTTDebugJScope6FSend(float v0, float v1, float v2, float v3, float v4, float v5)
{
    RTTDebugJScope6FPacket_t pkt;

    pkt.time_us = (uint32_t)DWT_GetTimeline_us();
    pkt.v0 = v0;
    pkt.v1 = v1;
    pkt.v2 = v2;
    pkt.v3 = v3;
    pkt.v4 = v4;
    pkt.v5 = v5;

    SEGGER_RTT_Write(jscope6_channel, &pkt, sizeof(pkt));

    if (jscope6_csv_enable && ++jscope6_csv_count >= jscope6_csv_div)
    {
        jscope6_csv_count = 0;
        SEGGER_RTT_printf(0, "%u,%d,%d,%d,%d,%d,%d\r\n",
                          (unsigned int)pkt.time_us,
                          (int)(v0 * 100.0f),
                          (int)(v1 * 100.0f),
                          (int)(v2 * 100.0f),
                          (int)(v3 * 100.0f),
                          (int)(v4 * 100.0f),
                          (int)(v5 * 100.0f));
    }
}
