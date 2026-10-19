#ifndef RTT_DEBUG_H
#define RTT_DEBUG_H

#include <stdint.h>

typedef struct
{
    uint8_t channel;
    const char *buffer_name;
    const char *csv_header;
    uint8_t csv_enable;
    uint16_t csv_div;
} RTTDebugJScope2FConfig_t;

void RTTDebugJScope2FInit(const RTTDebugJScope2FConfig_t *config);
void RTTDebugJScope2FSend(float value0, float value1);

typedef struct
{
    uint8_t channel;
    const char *buffer_name;
    const char *csv_header;
    uint8_t csv_enable;
    uint16_t csv_div;
} RTTDebugJScope6FConfig_t;

void RTTDebugJScope6FInit(const RTTDebugJScope6FConfig_t *config);
void RTTDebugJScope6FSend(float v0, float v1, float v2, float v3, float v4, float v5);

#endif
