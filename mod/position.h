#ifndef __POSITION_H
#define __POSITION_H

#include <stdint.h>

typedef struct
{
    float altitude;      // 高度（米）
    float pressure;      // 气压（mbar）
    float temperature;   // 温度（摄氏度）
    uint8_t healthy;     // 健康状态：0=异常，1=正常
} position_t;

void position_init(void);
void position_update(position_t *data);
void position_calibrate(void);               // 校准：记录参考气压（高度归零）
void position_offset_get(position_t *offset); // 获取校准参数（参考气压）
float position_get_altitude(void);

#endif
