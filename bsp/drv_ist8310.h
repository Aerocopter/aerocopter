#ifndef __DRV_IST8310_H
#define __DRV_IST8310_H

#include <stdint.h>

typedef struct
{
    int16_t mag_raw[3];   // 原始磁场
    float   mag[3];       // 校准后磁场（可选）
    uint8_t healthy;
} ist8310_t;

uint8_t drv_ist8310_detect(void);
uint8_t drv_ist8310_init(void);
void drv_ist8310_read_raw(int16_t *mag);
void drv_ist8310_update(void);

extern ist8310_t ist8310;

#endif
