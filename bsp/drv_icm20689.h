#ifndef __DRV_ICM20689_H
#define __DRV_ICM20689_H

#include <stdint.h>

typedef struct
{
    int16_t acc[3];
    int16_t gyro[3];
} drv_icm20689_raw_t;

void drv_icm20689_init(void);
void drv_icm20689_read_raw(drv_icm20689_raw_t *raw);
uint8_t drv_icm20689_who_am_i(void);

#endif
