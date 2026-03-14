#ifndef __DRV_ICM42688_H
#define __DRV_ICM42688_H

#include <stdint.h>

/* Raw sensor data */
typedef struct
{
    int16_t acc[3];
    int16_t gyro[3];
} drv_icm42688_raw_t;

/* Initialize sensor */
void drv_icm42688_init(void);

/* Read raw accel and gyro */
void drv_icm42688_read_raw(drv_icm42688_raw_t *raw);

#endif
