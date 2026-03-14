#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    float acc[3];   // m/s²
    float gyro[3];  // rad/s
    float mag[3];   // 任意单位（与IMU坐标系对齐）
} imu_t;

void imu_init(void);
void imu_update(imu_t *data);
void imu_calibrate(void);
void imu_get_offsets(imu_t *data);

#endif
