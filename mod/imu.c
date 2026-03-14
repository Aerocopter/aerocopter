#include "imu.h"
#include "delay.h"   // ← 新增：用于 delay_ms()

#define IMU_TYPE_ICM20689   1
#define IMU_TYPE_ICM42688   2

#define IMU_TYPE            IMU_TYPE_ICM42688 

#if (IMU_TYPE == IMU_TYPE_ICM20689)
#include "drv_icm20689.h"
#elif (IMU_TYPE == IMU_TYPE_ICM42688)
#include "drv_icm42688.h"
#endif
#include "drv_ist8310.h"

#if (IMU_TYPE == IMU_TYPE_ICM20689)

/* ±2000 dps */
#define GYRO_SCALE   (0.0174532925f / 16.4f)
/* ±16g */
#define ACC_SCALE    (9.80665f / 2048.0f)

#elif (IMU_TYPE == IMU_TYPE_ICM42688)

/* 42688 ±2000 dps */
#define GYRO_SCALE   (0.0174532925f / 16.4f)
/* 42688 ±16g */
#define ACC_SCALE    (9.80665f / 2048.0f)

#endif

static float gyro_offset[3] = {0};
static float acc_offset[3]  = {0};
static float mag_offset[3]  = {0};

void imu_init(void)
{
#if (IMU_TYPE == IMU_TYPE_ICM20689)
    drv_icm20689_init();
#elif (IMU_TYPE == IMU_TYPE_ICM42688)
    drv_icm42688_init();
#endif

    drv_ist8310_init();
}

void imu_update(imu_t *data)
{
    if(data == 0)
        return;

#if (IMU_TYPE == IMU_TYPE_ICM20689)

    drv_icm20689_raw_t raw;
    drv_icm20689_read_raw(&raw);

#elif (IMU_TYPE == IMU_TYPE_ICM42688)

    drv_icm42688_raw_t raw;
    drv_icm42688_read_raw(&raw);

#endif

    int16_t mag_raw[3];
    drv_ist8310_read_raw(mag_raw);

    float ax = raw.acc[0] * ACC_SCALE - acc_offset[0];
    float ay = raw.acc[1] * ACC_SCALE - acc_offset[1];
    float az = raw.acc[2] * ACC_SCALE - acc_offset[2];

    float gx = raw.gyro[0] * GYRO_SCALE - gyro_offset[0];
    float gy = raw.gyro[1] * GYRO_SCALE - gyro_offset[1];
    float gz = raw.gyro[2] * GYRO_SCALE - gyro_offset[2];

    // 坐标系转换（根据硬件安装）
    data->acc[0] = ax;
    data->acc[1] = ay;
    data->acc[2] = az;

    data->gyro[0] = gx;
    data->gyro[1] = gy;
    data->gyro[2] = gz;

    // 磁力计坐标系转换（与IMU对齐）
    data->mag[0] = mag_raw[0];
    data->mag[1] = mag_raw[1];
    data->mag[2] = mag_raw[2];
}

/* ================= Calibration ================= */

void imu_calibrate(void)
{
    const int samples = 500;

    int16_t mag_raw[3];
    long gx = 0, gy = 0, gz = 0;
    long ax = 0, ay = 0, az = 0;
    long mx = 0, my = 0, mz = 0;

#if (IMU_TYPE == IMU_TYPE_ICM20689)

    drv_icm20689_raw_t raw;
    drv_icm20689_read_raw(&raw);

#elif (IMU_TYPE == IMU_TYPE_ICM42688)

    drv_icm42688_raw_t raw;
    drv_icm42688_read_raw(&raw);

#endif

    for(int i = 0; i < samples; i++)
    {
#if (IMU_TYPE == IMU_TYPE_ICM20689)
        drv_icm20689_read_raw(&raw);
#elif (IMU_TYPE == IMU_TYPE_ICM42688)
        drv_icm42688_read_raw(&raw);
#endif

        drv_ist8310_read_raw(mag_raw);

        gx += raw.gyro[0];
        gy += raw.gyro[1];
        gz += raw.gyro[2];

        ax += raw.acc[0];
        ay += raw.acc[1];
        az += raw.acc[2];

        mx += mag_raw[0];
        my += mag_raw[1];
        mz += mag_raw[2];

        delay_ms(1); 
    }

    gyro_offset[0] = (gx / (float)samples) * GYRO_SCALE;
    gyro_offset[1] = (gy / (float)samples) * GYRO_SCALE;
    gyro_offset[2] = (gz / (float)samples) * GYRO_SCALE;

    acc_offset[0] = (ax / (float)samples) * ACC_SCALE;
    acc_offset[1] = (ay / (float)samples) * ACC_SCALE;
    acc_offset[2] = ((az / (float)samples) * ACC_SCALE) - 9.80665f;
    
    mag_offset[0] = (mx / (float)samples);
    mag_offset[1] = (my / (float)samples);
    mag_offset[2] = (mz / (float)samples);
}

/* ================= Offset Get ================= */

void imu_offset_get(imu_t *data)
{
    if(data == 0)
        return;

    data->acc[0]  = acc_offset[0];
    data->acc[1]  = acc_offset[1];
    data->acc[2]  = acc_offset[2];

    data->gyro[0] = gyro_offset[0];
    data->gyro[1] = gyro_offset[1];
    data->gyro[2] = gyro_offset[2];
    
    data->mag[0] = mag_offset[0];
    data->mag[1] = mag_offset[1];
    data->mag[2] = mag_offset[2];
}
