#ifndef FILTERCF_H
#define FILTERCF_H

/* 初始化互补滤波器，设置采样频率 (Hz) */
void CF_Init(float sampleFreq);

/* IMU 更新（无磁力计） */
void CF_UpdateIMU(float gx, float gy, float gz,
                  float ax, float ay, float az);

/* AHRS 更新（带磁力计） */
void CF_UpdateAHRS(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz);

/* 获取欧拉角（弧度） */
void CF_GetEuler(float *roll, float *pitch, float *yaw);

#endif
