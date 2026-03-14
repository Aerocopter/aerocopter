#ifndef FILTERUKF_H
#define FILTERUKF_H

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化UKF滤波器 */
void UKF_Init(float sample_freq);

/* IMU数据更新 */
void UKF_UpdateIMU(float gx, float gy, float gz,
                    float ax, float ay, float az);

/* 获取欧拉角（弧度） */
void UKF_GetEuler(float *roll, float *pitch, float *yaw);

/* 获取陀螺仪零偏估计 */
void UKF_Get_Gyrobias(float *bgx, float *bgy, float *bgz);

#ifdef __cplusplus
}
#endif

#endif
