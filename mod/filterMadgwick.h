#ifndef MADGWICK_FILTER_H
#define MADGWICK_FILTER_H

/**
 * @brief 初始化 Madgwick 滤波器
 * @param sample_freq 采样频率 (Hz)
 * @param beta        滤波器增益（通常 0.1 左右），若传入 0 则使用默认值 betaDef=0.1f
 */
void Madgwick_Init(float sample_freq);

/**
 * @brief IMU 更新（仅加速度计 + 陀螺仪）
 * @param gx, gy, gz 陀螺仪 (rad/s)
 * @param ax, ay, az 加速度计 (m/s²)
 */
void Madgwick_UpdateIMU(float gx, float gy, float gz,
                        float ax, float ay, float az);

/**
 * @brief AHRS 更新（加速度计 + 陀螺仪 + 磁力计）
 * @param gx, gy, gz 陀螺仪 (rad/s)
 * @param ax, ay, az 加速度计 (m/s²)
 * @param mx, my, mz 磁力计 (任意单位，归一化前即可)
 */
void Madgwick_UpdateAHRS(float gx, float gy, float gz,
                         float ax, float ay, float az,
                         float mx, float my, float mz);

/**
 * @brief 获取欧拉角（弧度）
 * @param roll  滚转角地址 (输出)
 * @param pitch 俯仰角地址 (输出)
 * @param yaw   偏航角地址 (输出)
 */
void Madgwick_GetEuler(float *roll, float *pitch, float *yaw);

/**
 * @brief 获取四元数
 */
void Madgwick_GetQuaternion(float *q0, float *q1, float *q2, float *q3);

#endif /* MADGWICK_FILTER_H */
