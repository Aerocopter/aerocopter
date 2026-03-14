#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H

/**
 * @brief 初始化 Mahony 滤波器
 * @param sample_freq 采样频率 (Hz)
 */
void Mahony_Init(float sample_freq);

/**
 * @brief 从加速度计和磁力计初始姿态（可选，用于减少启动收敛时间）
 * @param ax, ay, az 加速度计测量值 (m/s²)
 * @param mx, my, mz 磁力计测量值 (任意单位，未归一化)
 */
void Mahony_InitFromSensors(float ax, float ay, float az, float mx, float my, float mz);

/**
 * @brief IMU 更新（仅加速度计 + 陀螺仪）
 * @param gx, gy, gz 陀螺仪 (rad/s)
 * @param ax, ay, az 加速度计 (m/s²)
 */
void Mahony_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);

/**
 * @brief AHRS 更新（加速度计 + 陀螺仪 + 磁力计）
 * @param gx, gy, gz 陀螺仪 (rad/s)
 * @param ax, ay, az 加速度计 (m/s²)
 * @param mx, my, mz 磁力计 (任意单位，归一化前即可)
 */
void Mahony_UpdateAHRS(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz);

/**
 * @brief 获取欧拉角（弧度）
 * @param roll  滚转角地址 (输出)
 * @param pitch 俯仰角地址 (输出)
 * @param yaw   偏航角地址 (输出)
 */
void Mahony_GetEuler(float *roll, float *pitch, float *yaw);

/**
 * @brief 获取四元数
 */
void Mahony_GetQuaternion(float *q0, float *q1, float *q2, float *q3);

#endif /* MAHONY_FILTER_H */
