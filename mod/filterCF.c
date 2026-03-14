#include "filterCF.h"
#include <math.h>

/* ===== 可调参数 ===== */
#define Kp 	8.0f    /* 比例增益 */
#define Ki 	0.05f   /* 积分增益 */

/* ===== 内部状态 ===== */
static float T = 0.01f;      /* 采样周期 (秒) */
static float halfT = 0.005f; /* 半采样周期 */

float Yaw, Pitch, Roll;      /* 备用全局变量（未使用）*/

/* 四元数姿态 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
/* 误差积分项 */
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

/* ===== 快速逆平方根 ===== */
static float invSqrt(float number)
{
    volatile long i;
    volatile float x, y;
    volatile const float f = 1.5F;

    x = number * 0.5F;
    y = number;
    i = *((long*)&y);
    i = 0x5f375a86 - (i >> 1);
    y = *((float*)&i);
    y = y * (f - (x * y * y));
    return y;
}

/* ===== 初始化 ===== */
void CF_Init(float sampleFreq)
{
    if (sampleFreq <= 0.0f) return;
    T = 1.0f / sampleFreq;
    halfT = 0.5f * T;
}

/* ===== IMU 更新（仅加速度计+陀螺仪）===== */
void CF_UpdateIMU(float gx, float gy, float gz,
                  float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    /* 保存上一时刻四元数 */
    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;

    /* 归一化加速度计 */
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* 估计重力方向（由当前四元数推算） */
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 误差 = 测量重力与估计重力叉积 */
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    /* 积分误差 */
    exInt += ex * Ki * T;
    eyInt += ey * Ki * T;
    ezInt += ez * Ki * T;

    /* PI 修正陀螺仪 */
    gx += Kp*ex + exInt;
    gy += Kp*ey + eyInt;
    gz += Kp*ez + ezInt;

    /* 一阶四元数积分 */
    q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz) * halfT;
    q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy) * halfT;
    q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx) * halfT;
    q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx) * halfT;

    /* 归一化四元数 */
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

/* ===== AHRS 更新（带磁力计）===== */
void CF_UpdateAHRS(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float mx, float my, float mz)
{
    float norm;
    float vx, vy, vz;          /* 估计重力方向 */
    float wx, wy, wz;          /* 估计磁场方向 */
    float ex, ey, ez;          /* 总误差 */
    float hx, hy, hz;          /* 世界坐标系下的磁场 */
    float bx, bz;              /* 参考磁场（水平/垂直）*/

    float q0_last = q0, q1_last = q1, q2_last = q2, q3_last = q3;

    /* 归一化加速度计 */
    norm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* 坐标系转换 */
    my = -my;

    /* 归一化磁力计 */
    norm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= norm;
    my *= norm;
    mz *= norm;

    /* 估计重力方向 */
    vx = 2*(q1*q3 - q0*q2);
    vy = 2*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    /* 将磁力计旋转到世界坐标系，得到参考磁场方向 */
    hx = 2*mx*(0.5f - q2*q2 - q3*q3) + 2*my*(q1*q2 - q0*q3) + 2*mz*(q1*q3 + q0*q2);
    hy = 2*mx*(q1*q2 + q0*q3) + 2*my*(0.5f - q1*q1 - q3*q3) + 2*mz*(q2*q3 - q0*q1);
    hz = 2*mx*(q1*q3 - q0*q2) + 2*my*(q2*q3 + q0*q1) + 2*mz*(0.5f - q1*q1 - q2*q2);

    /* 参考磁场取水平分量幅度和垂直分量 */
    bx = sqrtf(hx*hx + hy*hy);
    bz = hz;

    /* 将参考磁场旋转回机体坐标系，得到估计的磁场方向 */
    wx = 2*bx*(0.5f - q2*q2 - q3*q3) + 2*bz*(q1*q3 - q0*q2);
    wy = 2*bx*(q1*q2 - q0*q3) + 2*bz*(q0*q1 + q2*q3);
    wz = 2*bx*(q0*q2 + q1*q3) + 2*bz*(0.5f - q1*q1 - q2*q2);

    /* 总误差 = 重力叉积 + 磁场叉积 */
    ex = (ay*vz - az*vy) + (my*wz - mz*wy);
    ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

    /* 积分误差 */
    exInt += ex * Ki * T;
    eyInt += ey * Ki * T;
    ezInt += ez * Ki * T;

    /* PI 修正陀螺仪 */
    gx += Kp*ex + exInt;
    gy += Kp*ey + eyInt;
    gz += Kp*ez + ezInt;

    /* 一阶四元数积分 */
    q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz) * halfT;
    q1 = q1_last + ( q0_last*gx + q2_last*gz - q3_last*gy) * halfT;
    q2 = q2_last + ( q0_last*gy - q1_last*gz + q3_last*gx) * halfT;
    q3 = q3_last + ( q0_last*gz + q1_last*gy - q2_last*gx) * halfT;

    /* 归一化四元数 */
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

/* ===== 获取欧拉角 ===== */
void CF_GetEuler(float *roll, float *pitch, float *yaw)
{
    if (roll)
        *roll = atan2f(2.0f*(q0*q1 + q2*q3),
                       1.0f - 2.0f*(q1*q1 + q2*q2));
    if (pitch)
        *pitch = -asinf(2.0f*(q0*q2 - q3*q1));
    if (yaw)
        *yaw = atan2f(2.0f*(q0*q3 + q1*q2),
                      1.0f - 2.0f*(q2*q2 + q3*q3));
}
