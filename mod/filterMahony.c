#include "filterMahony.h"
#include <math.h>

/* ===== 可调参数 ===== */
#define twoKpDef    (1.0f)   /* 比例增益 (2 * Kp) */
#define twoKiDef    (0.0f)   /* 积分增益 (2 * Ki)，0 表示禁用积分 */

/* ===== 内部状态 ===== */
static float twoKi;                 /* 实际使用的积分增益 (2*Ki) */
static float q0, q1, q2, q3;        /* 四元数，描述传感器坐标系到世界坐标系的旋转 */
static float integralFBx, integralFBy, integralFBz; /* 积分误差项 (已乘 Ki) */
static float invSampleFreq;          /* 采样周期的倒数 (1/fs) */
static int   anglesComputed;          /* 欧拉角是否已计算标志 */
static float roll, pitch, yaw;        /* 缓存的欧拉角 */

/* ===== 快速逆平方根 (Quake III 算法) ===== */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;               // 将浮点数按整数解释
    i = 0x5f3759df - (i >> 1);          // 初始猜测
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));    // 一次牛顿迭代
    return y;
}

/* ===== 初始化 ===== */
void Mahony_Init(float sample_freq)
{
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;
    integralFBx = integralFBy = integralFBz = 0.0f;
    anglesComputed = 0;
    invSampleFreq = 1.0f / sample_freq;
}

/* ===== 从传感器数据初始化姿态（减少启动收敛时间） ===== */
void Mahony_InitFromSensors(float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_roll, init_pitch, init_yaw;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    // 归一化加速度计
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 归一化磁力计（如果有效）
    if (!(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
    }

    // 从加速度计计算初始俯仰角和滚转角
    init_pitch = atan2f(-ax, az);               // 注意：标准公式可能与坐标系有关
    init_roll  = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    sin_pitch = sinf(init_pitch);
    cos_pitch = cosf(init_pitch);

    // 从磁力计计算初始偏航角（如果有磁力计）
    if (!(mx == 0.0f && my == 0.0f && mz == 0.0f)) {
        magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
        magY = my * cos_roll - mz * sin_roll;
        init_yaw = atan2f(-magY, magX);
    } else {
        init_yaw = 0.0f;
    }

    // 将欧拉角转换为四元数
    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0 = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1 = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2 = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3 = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // 归一化四元数
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    anglesComputed = 0;
}

/* ===== 内部函数：计算欧拉角（从四元数）===== */
static void ComputeAngles(void)
{
    roll  = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    pitch = asinf(2.0f * (q0*q2 - q3*q1));
    yaw   = atan2f(2.0f * (q1*q2 + q0*q3), 1.0f - 2.0f * (q2*q2 + q3*q3));
    anglesComputed = 1;
}

/* ===== IMU 更新（无磁力计）===== */
void Mahony_UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 仅在加速度计有效时计算反馈
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        // 归一化加速度计
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向（由当前四元数推算）
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // 误差 = 测量与估计的重力叉积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分反馈（如果启用）
        if (twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * invSampleFreq;
            integralFBy += twoKiDef * halfey * invSampleFreq;
            integralFBz += twoKiDef * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        // 比例反馈
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // 四元数微分方程
    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    anglesComputed = 0;
}

/* ===== AHRS 更新（带磁力计）===== */
void Mahony_UpdateAHRS(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // 如果磁力计无效，退化为 IMU 模式
    if (mx == 0.0f && my == 0.0f && mz == 0.0f) {
        Mahony_UpdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }

    // 仅在加速度计有效时计算反馈
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f)) {
        // 归一化加速度计
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 归一化磁力计
        recipNorm = invSqrt(mx*mx + my*my + mz*mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // 预计算四元数乘积
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // 参考地磁场方向（旋转磁力计到世界坐标系）
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrtf(hx*hx + hy*hy);                // 水平分量幅度
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // 估计重力方向（由四元数推算）
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;

        // 估计地磁场方向（由参考磁场和四元数推算）
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // 总误差 = 重力叉积 + 地磁叉积
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // 积分反馈
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * invSampleFreq;
            integralFBy += twoKi * halfey * invSampleFreq;
            integralFBz += twoKi * halfez * invSampleFreq;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        // 比例反馈
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // 四元数微分方程
    gx *= (0.5f * invSampleFreq);
    gy *= (0.5f * invSampleFreq);
    gz *= (0.5f * invSampleFreq);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += ( qa * gx + qc * gz - q3 * gy);
    q2 += ( qa * gy - qb * gz + q3 * gx);
    q3 += ( qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    anglesComputed = 0;
}

/* ===== 获取欧拉角 ===== */
void Mahony_GetEuler(float *roll_out, float *pitch_out, float *yaw_out)
{
    if (!anglesComputed) ComputeAngles();
    if (roll_out)  *roll_out = roll;
    if (pitch_out) *pitch_out = pitch;
    if (yaw_out)   *yaw_out = yaw;
}

/* ===== 获取四元数 ===== */
void Mahony_GetQuaternion(float *q0_out, float *q1_out, float *q2_out, float *q3_out)
{
    if (q0_out) *q0_out = q0;
    if (q1_out) *q1_out = q1;
    if (q2_out) *q2_out = q2;
    if (q3_out) *q3_out = q3;
}
