#include "stdio.h"
#include "stdint.h"
#include "filterUKF.h"
#include <math.h>

#define ALPHA 0.8f
#define BETA  2.0f
#define KAPPA 2.0f

#define UKF_N 7          // 状态维度: 四元数(4) + 陀螺零偏(3)
#define UKF_M 3          // 测量维度: 加速度计(3)
#define UKF_SIGMA (2*UKF_N+1)

/* ===== 内部数据结构 ===== */
typedef struct
{
    float x[UKF_N];                     // 状态向量 [q0,q1,q2,q3, bgx,bgy,bgz]
    float P[UKF_N][UKF_N];              // 状态协方差
    float Q[UKF_N][UKF_N];              // 过程噪声
    float R[UKF_M][UKF_M];              // 测量噪声
    
    // 输出的欧拉角
    float roll;
    float pitch;
    float yaw;
} ukf_context_t;

/* ===== 静态变量 ===== */
static ukf_context_t ukf;
static float dt = 0.01f;
static float lambda;
static float wm[UKF_SIGMA];
static float wc[UKF_SIGMA];
static int initialized = 0;

/* ===== 内部工具函数 ===== */
static void quat_normalize(float *q)
{
    float norm_sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    if (norm_sq < 1e-12f) return;
    
    float n = 1.0f / sqrtf(norm_sq);
    for(int i = 0; i < 4; i++) q[i] *= n;
}

static void matrix_add(float A[UKF_N][UKF_N], float B[UKF_N][UKF_N])
{
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_N; j++) {
            A[i][j] += B[i][j];
        }
    }
}

static void cholesky(float A[UKF_N][UKF_N], float L[UKF_N][UKF_N])
{
    // 手动初始化L矩阵为0
    for(uint8_t i = 0; i < UKF_N; i++) {
        for(uint8_t j = 0; j < UKF_N; j++) {
            L[i][j] = 0.0f;
        }
    }

    for(uint8_t i = 0; i < UKF_N; i++)
    {
        for(uint8_t j = 0; j <= i; j++)
        {
            float sum = 0;
            for(uint8_t k = 0; k < j; k++)
                sum += L[i][k] * L[j][k];

            if(i == j) {
                float val = A[i][i] - sum;
                L[i][j] = (val > 0) ? sqrtf(val) : 0;
            } else {
                if (L[j][j] > 1e-12f)
                    L[i][j] = (A[i][j] - sum) / L[j][j];
                else
                    L[i][j] = 0;
            }
        }
    }
}

static void process_model(float *x, float gx, float gy, float gz, float dt)
{
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float bgx = x[4], bgy = x[5], bgz = x[6];

    // 减去零偏
    gx -= bgx;
    gy -= bgy;
    gz -= bgz;

    float halfdt = 0.5f * dt;

    // 四元数更新
    float new_q0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfdt;
    float new_q1 = q1 + ( q0*gx + q2*gz - q3*gy) * halfdt;
    float new_q2 = q2 + ( q0*gy - q1*gz + q3*gx) * halfdt;
    float new_q3 = q3 + ( q0*gz + q1*gy - q2*gx) * halfdt;

    x[0] = new_q0;
    x[1] = new_q1;
    x[2] = new_q2;
    x[3] = new_q3;

    quat_normalize(x);
    // 零偏保持不变
}

static void measurement_model(float *x, float *z)
{
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];

    // 重力向量在机体坐标系下的投影
    z[0] = 2.0f * (q1*q3 - q0*q2);
    z[1] = 2.0f * (q0*q1 + q2*q3);
    z[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

#if 0
/* ===== 初始化UKF权重 ===== */
static void init_weights(void)
{
    lambda = ALPHA * ALPHA * (UKF_N + KAPPA) - UKF_N;

    wm[0] = lambda / (UKF_N + lambda);
    wc[0] = wm[0] + (1 - ALPHA*ALPHA + BETA);

    for(int i = 1; i < UKF_SIGMA; i++) {
        wm[i] = 1.0f / (2.0f * (UKF_N + lambda));
        wc[i] = wm[i];
    }
}
#endif

static void init_weights(void)
{
    // 增大lambda以扩大Sigma点分布
    // lambda = α²(n+κ) - n

    // 原公式
    // lambda = ALPHA * ALPHA * (UKF_N + KAPPA) - UKF_N;

    // 修改为动态lambda，前期更大
    lambda = ALPHA * ALPHA * (UKF_N + KAPPA) - UKF_N;

    // 如果lambda太小，强制设定最小值
    float n_lambda = UKF_N + lambda;
    if (n_lambda < 0.1f) {
        lambda = 0.1f - UKF_N;  // 确保分母不会太小
    }

    wm[0] = lambda / (UKF_N + lambda);
    wc[0] = wm[0] + (1 - ALPHA*ALPHA + BETA);

    for(int i = 1; i < UKF_SIGMA; i++) {
        wm[i] = 1.0f / (2.0f * (UKF_N + lambda));
        wc[i] = wm[i];
    }
}

/* ===== 对外接口 ===== */

void UKF_Init(float sample_freq)
{
    if (sample_freq > 0) {
        dt = 1.0f / sample_freq;
    }

    // 手动初始化ukf结构体，不使用memset
    
    // 初始化状态向量
    ukf.x[0] = 1.0f;  // q0
    ukf.x[1] = 0.0f;  // q1
    ukf.x[2] = 0.0f;  // q2
    ukf.x[3] = 0.0f;  // q3
    ukf.x[4] = 0.0f;  // bgx
    ukf.x[5] = 0.0f;  // bgy
    ukf.x[6] = 0.0f;  // bgz
    
    ukf.roll = 0.0f;
    ukf.pitch = 0.0f;
    ukf.yaw = 0.0f;

    // 初始化协方差矩阵P
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_N; j++) {
            if (i == j) {
                ukf.P[i][j] = 0.01f;      // 对角线
            } else {
                ukf.P[i][j] = 0.0f;        // 非对角线
            }
        }
    }

    // 初始化过程噪声矩阵Q
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_N; j++) {
            if (i == j) {
                ukf.Q[i][j] = 1e-6f;       // 对角线
            } else {
                ukf.Q[i][j] = 0.0f;        // 非对角线
            }
        }
    }

    // 初始化测量噪声矩阵R
    for(int i = 0; i < UKF_M; i++) {
        for(int j = 0; j < UKF_M; j++) {
            ukf.R[i][j] = 0.0f;             // 先全设为0
        }
    }
    ukf.R[0][0] = 0.01f;
    ukf.R[1][1] = 0.01f;
    ukf.R[2][2] = 0.01f;

    // 初始化UKF权重
    init_weights();
    
    initialized = 1;
}

void UKF_UpdateIMU(float gx, float gy, float gz,
                    float ax, float ay, float az)
{
    if (!initialized || dt <= 0) return;
    
    float sigma[UKF_SIGMA][UKF_N];
    float L[UKF_N][UKF_N];

    // 步骤1: Cholesky分解
    cholesky(ukf.P, L);

    // 步骤2: 生成Sigma点
    // 手动复制状态到第一个Sigma点
    for(int j = 0; j < UKF_N; j++) {
        sigma[0][j] = ukf.x[j];
    }

    float gamma = sqrtf(UKF_N + lambda);

    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_N; j++) {
            sigma[i+1][j]          = ukf.x[j] + gamma * L[j][i];
            sigma[i+1+UKF_N][j]    = ukf.x[j] - gamma * L[j][i];
        }
    }

    // 步骤3: 传播Sigma点
    for(int i = 0; i < UKF_SIGMA; i++) {
        process_model(sigma[i], gx, gy, gz, dt);
    }

    // 步骤4: 计算预测状态均值
    float x_pred[UKF_N];
    // 初始化x_pred为0
    for(int j = 0; j < UKF_N; j++) {
        x_pred[j] = 0.0f;
    }
    
    for(int i = 0; i < UKF_SIGMA; i++) {
        for(int j = 0; j < UKF_N; j++) {
            x_pred[j] += wm[i] * sigma[i][j];
        }
    }
    quat_normalize(x_pred);

    // 步骤5: 计算预测协方差
    float P_pred[UKF_N][UKF_N];
    // 初始化P_pred为0
    for(int r = 0; r < UKF_N; r++) {
        for(int c = 0; c < UKF_N; c++) {
            P_pred[r][c] = 0.0f;
        }
    }
    
    for(int i = 0; i < UKF_SIGMA; i++) {
        float dx[UKF_N];
        for(int j = 0; j < UKF_N; j++) {
            dx[j] = sigma[i][j] - x_pred[j];
        }

        for(int r = 0; r < UKF_N; r++) {
            for(int c = 0; c < UKF_N; c++) {
                P_pred[r][c] += wc[i] * dx[r] * dx[c];
            }
        }
    }
    matrix_add(P_pred, ukf.Q);

    // 步骤6: 归一化加速度计
    float acc_norm = sqrtf(ax*ax + ay*ay + az*az);
    if (acc_norm > 1e-6f) {
        ax /= acc_norm;
        ay /= acc_norm;
        az /= acc_norm;
    }

    // 步骤7: 测量更新
    float z_sigma[UKF_SIGMA][UKF_M];
    for(int i = 0; i < UKF_SIGMA; i++) {
        measurement_model(sigma[i], z_sigma[i]);
    }

    // 计算预测测量均值
    float z_pred[UKF_M];
    // 初始化z_pred为0
    for(int j = 0; j < UKF_M; j++) {
        z_pred[j] = 0.0f;
    }
    
    for(int i = 0; i < UKF_SIGMA; i++) {
        for(int j = 0; j < UKF_M; j++) {
            z_pred[j] += wm[i] * z_sigma[i][j];
        }
    }

    // 计算新息协方差S和互协方差Pxz
    float S[UKF_M][UKF_M];
    float Pxz[UKF_N][UKF_M];
    
    // 初始化S和Pxz为0
    for(int r = 0; r < UKF_M; r++) {
        for(int c = 0; c < UKF_M; c++) {
            S[r][c] = 0.0f;
        }
    }
    for(int r = 0; r < UKF_N; r++) {
        for(int c = 0; c < UKF_M; c++) {
            Pxz[r][c] = 0.0f;
        }
    }

    for(int i = 0; i < UKF_SIGMA; i++) {
        float dz[UKF_M];
        float dx[UKF_N];

        for(int j = 0; j < UKF_M; j++) {
            dz[j] = z_sigma[i][j] - z_pred[j];
        }
        for(int j = 0; j < UKF_N; j++) {
            dx[j] = sigma[i][j] - x_pred[j];
        }

        for(int r = 0; r < UKF_M; r++) {
            for(int c = 0; c < UKF_M; c++) {
                S[r][c] += wc[i] * dz[r] * dz[c];
            }
        }
        for(int r = 0; r < UKF_N; r++) {
            for(int c = 0; c < UKF_M; c++) {
                Pxz[r][c] += wc[i] * dx[r] * dz[c];
            }
        }
    }

    // 添加测量噪声
    for(int i = 0; i < UKF_M; i++) {
        S[i][i] += ukf.R[i][i];
    }

    // 计算卡尔曼增益
    float K[UKF_N][UKF_M];
    // 初始化K为0
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_M; j++) {
            K[i][j] = 0.0f;
        }
    }
    
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_M; j++) {
            if (S[j][j] > 1e-12f) {
                K[i][j] = Pxz[i][j] / S[j][j];
            }
        }
    }

    // 更新状态
    float z[3] = {ax, ay, az};
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_M; j++) {
            x_pred[i] += K[i][j] * (z[j] - z_pred[j]);
        }
    }
    quat_normalize(x_pred);

    // 更新协方差
    for(int i = 0; i < UKF_N; i++) {
        for(int j = 0; j < UKF_N; j++) {
            float sum = 0;
            for(int k = 0; k < UKF_M; k++) {
                sum += K[i][k] * S[k][k] * K[j][k];
            }
            P_pred[i][j] -= sum;
        }
    }

    // 保存结果 - 手动复制
    for(int j = 0; j < UKF_N; j++) {
        ukf.x[j] = x_pred[j];
    }
    for(int r = 0; r < UKF_N; r++) {
        for(int c = 0; c < UKF_N; c++) {
            ukf.P[r][c] = P_pred[r][c];
        }
    }

    // 计算欧拉角
    float q0 = ukf.x[0];
    float q1 = ukf.x[1];
    float q2 = ukf.x[2];
    float q3 = ukf.x[3];

    ukf.roll  = atan2f(2.0f * (q0*q1 + q2*q3), 
                       1.0f - 2.0f * (q1*q1 + q2*q2));
    ukf.pitch = asinf(2.0f * (q0*q2 - q3*q1));
    ukf.yaw   = atan2f(2.0f * (q0*q3 + q1*q2), 
                       1.0f - 2.0f * (q2*q2 + q3*q3));
}

void UKF_GetEuler(float *roll, float *pitch, float *yaw)
{
    if (roll) *roll = ukf.roll;
    if (pitch) *pitch = ukf.pitch;
    if (yaw) *yaw = ukf.yaw;
}

void UKF_Get_Gyrobias(float *bgx, float *bgy, float *bgz)
{
    if (bgx) *bgx = ukf.x[4];
    if (bgy) *bgy = ukf.x[5];
    if (bgz) *bgz = ukf.x[6];
}
