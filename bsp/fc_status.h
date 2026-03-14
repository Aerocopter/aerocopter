#ifndef __FC_STATUS_H
#define __FC_STATUS_H

#include <stdint.h>

// 飞控状态枚举（根据你的实际状态机调整或扩展）
typedef enum {
    FC_STATUS_POWER_ON = 0,      // 上电初始化
    FC_STATUS_BOOTING,            // 启动中（传感器校准）
    FC_STATUS_STANDBY_NO_GPS,     // 待机，GPS未锁定
    FC_STATUS_STANDBY_GPS_OK,     // 待机，GPS已锁定
    FC_STATUS_ARMED,       // 已解锁，GPS未锁定
    FC_STATUS_UNARMED,       // 已解锁，GPS已锁定
    FC_STATUS_LOW_BATTERY,        // 低电量警告
    FC_STATUS_ERROR,              // 严重错误（如EKF故障）
    FC_STATUS_GPS_LOST,           // GPS丢失
    FC_STATUS_FAILSAFE            // 故障保护
} FC_Status_t;

// 初始化状态管理（如果使用互斥量，需在创建任务前调用）
void FC_Status_Init(void);

// 设置当前飞控状态（由主任务或其他任务调用）
void FC_SetStatus(FC_Status_t new_status);

// 获取当前飞控状态（供LED任务等调用）
FC_Status_t FC_GetStatus(void);

#endif
