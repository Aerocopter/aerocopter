// fc_status.c (裸机版)
#include "fc_status.h"

static volatile FC_Status_t g_fc_status = FC_STATUS_POWER_ON;

void FC_Status_Init(void)
{
    // 空函数，或者用于初始化其他东西
}

void FC_SetStatus(FC_Status_t new_status)
{
    g_fc_status = new_status;
}

FC_Status_t FC_GetStatus(void)
{
    return g_fc_status;
}
