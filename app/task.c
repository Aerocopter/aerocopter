#include "stdio.h"
#include "bsp_uart.h"
#include "bsp_led.h"
#include "task.h"
#include "imu.h"
#include "attitude.h"
#include "telemetry.h"
#include "fc_status.h"
#include "position.h"

static OS_STK imu_stk[TASK_STK_SIZE];
static OS_STK tel_stk[TASK_STK_SIZE];
static OS_STK led_stk[TASK_STK_SIZE];
static OS_STK pos_stk[TASK_STK_SIZE];
static OS_STK flash_stk[TASK_STK_SIZE];

static imu_t imu; 
static float roll, pitch, yaw;
static float pos;

void task_imu(void *pdata)
{

    imu_init();
    imu_calibrate(); 
    attitude_init(ATT_FILTER_MAHONY,100.0f); 

    while (1)
    {
        imu_update(&imu);
        attitude_update(&imu);
        attitude_get(&roll, &pitch, &yaw);

	CoTickDelay(10);
    }
}

void task_pos(void *pdata)
{
    
    position_init(); 
    position_calibrate();

    while (1) {

        pos = position_get_altitude();
	CoTickDelay(100);
    }
}

void task_tel(void *pdata)
{
    while (1)
    {
        printf("%.2f,",pos);
        printf("%.2f,%.2f,%.2f\n", yaw*57.2958f, -pitch*57.2958f, roll*57.2958f);
        CoTickDelay(100);
    }
}

void task_led(void *pdata)
{
    static uint8_t tick = 0;
    BSP_LED_Init(); 
    while (1)
    {
        FC_Status_t status = FC_GetStatus();
        tick = (tick + 1) % 20;

        switch (status)
        {
            case FC_STATUS_POWER_ON:
            case FC_STATUS_BOOTING:
                if (tick < 5)      BSP_LED_SetAll(1,0,1);
                else               BSP_LED_SetAll(0,0,0);
                break;

            case FC_STATUS_ARMED:
                if (tick < 10)     BSP_LED_SetAll(0,1,1);
                else               BSP_LED_SetAll(1,1,1);
                break;

            case FC_STATUS_UNARMED:
                if (tick < 1)      BSP_LED_SetAll(0,1,1);
                else               BSP_LED_SetAll(1,1,1);
                break;

            case FC_STATUS_ERROR:
                if (tick < 5)      BSP_LED_SetAll(1,0,0);
                else               BSP_LED_SetAll(0,0,0);
                break;

            case FC_STATUS_FAILSAFE:
                if (tick % 2)      BSP_LED_SetAll(1,0,1);
                else               BSP_LED_SetAll(1,1,1);
                break;

            default:
                BSP_LED_SetAll(0,0,0);
                break;
        }
	CoTickDelay(80);
    }
}

void task_flash(void *pdata)
{
    while (1)
    {
        CoTickDelay(1000);
    }
}


void task_create(void)
{
    CoCreateTask(task_imu, 0, 1,  &imu_stk[TASK_STK_SIZE - 1], TASK_STK_SIZE);
    CoCreateTask(task_tel, 0, 10, &tel_stk[TASK_STK_SIZE - 1], TASK_STK_SIZE);
    CoCreateTask(task_led, 0, 11, &led_stk[TASK_STK_SIZE - 1], TASK_STK_SIZE);
    CoCreateTask(task_pos, 0, 12, &pos_stk[TASK_STK_SIZE - 1], TASK_STK_SIZE);
    CoCreateTask(task_flash, 0, 13, &flash_stk[TASK_STK_SIZE - 1], TASK_STK_SIZE);
}
