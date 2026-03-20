/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    27-January-2022
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "bsp_led.h"
#include "bsp_uart.h"
#include "bsp_spi.h"
#include "bsp_iic.h"
#include "task.h"
#include "delay.h"
#include "imu.h"
#include "fc_status.h"
#include "drv_w25q128.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    FC_SetStatus(FC_STATUS_UNARMED);

    bsp_spi1_init(SPI_MODE_0);
    bsp_spi2_init(SPI_MODE_0);
    bsp_spi3_init(SPI_MODE_0);

    uart_init(115200);
    delay_ms(100);   // wait for PC serial ready

    if(w25q128_init())
        printf("w25q128 init failed!\n");

    bsp_i2c3_init();
    
    imu_init();
    imu_calibrate();

    CoInitOS();
    task_create();
    CoStartOS(); 

    while(1);
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

