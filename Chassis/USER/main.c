#include "stm32f4xx.h"
#include "main.h"

//system hard file
#include "delay.h"
#include "sys.h"

//free_RTOS hard file
#include "FreeRTOS.h"
#include "task.h"

//BSP
#include "can.h"
#include "usart.h"
#include "led.h"
#include "power_ctrl.h"
#include "spi.h"
#include "oled.h"

//mytask hard file
#include "start_task.h"

#include "ui.h"

uint8_t amb_power = 50;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(SysCoreClock);  //初始化延时函数

	
	
	
	
//BSP_init_begin
			led_configuration();
			//裁判系统接口初始化
			usart2_init();
			//CAN接口初始化
			CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
			CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
//BSP_init_end

	
	
	
	
	
	StartTask();
	vTaskStartScheduler();
	while(1)
		;
}
