#include "stm32f4xx.h"
#include "main.h"

//system hard file
#include "delay.h"
#include "sys.h"

//free_RTOS hard file
#include "FreeRTOS.h"
#include "task.h"

//DSP
//#include "arm_math.h"

//BSP
#include "led.h"
#include "buzzer.h"
#include "rc.h"
#include "can.h"
#include "laser.h"
#include "power_ctrl.h"
#include "flash.h"
#include "Ambition_data_rule.h"
#include "visual_recognition.h"
#include "usart.h"
#include "stdio.h"
#include "spi.h"
#include "oled.h"
#include "snail_motor.h"

//func hard flie
#include "remote_control.h"
#include "INS_task.h"


//mytask hard file
#include "start_task.h"

#define POWER_CTRL_ONE_BY_ONE_TIME 709
void user_key_init(void);
void BSP_init(void);


flash_data_list_u* flash_read_buffer;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(SysCoreClock);  //初始化延时函数
//此段为flash写0的代码
//	{
//		for(int i = 0; i < 21;i++)
//		{
//			for(int j = 0;j < 3;j++)
//			{
//				flash_read_buffer.flash_data_list.gyro_cali_temp_40_20[i][j] = 0.0f;
//			}
//		}
//		STMFLASH_Write(AMBITION_DATA_DAAR_BEGIN,flash_read_buffer.flash_data_buffer,FLASH_DATA_SIZE);
//	}
	flash_read_buffer = get_flash_union_data_point();
	STMFLASH_Read(AMBITION_DATA_DAAR_BEGIN,flash_read_buffer->flash_data_buffer,FLASH_DATA_SIZE);
	write_flash_to_data(flash_read_buffer);
	BSP_init();
	
	
	
	StartTask();
	vTaskStartScheduler();
	while(1)
		;
}


void user_key_init(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void BSP_init(void)
{
	led_configuration();
	user_key_init();
	laser_configuration();
	remote_control_init();

	power_ctrl_configuration();
	//24v 输出 依次上电
	for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
	{
		power_ctrl_on(i);
		delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
	}
	//CAN接口初始化
	CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
//	usart2_init();
//	uart8_init();//FPV
	TIM8_Init(20000,180);//50hz
	TIM_SetCompare4(TIM8,1500);
	snail_init();
	SPI1Init();
	oled_init();
	button_AD_init();
	

	
}



