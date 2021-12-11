#include "start_task.h"
#include "mytaskconfig.h"
#include "Ambition_data_rule.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"

//mytask
#include "led.h"
#include "led_task.h"
#include "ins_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "delay.h"

//start task parameter
#define START_TASK_PRIO 1
#define START_STK_SIZE 128
static TaskHandle_t StartTask_Handler;

//led task parameter
#define LED_TASK_PRIO 5
#define LED_TASK_SIZE 512
static TaskHandle_t LEDTask_Handler;

//INS task parameter
#define INS_TASK_PRIO 20
#define INS_TASK_SIZE 512
static TaskHandle_t INSTask_Handler;

//chassis task parameter
#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 256
TaskHandle_t ChassisTask_Handler;

//gimabal task parameter
#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 2048
TaskHandle_t GIMBALTask_Handler;

//shoot task parameter
#define SHOOT_TASK_PRIO 17
#define SHOOT_STK_SIZE 512
TaskHandle_t SHOOTTask_Handler;

#define SYS_INIT_TIME 5000
#define USER_KEY_MAX_TIME 5



void start_task(void *pvParameters)
{

	flash_data_list_t* flash_data_starttask;
	uint16_t load_max_time = 0;
	uint16_t user_key_cnt = 0;
	uint8_t  remaber_user_key_is_press = 0;
	
	flash_data_starttask = get_flash_data_list_point();
	flow_led_on(7);//
	if(flash_data_starttask->board_sate == DEBUG_MODE_C)
	{
		load_max_time = SYS_INIT_TIME;
		flow_led_on(1);
	}
	else if(flash_data_starttask->board_sate == BEGIN_GAME_MODE_C)
	{
		load_max_time = 10;
		flow_led_on(2);
	}
	else if(flash_data_starttask->board_sate == UNDERWAY_GAME_MODE_C)
	{
		load_max_time = 10;
		flow_led_on(3);
	}
	
	
	
	for(int i = 0; i < load_max_time; i++)
	{
		if(user_key_cnt >= USER_KEY_MAX_TIME)
		{
			remaber_user_key_is_press = 1;
		}
		else 
		{
			;
		}
			
		if(remaber_user_key_is_press == 0)
		{
			if(USER_KEY)
			{
				user_key_cnt++;
			}
			else
			{
				user_key_cnt = 0;
			}
		}
		else if(remaber_user_key_is_press == 1)
		{
			flow_led_on(6);
		}
		vTaskDelay(1);
	}
	
	
	
	if(remaber_user_key_is_press == 1)
	{
		if(flash_data_starttask->board_sate == DEBUG_MODE_C)
		{
			flash_data_starttask->board_sate = BEGIN_GAME_MODE_C;
		}
		else if(flash_data_starttask->board_sate == BEGIN_GAME_MODE_C)
		{
			flash_data_starttask->board_sate = DEBUG_MODE_C;
		}
		else if(flash_data_starttask->board_sate == UNDERWAY_GAME_MODE_C)
		{
			flash_data_starttask->board_sate = DEBUG_MODE_C;
		}
	}
	else
	{
		if(flash_data_starttask->board_sate == BEGIN_GAME_MODE_C)
		{
			flash_data_starttask->board_sate = UNDERWAY_GAME_MODE_C;
		}
		else if(flash_data_starttask->board_sate == DEBUG_MODE_C)
		{
			flash_data_starttask->board_sate = DEBUG_MODE_C;
		}
		else if(flash_data_starttask->board_sate == UNDERWAY_GAME_MODE_C)
		{
			flash_data_starttask->board_sate = UNDERWAY_GAME_MODE_C;
		}
	}
	
	
	for(int i = 0; i < 8; i++ )
	{
		flow_led_off(i);
	}
	if(flash_data_starttask->board_sate == DEBUG_MODE_C)
	{
		flow_led_on(1);
	}
	else if(flash_data_starttask->board_sate == BEGIN_GAME_MODE_C)
	{
		flow_led_on(2);
	}
	else if(flash_data_starttask->board_sate == UNDERWAY_GAME_MODE_C)
	{
		flow_led_on(3);
	}
	
	
	
	taskENTER_CRITICAL();
	
	xTaskCreate((TaskFunction_t)LEDTask,
                (const char *)"LEDTask",
                (uint16_t)LED_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)LED_TASK_PRIO,
                (TaskHandle_t *)&LEDTask_Handler);
	
	xTaskCreate((TaskFunction_t)INSTask,
                (const char *)"INSTask",
                (uint16_t)INS_TASK_SIZE,
                (void *)NULL,
                (UBaseType_t)INS_TASK_PRIO,
                (TaskHandle_t *)&INSTask_Handler);
	
  xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
								
	xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);
								
	xTaskCreate((TaskFunction_t)Shoot_task,
                (const char *)"Shoot_task",
                (uint16_t)SHOOT_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)SHOOT_TASK_PRIO,
                (TaskHandle_t *)&SHOOTTask_Handler);
	
	
	
	vTaskDelete(StartTask_Handler);
  taskEXIT_CRITICAL();
	
}


void StartTask(void)
{
    xTaskCreate((TaskFunction_t)start_task,
                (const char *)"start_task",
                (uint16_t)START_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIO,
                (TaskHandle_t *)&StartTask_Handler);
}

