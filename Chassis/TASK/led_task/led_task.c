#include "led_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "sys.h"
#include "CAN_Receive.h"
#include "oled.h"
#include "decision.h"
#include "data_core.h"

uint16_t send_power = 50;
fp32 now_power = 0.0f;
decision_all_data_t* dec_data; 

void LEDTask(void *pvParameters)
{
	dec_data = return_decision_data_point();
	while(1)
	{
		send_power = dec_data->_game_robot_state.game_robot_state.chassis_power_limit;

		
		if((send_power >= 50) && (send_power <= 120))
		{
			CAN_CMD_POWER_DATA(send_power);
		}
		else
		{
				CAN_CMD_POWER_DATA(50); 
		}
//		CAN_CMD_POWER_DATA(90); 
		
		led_green_toggle();
		vTaskDelay(50);
	}
}

