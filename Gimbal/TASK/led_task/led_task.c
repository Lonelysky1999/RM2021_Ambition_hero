#include "led_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"
#include "main.h"
#include "Ambition_data_rule.h"
#include "flash.h"
#include "oled.h"
#include "fpv_control.h"
#include "shoot_task.h"
#include "adc.h"
#include "CAN_Receive.h"
#include "remote_control.h"
#include "Gimbal_Task.h"

#include "INS_task.h"

flash_data_list_u* flash_union_data_point;
u32 flash_data_len = 0;

u8 mpu_offset_cail_is_ok = 0;

const fp32 *angle;
const fp32 *zero_offset;

shoot_sys_t* shoot_data;
_judg_data_t* judg_data_oled;
const motor_measure_t* shoot_L; 
const motor_measure_t* shoot_R; 
const RC_ctrl_t * rc_oled;
const Gimbal_Motor_t * pitch_oled;
const Gimbal_Motor_t * yaw_oled;

uint16_t butten_ad = 0;
uint8_t page = 0;

void LEDTask(void *pvParameters)
{
	led_green_off();
	flow_led_off(0);

	//flash
	flash_union_data_point = get_flash_union_data_point();
	flash_data_len = get_flash_data_buffer_len();
	
	//ins
	angle = get_INS_angle_point();
	zero_offset = get_gyro_offset_point();
	
	//shoot
	shoot_data = return_shoot_data_p();
	judg_data_oled = get_judg_data_point();
	shoot_L = get_Shoot_Motor_L_Measure_Point();
	shoot_R = get_Shoot_Motor_R_Measure_Point();
	
	//
	rc_oled = get_remote_control_point();
	
	//g
	pitch_oled = get_pitch_motor_point();
	yaw_oled   = get_yaw_motor_point();
	
	while(1)
	{
		flow_led_toggle(0);
//		if(USER_KEY && mpu_offset_cail_is_ok)
//		{
//			amb_write_flash();
//		}
		butten_ad = get_ADC(ADC_Channel_6);
		
		if(butten_ad > 0 && butten_ad <= 100) //mid
		{
			page = 0;
		}
		else if(butten_ad >= 2000 && butten_ad <= 2400)//up
		{
			page = 2;
		}
		else if(butten_ad >= 2900 && butten_ad <= 3100)//down
		{
			page = 3;
		}
		else if(butten_ad >= 700 && butten_ad <= 1000)//left
		{
			page = 4;
		}
		else if(butten_ad >= 1500 && butten_ad <= 1800)//right
		{
			page = 5;
		}
		else
		{
			page = page;
		}
		
		oled_clear(Pen_Clear);
		if(page == 0)
		{
			oled_printf(1,1,"temp :%f",get_MPU6500_temp());
			oled_printf(2,1,"yaw  :%f",angle[0]);
			oled_printf(3,1,"pitch:%f",angle[2]);
			oled_printf(4,1,"pwm  :%d",TIM3->CCR2);
			
		}
		else if(page == 2)//up
		{
			
			oled_printf(1,1,"big_motor_L:%4f",shoot_data->speed_L);
			oled_printf(2,1,"big_motor_R:%4f",shoot_data->speed_R);
			oled_printf(3,1,"tempL:%f" ,shoot_L->temperate);
			oled_printf(4,1,"tempR:%f" ,shoot_R->temperate);

		}
		else if(page == 3)//down
		{
			oled_printf(3,1,"pwm_set      :%d",shoot_data->snail_pwm_set_R);
			oled_printf(4,1,"dubug_pwm_set:%d",return_debug_snail_set());
		}
		else if(page == 4)
		{
			oled_printf(1,1,"x:%d", rc_oled->mouse.x);
			oled_printf(2,1,"y:%d", rc_oled->mouse.y);
			oled_printf(3,1,"z:%d", rc_oled->mouse.z);
		}
		else if(page == 5)
		{
			oled_printf(1,1,"yaw_set  :%4f",yaw_oled->relative_angle_set_360);
			oled_printf(2,1,"yaw      :%4f",yaw_oled->relative_angle_360);
			oled_printf(3,1,"pitch_set:%f" ,pitch_oled->relative_angle_set_360);
			oled_printf(4,1,"pitch    :%f" ,pitch_oled->relative_angle_360);
		}
		else
		{
			oled_printf(2,1,"error");
		}
		

		oled_refresh_gram();
		
		vTaskDelay(50);
	}
}

void set_offset_cail_is_ok(void)
{
	mpu_offset_cail_is_ok = 1;
}

uint8_t return_mpu_offset_is_ok(void)
{
	return mpu_offset_cail_is_ok;
}

void amb_write_flash(void)
{
	led_green_on();
	write_data_to_flash(flash_union_data_point);
	STMFLASH_Write(AMBITION_DATA_DAAR_BEGIN,(flash_union_data_point->flash_data_buffer),flash_data_len);
	led_green_off();
}

