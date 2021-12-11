#include "shoot_task.h"
#include "snail_motor.h"
#include "gimbal_task.h"
#include "user_lib.h"
#include "CAN_Receive.h"
#include "Ambition_pid.h"
#include "remote_control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define BIGMODE shoot_sys_data.bigshoot_mode
#define SMAMODE shoot_sys_data.smallshoot_mode

#define BIG_SHOOT_SPEED 6250		
#define BIG_SHOOT_LOW_SPEED 5100		
//6250

//4650
#define BIG_TRIGGER_SPEED -1.0



static void sys_shoot_init(void);	//射击系统初始化
static void shoot_set_mode(void);	//射击状态机
static void shoot_feedback_update(void);	//相关数据更新
static void shoot_control_set(void);	//控制量设置
static void shoot_pid_loop(void);	//pid计算

static fp32 Trigger_angle_PID_Calc(Trigger_angle_PID_t *pid, fp32 get, fp32 set);
static void Trigger_angle_PID_Init(Trigger_angle_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);

static void big_shoot_motor_speed_set(void);
static void big_trigger_set(void);
static void small_shoot_motor_speed_set(void);
static void small_trigger_set(void);

fp32 _s_abs_fp32(fp32 input);

shoot_sys_t shoot_sys_data;
_judg_data_t* judg_data_shoot;
Visual_Control_t* visual_control_shoot;


static uint8_t g_is_press = 0;
static uint8_t g_is_press_q = 0;
static uint8_t g_cnt = 0;

//摩擦轮缓启动系数
const static fp32 shoot_motor_order_filter[1] = {0.5666666667f};
static fp32 shoot_i_block[2] = {100,500};

static int motor_t_diert_edc = 0.0f;
static uint8_t  snail_init_is_ok = 0;
static uint16_t snail_init_cnt   = 0;

fp32 look_l;
fp32 look_r;



//二阶低通滤波系数
const static fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};


static uint16_t dubug_pwm_set = 1000;

static uint16_t send_cnt = 0;

void Shoot_task(void *pvParameters)
{
	
	sys_shoot_init();
	visual_control_shoot = return_visual_control_data();
	vTaskDelay(2000);
	while (1)
	{
		shoot_set_mode();
		shoot_feedback_update();
		shoot_control_set();
		shoot_pid_loop();
		CAN_CMD_CHASSIS(shoot_sys_data.given_current_L,shoot_sys_data.given_current_R, shoot_sys_data.small_trigger.given_current,0);
		snail_1_set(shoot_sys_data.snail_pwm_set_L);
		snail_2_set(shoot_sys_data.snail_pwm_set_R);
		
		
		
		if(send_cnt >= 1000)
		{
			send_data_to_pc(visual_control_shoot->visual_mode_by_user);
			send_cnt = 0;
		}
		else
		{
			send_cnt = send_cnt + 1;
		}
		
		
		
		vTaskDelay(1);
	} 
}

//发射系统初始化
static void sys_shoot_init(void)
{
	//大拨弹轮速度环PID
    static const fp32 big_trigger_speed_pid[3] = {BIG_TRIGGER_SPEED_PID_KP,BIG_TRIGGER_SPEED_PID_KI,BIG_TRIGGER_SPEED_PID_KD};
	//小拨弹轮速度环PID
    static const fp32 small_trigger_speed_pid[3] = {SMALL_TRIGGER_SPEED_PID_KP,SMALL_TRIGGER_SPEED_PID_KI,SMALL_TRIGGER_SPEED_PID_KD};

		//初始化大拨弹轮速度环pid
    PID_Init(&(shoot_sys_data.big_trigger.trigger_motor_speed_pid), PID_POSITION, big_trigger_speed_pid, BIG_TRIGGER_SPEED_PID_MAX_OUT,BIG_TRIGGER_SPEED_PID_MAX_IOUT);
		//初始化大拨弹论角度环pid
		Trigger_angle_PID_Init(&(shoot_sys_data.big_trigger.trigger_motor_angle_pid),BIG_TRIGGER_ANGLE_PID_MAX_OUT,BIG_TRIGGER_ANGLE_PID_MAX_IOUT,BIG_TRIGGER_ANGLE_PID_KP,BIG_TRIGGER_ANGLE_PID_KI,BIG_TRIGGER_ANGLE_PID_KD);
		//初始化小拨弹轮速度环pid
		PID_Init(&(shoot_sys_data.small_trigger.trigger_motor_speed_pid), PID_POSITION, small_trigger_speed_pid, SMALL_TRIGGER_SPEED_PID_MAX_OUT, SMALL_TRIGGER_SPEED_PID_MAX_IOUT);
		//初始化小拨弹轮角度环pid
		Trigger_angle_PID_Init(&(shoot_sys_data.small_trigger.trigger_motor_angle_pid),SMALL_TRIGGER_ANGLE_PID_MAX_OUT,SMALL_TRIGGER_ANGLE_PID_MAX_IOUT,SMALL_TRIGGER_ANGLE_PID_KP,SMALL_TRIGGER_ANGLE_PID_KI,SMALL_TRIGGER_ANGLE_PID_KD);
		
		Abm_PID_init(&(shoot_sys_data.shoot_motor_L_Apid), BIG_SHOOT_L_SPEED_PID_KP, BIG_SHOOT_L_SPEED_PID_KI, BIG_SHOOT_L_SPEED_PID_KD, BIG_SHOOT_L_SPEED_PID_MAX_OUT, BIG_SHOOT_L_SPEED_PID_MAX_IOUT, shoot_i_block,0.0f,0.0f,0.0f,0.0f);
		Abm_PID_init(&(shoot_sys_data.shoot_motor_R_Apid), BIG_SHOOT_R_SPEED_PID_KP, BIG_SHOOT_R_SPEED_PID_KI, BIG_SHOOT_R_SPEED_PID_KD, BIG_SHOOT_R_SPEED_PID_MAX_OUT, BIG_SHOOT_R_SPEED_PID_MAX_IOUT, shoot_i_block,0.0f,0.0f,0.0f,0.0f);
		
		judg_data_shoot = get_judg_data_point();
		
    //遥控器指针
    shoot_sys_data.shoot_RC = get_remote_control_point();
    //左右摩擦轮指针
    shoot_sys_data.shoot_motor_L = get_Shoot_Motor_L_Measure_Point();
    shoot_sys_data.shoot_motor_R = get_Shoot_Motor_R_Measure_Point();
    //大拨弹轮指针
    shoot_sys_data.big_trigger.trigger_motor = get_Trigger_Motor_Measure_Point();
		//小拨弹轮指针
    shoot_sys_data.small_trigger.trigger_motor = get_Small_Trigger_Measure_Point();


    //初始化大发射机构状态机
    shoot_sys_data.bigshoot_mode = Shoot_stop;
		shoot_sys_data.smallshoot_mode = shoot_stop;

    //大摩擦轮缓启动
    first_order_filter_init(&(shoot_sys_data.shoot_motor_L_filter), 0.001f, shoot_motor_order_filter);
    first_order_filter_init(&(shoot_sys_data.shoot_motor_R_filter), 0.001f, shoot_motor_order_filter);
		
		//小摩擦轮缓启动
    first_order_filter_init(&(shoot_sys_data.snail_L_filter), 0.001f, shoot_motor_order_filter);
    first_order_filter_init(&(shoot_sys_data.snail_R_filter), 0.001f, shoot_motor_order_filter);
		
    
	//拨弹轮转速的二阶低通滤波器初始化
    shoot_sys_data.big_trigger.speed_fliter_1 = 0.0f;
    shoot_sys_data.big_trigger.speed_fliter_2 = 0.0f;
    shoot_sys_data.big_trigger.speed_fliter_3 = 0.0f;
		shoot_sys_data.small_trigger.speed_fliter_1 = 0.0f;
    shoot_sys_data.small_trigger.speed_fliter_2 = 0.0f;
    shoot_sys_data.small_trigger.speed_fliter_3 = 0.0f;
	 

    //大摩擦轮数值初始化
    shoot_sys_data.speed_L          = 0.0f;
		shoot_sys_data.speed_raw_set_L  = 0.0f;
    shoot_sys_data.speed_set_L      = 0.0f;
    shoot_sys_data.given_current_L  = 0.0f;
    shoot_sys_data.speed_R          = 0.0f;
		shoot_sys_data.speed_raw_set_R  = 0.0f;
    shoot_sys_data.speed_set_R 	    = 0.0f;
    shoot_sys_data.given_current_R  = 0.0f;


    //鼠标控制量初始化
    shoot_sys_data.press_l = 0;
    shoot_sys_data.press_r = 0;
    shoot_sys_data.last_press_l = 0;
    shoot_sys_data.last_press_r = 0;
    shoot_sys_data.press_l_time = 0;
    shoot_sys_data.press_r_time = 0;

    shoot_sys_data.snail_pwm_set_L = 1000;
    shoot_sys_data.snail_pwm_set_R = 1000;


    //数据更新
    shoot_feedback_update();
		shoot_sys_data.big_trigger.last_edc   = shoot_sys_data.big_trigger.edc;
		shoot_sys_data.small_trigger.last_edc = shoot_sys_data.small_trigger.edc;

	//拨弹轮初始设定值
	shoot_sys_data.big_trigger.angle_set = shoot_sys_data.big_trigger.angle;
	shoot_sys_data.big_trigger.angle_set_lock = 1;//锁定状态
	shoot_sys_data.big_trigger.move_set_lock = 1;

	shoot_sys_data.small_trigger.angle_set = shoot_sys_data.small_trigger.angle;
	shoot_sys_data.small_trigger.angle_set_lock = 1;//锁定状态

}

//发射系统状态机转换
static void shoot_set_mode(void)
{

		if(switch_is_up(shoot_sys_data.shoot_RC->rc.s[1]))
		{
			BIGMODE = Shoot_stop;
			SMAMODE = shoot_stop;
		}
		else if(switch_is_mid(shoot_sys_data.shoot_RC->rc.s[1]))
		{
			if(judg_data_shoot->remain_42mm_num == 0)
			{
				BIGMODE = Shoot_stop;
			}
			else
			{
				BIGMODE = Shoot_ready;
			}
			
			SMAMODE = shoot_ready;
		}
		else if(switch_is_down(shoot_sys_data.shoot_RC->rc.s[1]))
		{
			BIGMODE = Shoot_clean;
			SMAMODE = shoot_ready;
		}

		

		
		if(judg_data_shoot->small_shoot_power == 0)
		{
			snail_init_is_ok = 0;
			snail_init_cnt = 0;
		}
		else if(judg_data_shoot->small_shoot_power == 1)
		{
			if(snail_init_cnt > 4000)
			{
				snail_init_is_ok = 1;
			}
			else
			{
				snail_init_cnt++;
				snail_init_is_ok = 0;
			}
		}
		
}



//实时数据更新
static void shoot_feedback_update(void)
{
	//大拨弹轮转速二阶低通滤波
    shoot_sys_data.big_trigger.speed_fliter_1 = shoot_sys_data.big_trigger.speed_fliter_2;
    shoot_sys_data.big_trigger.speed_fliter_2 = shoot_sys_data.big_trigger.speed_fliter_3;
    shoot_sys_data.big_trigger.speed_fliter_3 = shoot_sys_data.big_trigger.speed_fliter_2 * fliter_num[0] + shoot_sys_data.big_trigger.speed_fliter_1 * fliter_num[1] + (shoot_sys_data.big_trigger.trigger_motor->speed_rpm * Motor_RMP_TO_SPEED_3508) * fliter_num[2];
    shoot_sys_data.big_trigger.speed = shoot_sys_data.big_trigger.speed_fliter_3;
	//小拨弹轮转速二阶低通滤波
    shoot_sys_data.small_trigger.speed_fliter_1 = shoot_sys_data.small_trigger.speed_fliter_2;
    shoot_sys_data.small_trigger.speed_fliter_2 = shoot_sys_data.small_trigger.speed_fliter_3;
    shoot_sys_data.small_trigger.speed_fliter_3 = shoot_sys_data.small_trigger.speed_fliter_2 * fliter_num[0] + shoot_sys_data.small_trigger.speed_fliter_1 * fliter_num[1] + (shoot_sys_data.small_trigger.trigger_motor->speed_rpm * Motor_RMP_TO_SPEED_2006) * fliter_num[2];
    shoot_sys_data.small_trigger.speed = shoot_sys_data.small_trigger.speed_fliter_3;
    
	//获取大摩擦轮转速
    shoot_sys_data.speed_L = shoot_sys_data.shoot_motor_L->speed_rpm;
    shoot_sys_data.speed_R = shoot_sys_data.shoot_motor_R->speed_rpm;
look_l = shoot_sys_data.speed_L;
look_r = -shoot_sys_data.speed_R;

	shoot_sys_data.big_trigger.last_edc = shoot_sys_data.big_trigger.edc;
	shoot_sys_data.big_trigger.edc      = shoot_sys_data.big_trigger.trigger_motor->ecd;
	motor_t_diert_edc = shoot_sys_data.big_trigger.edc - shoot_sys_data.big_trigger.last_edc;
	//大拨弹轮角度计算
	if(motor_t_diert_edc > Half_ecd_range)
	{
		shoot_sys_data.big_trigger.ecd_count--;
	}
	else if (motor_t_diert_edc < -Half_ecd_range)
	{
		shoot_sys_data.big_trigger.ecd_count++;
	}
	
	
	if(shoot_sys_data.big_trigger.ecd_count == 19)
	{
		shoot_sys_data.big_trigger.ecd_count = 0;
	}
	if(shoot_sys_data.big_trigger.ecd_count < 0 )
	{
		shoot_sys_data.big_trigger.ecd_count += 19; 
	}
	fp32 lang_0_2pi = 0.0f;
	lang_0_2pi = (shoot_sys_data.big_trigger.ecd_count * ecd_range + shoot_sys_data.big_trigger.trigger_motor->ecd) * Motor_ECD_TO_ANGLE_3508;
	if(lang_0_2pi > PI)
	{
		lang_0_2pi -= (2 * PI); 
	}
	if(lang_0_2pi <= -PI)
	{
		lang_0_2pi += (2 * PI);
	}
	shoot_sys_data.big_trigger.angle = lang_0_2pi;



		
		//小拨弹轮角度计算
		if (shoot_sys_data.small_trigger.trigger_motor->ecd - shoot_sys_data.small_trigger.trigger_motor->last_ecd > Half_ecd_range)
    {
        shoot_sys_data.small_trigger.ecd_count--;
    }
    else if (shoot_sys_data.small_trigger.trigger_motor->ecd - shoot_sys_data.small_trigger.trigger_motor->last_ecd < -Half_ecd_range)
    {
        shoot_sys_data.small_trigger.ecd_count++;
}
		if(shoot_sys_data.small_trigger.ecd_count == 36)
		{
			shoot_sys_data.small_trigger.ecd_count = 0;
		}
		if(shoot_sys_data.small_trigger.ecd_count < 0 )
		{
			shoot_sys_data.small_trigger.ecd_count += 36; 
		}
		fp32 ang_0_2pi = 0.0f;
		ang_0_2pi = (shoot_sys_data.small_trigger.ecd_count * ecd_range + shoot_sys_data.small_trigger.trigger_motor->ecd) * Motor_ECD_TO_ANGLE_2006;
		if(ang_0_2pi > PI)
		{
			ang_0_2pi -= (2 * PI); 
		}
		if(ang_0_2pi <= -PI)
		{
			ang_0_2pi += (2 * PI);
		}
		shoot_sys_data.small_trigger.angle = ang_0_2pi;
		
    //鼠标按键
    shoot_sys_data.last_press_l = shoot_sys_data.press_l;
    shoot_sys_data.last_press_r = shoot_sys_data.press_r;
    shoot_sys_data.press_l = shoot_sys_data.shoot_RC->mouse.press_l;
    shoot_sys_data.press_r = shoot_sys_data.shoot_RC->mouse.press_r;
	
		if((BIGMODE == Shoot_ready) || (BIGMODE == Shoot_clean))
		{
			if(shoot_sys_data.last_press_l == 0 && shoot_sys_data.press_l != 0)
			{
				shoot_sys_data.big_trigger.angle_set_lock = 0;
				shoot_sys_data.big_trigger.move_set_lock = 0;
			}
			
			if(shoot_sys_data.press_l == 0)
			{
				shoot_sys_data.big_trigger.angle_set_lock = 1;
			}
		}
		else
		{
			shoot_sys_data.big_trigger.angle_set_lock = 1;
		}


		
		
    //左键长按检测
    if(shoot_sys_data.press_l)
    {
        if(shoot_sys_data.press_l_time < PRESS_LONG_TIME)
        {
            shoot_sys_data.press_l_time++;
        }
    }
    else
    {
        shoot_sys_data.press_l_time = 0;
    }
		
    //右键长按检测
    if(shoot_sys_data.press_r)
    {
        if(shoot_sys_data.press_r_time < PRESS_LONG_TIME)
        {
            shoot_sys_data.press_r_time++;
        }
    }
    else
    {
        shoot_sys_data.press_r_time = 0;
    }
	

		
//弹舱开启按键
		g_is_press_q = g_is_press;
		if((shoot_sys_data.shoot_RC->key.v & KEY_PRESSED_OFFSET_G) != 0)
		{
			g_is_press = 1;
		}
		else
		{
			g_is_press = 0;
		}
		
		//边沿检测与计数器累加
		if((g_is_press == 1) && (g_is_press_q == 0))
		{
			g_cnt++; 
		}
		else
		{
			g_cnt = g_cnt;
		}
		
		//就判断模式切换
		if((g_cnt & 0x01) == 0)
		{
			//open
			TIM_SetCompare4(TIM8,1500);
		}
		else if((g_cnt & 0x01) == 1)
		{
			//close
			TIM_SetCompare4(TIM8,500);
		}
		
}

//发射相关控制量设定
static void shoot_control_set(void)
{
	big_shoot_motor_speed_set();
	big_trigger_set();
	small_shoot_motor_speed_set();
	small_trigger_set();
}

static void shoot_pid_loop(void)
{
	fp32 c_r = 0.0f;
	fp32 c_l = 0.0f;
	fp32 c_bt = 0.0f;
	fp32 c_st = 0.0f;
	c_l = Amb_PID_cail(&(shoot_sys_data.shoot_motor_L_Apid),shoot_sys_data.speed_set_L,shoot_sys_data.speed_L,LINE_ERR);
	c_r = Amb_PID_cail(&(shoot_sys_data.shoot_motor_R_Apid),shoot_sys_data.speed_set_R,shoot_sys_data.speed_R,LINE_ERR);
	
	shoot_sys_data.given_current_L = (int16_t)(c_l);
	shoot_sys_data.given_current_R = (int16_t)(c_r);

	shoot_sys_data.big_trigger.speed_set = Trigger_angle_PID_Calc(&(shoot_sys_data.big_trigger.trigger_motor_angle_pid) , shoot_sys_data.big_trigger.angle , shoot_sys_data.big_trigger.angle_set);
	c_bt = PID_Calc(&(shoot_sys_data.big_trigger.trigger_motor_speed_pid) , shoot_sys_data.big_trigger.speed , shoot_sys_data.big_trigger.speed_set);
	
	if(SMAMODE != shoot_stop)
	{
		shoot_sys_data.small_trigger.speed_set = Trigger_angle_PID_Calc(&(shoot_sys_data.small_trigger.trigger_motor_angle_pid) , shoot_sys_data.small_trigger.angle , shoot_sys_data.small_trigger.angle_set);
		c_st = PID_Calc(&(shoot_sys_data.small_trigger.trigger_motor_speed_pid) , shoot_sys_data.small_trigger.speed , shoot_sys_data.small_trigger.speed_set);
	}
	
	shoot_sys_data.big_trigger.given_current = (int16_t)c_bt;
	shoot_sys_data.small_trigger.given_current = (int16_t)c_st;

	if(SMAMODE == shoot_stop)
	{
		shoot_sys_data.small_trigger.given_current = 0;
	}

}



//大摩擦轮速度设置
static void big_shoot_motor_speed_set(void)
{
	switch (BIGMODE)
	{
		case Shoot_stop:
			{
				shoot_sys_data.speed_raw_set_L = 0;
				shoot_sys_data.speed_raw_set_R = 0;
				break;
			}
		case Shoot_ready:
			{
				if(judg_data_shoot->max_42_speed == 16)
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_SPEED;
				}
				else if(judg_data_shoot->max_42_speed == 10)
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_LOW_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_LOW_SPEED;
				}
				else
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_LOW_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_LOW_SPEED;
				}


				break;
			}
		case Shoot_clean:
			{
				if(judg_data_shoot->max_42_speed == 16)
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_SPEED;
				}
				else if(judg_data_shoot->max_42_speed == 10)
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_LOW_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_LOW_SPEED;
				}
				else
				{
					shoot_sys_data.speed_raw_set_L = BIG_SHOOT_LOW_SPEED;
					shoot_sys_data.speed_raw_set_R = BIG_SHOOT_LOW_SPEED;
				}
				break;
			}
		default:
			break;
	}
	first_order_filter_cali(&(shoot_sys_data.shoot_motor_L_filter),shoot_sys_data.speed_raw_set_L);
	first_order_filter_cali(&(shoot_sys_data.shoot_motor_R_filter),shoot_sys_data.speed_raw_set_R);
	shoot_sys_data.speed_set_L = -shoot_sys_data.shoot_motor_L_filter.out;
	shoot_sys_data.speed_set_R = shoot_sys_data.shoot_motor_R_filter.out;
}



static uint8_t dz_sb = 0;

//大拨弹轮控制量设置
static void big_trigger_set(void)
{
	switch (BIGMODE)
	{
		case Shoot_stop://停止状态下，使用双环稳定静止
		{
			if(dz_sb == 0)
			{
				shoot_sys_data.big_trigger.angle_set = shoot_sys_data.big_trigger.angle;
				dz_sb = 1;
			}
			else
			{
				shoot_sys_data.big_trigger.angle_set = shoot_sys_data.big_trigger.angle_set;
			}
			
			break;
		}
		case Shoot_ready://
		{		
			dz_sb = 0;//                                                       裁判系统热量锁
				if((shoot_sys_data.big_trigger.angle_set_lock == 0)&&(judg_data_shoot->shoot_lock_42 == 0)&&(judg_data_shoot->remain_42mm_num != 0))
				{
					shoot_sys_data.big_trigger.angle_set = rad_format(shoot_sys_data.big_trigger.angle_set - SINGLE_BIG_TRIGGER_ADD_ANGLE);
					shoot_sys_data.big_trigger.angle_set_lock = 1;
				}
				else //if(shoot_sys_data.big_trigger.angle_set_lock == 1)
				{
					shoot_sys_data.big_trigger.angle_set = shoot_sys_data.big_trigger.angle_set;
				}
				break;
		}
		case Shoot_clean://赛后清弹过程中大拨弹轮持续旋转，吐出弹丸
		{
			dz_sb = 0;
			if((shoot_sys_data.big_trigger.angle_set_lock == 0)&&(judg_data_shoot->shoot_lock_42 == 0))
			{
				shoot_sys_data.big_trigger.angle_set = rad_format(shoot_sys_data.big_trigger.angle_set - SINGLE_BIG_TRIGGER_ADD_ANGLE);
				shoot_sys_data.big_trigger.angle_set_lock = 1;
			}
			else //if(shoot_sys_data.big_trigger.angle_set_lock == 1)
			{
				shoot_sys_data.big_trigger.angle_set = shoot_sys_data.big_trigger.angle_set;
			}
			break;
		}
		default:
			dz_sb = 0;
			break;
	}
}
//小摩擦轮控制
static void small_shoot_motor_speed_set(void)
{
	
	if((snail_init_is_ok == 0))
	{
		shoot_sys_data.snail_pwm_set_L = 1090;
		shoot_sys_data.snail_pwm_set_R = 1090;
	}
	else if(snail_init_is_ok == 1)
	{
		switch (SMAMODE)
		{
			case shoot_stop:
				{
					shoot_sys_data.snail_pwm_raw_set_L = 1090;
					shoot_sys_data.snail_pwm_raw_set_R = 1090;
					break;
				}
			case shoot_ready:
				{
					shoot_sys_data.snail_pwm_raw_set_L = 1310;
					shoot_sys_data.snail_pwm_raw_set_R = 1310;
					break;
				}
			case shoot_clean:
				{
					shoot_sys_data.snail_pwm_raw_set_L = 1310;
					shoot_sys_data.snail_pwm_raw_set_R = 1310;
					break;
				}
			default:
				break;
		}
		first_order_filter_cali(&(shoot_sys_data.snail_L_filter),shoot_sys_data.snail_pwm_raw_set_L);
		first_order_filter_cali(&(shoot_sys_data.snail_R_filter),shoot_sys_data.snail_pwm_raw_set_R);
		shoot_sys_data.snail_pwm_set_L = shoot_sys_data.snail_L_filter.out;
		shoot_sys_data.snail_pwm_set_R = shoot_sys_data.snail_R_filter.out;
	}
	
	
	
}
//小拨弹轮控制
static void small_trigger_set(void)
{
		switch (SMAMODE)
	{
		case shoot_stop://停止状态
		{
			shoot_sys_data.small_trigger.angle_set = shoot_sys_data.small_trigger.angle;
			break;
		}
		case shoot_ready://
		{
				if(shoot_sys_data.press_r && (judg_data_shoot->shoot_lock_17 == 0))
				{
					shoot_sys_data.small_trigger.angle_set = rad_format(shoot_sys_data.small_trigger.angle_set + PI/600);
				}
				else
				{
					shoot_sys_data.small_trigger.angle_set = shoot_sys_data.small_trigger.angle_set;
				}
				break;
		}
		case shoot_clean://赛后清弹过程中大拨弹轮持续旋转，吐出弹丸
		{
			if(judg_data_shoot->shoot_lock_17 == 0)
			{
				shoot_sys_data.small_trigger.angle_set = rad_format(shoot_sys_data.small_trigger.angle_set + PI/1000);
			}
			else
			{
				shoot_sys_data.small_trigger.angle_set = shoot_sys_data.small_trigger.angle_set;
			}
			break;
		}
		default:
			break;
	}
}


//拨弹轮位置环pid计算
static fp32 Trigger_angle_PID_Calc(Trigger_angle_PID_t *pid, fp32 get, fp32 set)
{
    fp32 err;
		fp32 derr;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;
		err = set - get;
		pid->last_err = pid->err;
    pid->err = rad_format(err);
		derr = pid->err - pid->last_err;
		pid->derr = rad_format(derr);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * pid->derr;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void Trigger_angle_PID_Init(Trigger_angle_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;
		pid->derr = 0.0f;
		pid->last_err = pid->err;
		
    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

int16_t get_big_trigger_motor_current(void)
{
	return shoot_sys_data.big_trigger.given_current;
}

fp32 _s_abs_fp32(fp32 input)
{
	return (input>=0)?input:-input;
}

uint16_t return_snail_set(void)
{
	return shoot_sys_data.snail_pwm_set_L;
}

uint16_t return_debug_snail_set(void)
{
	return dubug_pwm_set;
}

shoot_sys_t* return_shoot_data_p(void)
{
	return &(shoot_sys_data);
}


