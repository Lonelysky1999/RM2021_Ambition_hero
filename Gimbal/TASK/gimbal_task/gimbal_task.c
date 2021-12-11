/*
ins[2]
3400 --- -0.68

1650 --- 0.00

261  --- 0.52

*/
#include "Gimbal_Task.h"
#include "main.h"
#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "led_task.h"

#include "Ambition_pid.h"


//

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control;

//发送的can 指令
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0 , trigger_can_set_current = 0;

//云台初始化
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);

//云台状态设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//云台状态切换保存数据，例如从陀螺仪状态切换到编码器状态保存目标值
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
static fp32 yawmotor_ecd_to_angle_change(uint16_t ecd, uint16_t last_ecd, uint16_t offset_ecd);
//设置云台控制量
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制pid计算
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);

static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

//在陀螺仪角度控制下，对控制的目标值进限制以防超最大相对角度
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void pitch_limit(fp32 *num, fp32 max, fp32 min);
static void visual_absolute_yaw_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor);
static void visual_relative_yaw_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor);
static void visual_relative_pitch_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor);
static void visual_absolute_pitch_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor);


static void get_cmd_id_2(void);

static void start_sw_init(void);

static int last_ecd_1 = 0;
uint8_t yaw_ecd_direct = 0;//Yaw轴编码器方向

//NEW PID
_Amb_Pid_t AMB_pitch_s_pid;
_Amb_Pid_t AMB_pitch_encode_pid;
_Amb_Pid_t AMB_pitch_tuoluo_pid;
fp32 p_speed_i_block[2] = {0.1f, 1.0f};
fp32 p_angle_i_block[2] = {0.05f, 0.15f};

_Amb_Pid_t AMB_yaw_s_pid;
_Amb_Pid_t AMB_yaw_encode_pid;
_Amb_Pid_t AMB_yaw_tuoluo_pid;
fp32 y_speed_i_block[2] = {0.1f, 1.0f};
fp32 y_angle_i_block[2] = {0.02f, 0.10f};


static uint8_t start_switch = 0;

static uint16_t user_cmd_2 = 0x0000;
//end

void GIMBAL_task(void *pvParameters)
{
    //等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
		visual_recognition_init();
    GIMBAL_Init(&gimbal_control);
		start_sw_init();
    while(1)
    {
        GIMBAL_Set_Mode(&gimbal_control);    //设置云台控制模式
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //控制模式切换 控制数据过渡
        GIMBAL_Feedback_Update(&gimbal_control);             //云台数据反馈
        GIMBAL_Set_Contorl(&gimbal_control);                 //设置云台控制量
        GIMBAL_Control_loop(&gimbal_control);                //云台控制PID计算
				trigger_can_set_current = get_big_trigger_motor_current();
#if YAW_TURN
        Yaw_Can_Set_Current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        Pitch_Can_Set_Current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

				get_cmd_id_2();
			CAN_CMD_GIMBAL(Yaw_Can_Set_Current, Pitch_Can_Set_Current, trigger_can_set_current, user_cmd_2);	
//			CAN_CMD_GIMBAL(0, Pitch_Can_Set_Current, trigger_can_set_current, user_cmd_2);
//			CAN_CMD_GIMBAL(0, 0, 0, 0);

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
//获得yaw轴电机数据指针
const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}
//获得pitch轴电机数据指针
const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

//初始化pid 数据指针
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    //电机数据指针获取
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
		last_ecd_1 = gimbal_init->gimbal_yaw_motor.gimbal_motor_measure->ecd;
		
    //陀螺仪数据指针获取
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    gimbal_init->visual_data =return_visual_data_point();
    //初始化电机模式
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //初始化yaw电机pid                                                                     初始化yaw电机pid                                            初始化yaw电机pid                             初始化yaw电机pid
		Abm_PID_init(&AMB_yaw_s_pid,      YAW_SPEED_PID_KP,           YAW_SPEED_PID_KI,           YAW_SPEED_PID_KD,           YAW_SPEED_PID_MAX_OUT,           YAW_SPEED_PID_MAX_IOUT,           y_speed_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
		Abm_PID_init(&AMB_yaw_encode_pid, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, y_angle_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
		Abm_PID_init(&AMB_yaw_tuoluo_pid, YAW_GYRO_ABSOLUTE_PID_KP,   YAW_GYRO_ABSOLUTE_PID_KI,   YAW_GYRO_ABSOLUTE_PID_KD,   YAW_GYRO_ABSOLUTE_PID_MAX_OUT,   YAW_GYRO_ABSOLUTE_PID_MAX_IOUT,   y_angle_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
    
	
	//初始化pitch电机pid                                                          初始化pitch电机pid                                               初始化pitch电机pid                                             初始化pitch电机pid
		Abm_PID_init(&AMB_pitch_s_pid,      PITCH_SPEED_PID_KP,           PITCH_SPEED_PID_KI,                     PITCH_SPEED_PID_KD,           PITCH_SPEED_PID_MAX_OUT,           PITCH_SPEED_PID_MAX_IOUT,           p_speed_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
		Abm_PID_init(&AMB_pitch_encode_pid, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI,           PITCH_ENCODE_RELATIVE_PID_KD, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, p_angle_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
		Abm_PID_init(&AMB_pitch_tuoluo_pid, PITCH_GYRO_ABSOLUTE_PID_KP,   PITCH_GYRO_ABSOLUTE_PID_KI,             PITCH_GYRO_ABSOLUTE_PID_KD,   PITCH_GYRO_ABSOLUTE_PID_MAX_OUT,   PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT,   p_angle_i_block, 0.0f, 0.0f, 0.0f, 0.0f);
		
		//初始化电机零点编码器位置
    gimbal_init->gimbal_yaw_motor.offset_ecd = YAW_MOTOR_OFFSET;
    gimbal_init->gimbal_pitch_motor.offset_ecd = PITCH_MOTOR_OFFSET;

    GIMBAL_Feedback_Update(gimbal_init);
    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;

    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;
}
//云台状态机设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
		
		//自瞄开启按键
		gimbal_feedback_update->Visual_Control.e_is_press_q = gimbal_feedback_update->Visual_Control.e_is_press;
		if((gimbal_feedback_update->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) != 0)
		{
			gimbal_feedback_update->Visual_Control.e_is_press = 1;
		}
		else
		{
			gimbal_feedback_update->Visual_Control.e_is_press = 0;
		}
		
		//边沿检测与计数器累加
		if((gimbal_feedback_update->Visual_Control.e_is_press == 1) && (gimbal_feedback_update->Visual_Control.e_is_press_q == 0))
		{
			gimbal_feedback_update->Visual_Control.e_cnt++; 
		}
		else
		{
			gimbal_feedback_update->Visual_Control.e_cnt = gimbal_feedback_update->Visual_Control.e_cnt;
		}
		
		//就判断模式切换
		if((gimbal_feedback_update->Visual_Control.e_cnt & 0x01) == 0)
		{
			gimbal_feedback_update->Visual_Control.visual_switch_by_user = 0;//关闭自瞄
		}
		else if((gimbal_feedback_update->Visual_Control.e_cnt & 0x01) == 1)
		{
			gimbal_feedback_update->Visual_Control.visual_switch_by_user = 1;//开启自瞄
		}
		
		//自瞄模式切换按钮
		gimbal_feedback_update->Visual_Control.ctrl_is_press_q = gimbal_feedback_update->Visual_Control.ctrl_is_press;
		if((gimbal_feedback_update->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL) != 0)
		{
			gimbal_feedback_update->Visual_Control.ctrl_is_press = 1;
		}
		else
		{
			gimbal_feedback_update->Visual_Control.ctrl_is_press = 0;
		}
		
		if((gimbal_feedback_update->Visual_Control.ctrl_is_press == 1) && (gimbal_feedback_update->Visual_Control.ctrl_is_press_q == 0))
		{
			gimbal_feedback_update->Visual_Control.ctrl_cnt++; 
		}
		
		if((gimbal_feedback_update->Visual_Control.ctrl_cnt & 0x01) == 0)
		{
			gimbal_feedback_update->Visual_Control.visual_mode_by_user = 0;//大枪模式
		}
		else if((gimbal_feedback_update->Visual_Control.ctrl_cnt & 0x01) == 1)
		{
			gimbal_feedback_update->Visual_Control.visual_mode_by_user = 1;//小枪模式
		}
		
		
    //云台pitch轴数据更新
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                           gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);
    
		gimbal_feedback_update->gimbal_pitch_motor.absolute_angle_360 = gimbal_feedback_update->gimbal_pitch_motor.absolute_angle * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_pitch_motor.relative_angle_360 = gimbal_feedback_update->gimbal_pitch_motor.relative_angle * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_pitch_motor.absolute_angle_set_360 = gimbal_feedback_update->gimbal_pitch_motor.absolute_angle_set * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_pitch_motor.relative_angle_set_360 = gimbal_feedback_update->gimbal_pitch_motor.relative_angle_set * RAD_TO_ANG;
		
		
		//云台yaw轴数据更新
    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = -yawmotor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->last_ecd,
                                                                                            gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);
		
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = (arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle*ANG_TO_RAD) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
																												 - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle*ANG_TO_RAD) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET)));
		
		gimbal_feedback_update->gimbal_yaw_motor.absolute_angle_360 = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_yaw_motor.relative_angle_360 = gimbal_feedback_update->gimbal_yaw_motor.relative_angle * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_yaw_motor.absolute_angle_set_360 = gimbal_feedback_update->gimbal_yaw_motor.absolute_angle_set * RAD_TO_ANG;
		gimbal_feedback_update->gimbal_yaw_motor.relative_angle_set_360 = gimbal_feedback_update->gimbal_yaw_motor.relative_angle_set * RAD_TO_ANG;
		
}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad_2;//新英雄pitch轴齿轮比1：2
}
//yaw轴齿轮比例1：2编码器数值重新计算

static int32_t rel_ecd = 0;
static int32_t dilta_ecd = 0;

static fp32 yawmotor_ecd_to_angle_change(uint16_t ecd, uint16_t last_ecd, uint16_t offset_ecd)
{
	//角度映射
	
  rel_ecd = ecd - offset_ecd;
	dilta_ecd = last_ecd_1 - ecd;
	last_ecd_1 = ecd;
	start_switch = GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_9);
	if(dilta_ecd >  Half_ecd_range)
	{
		yaw_ecd_direct++;
	}
	if(dilta_ecd < -Half_ecd_range)
	{
		yaw_ecd_direct--;
	}
	
	if((return_init_mode())&&((gimbal_control.gimbal_yaw_motor.relative_angle>-0.5f)&&(gimbal_control.gimbal_yaw_motor.relative_angle<0.05f))) 
	{
		if(start_switch == 1)
		{
			yaw_ecd_direct = 0;
		}
		else
		{
			yaw_ecd_direct = 1;
		}
	}
	

	
	if((yaw_ecd_direct&0x01) == 1)
	{
		if(rel_ecd > 0)
		{
			rel_ecd -= ecd_range;
		}
		else
		{
			rel_ecd += ecd_range;
		}
	}

    return rel_ecd * Motor_Ecd_to_Rad_2;
}

//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
    //yaw电机模式控制
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
        
			if((gimbal_set_control->Visual_Control.visual_switch_by_user == 1) && (gimbal_set_control->visual_data->command == 1))
			{
				visual_absolute_yaw_set(gimbal_set_control->visual_data,&gimbal_set_control->gimbal_yaw_motor);
			}
			else
			{
				GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
			}
		}
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
			if((gimbal_set_control->Visual_Control.visual_switch_by_user == 1) && (gimbal_set_control->visual_data->command == 1))
			{
				visual_relative_yaw_set(gimbal_set_control->visual_data,&gimbal_set_control->gimbal_yaw_motor);
			}
			else
			{
				GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
			}
    }
		
		if(return_init_mode())
		{
			gimbal_set_control->gimbal_yaw_motor.relative_angle_set = 0;
		}

    //pitch电机模式控制
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro模式下，陀螺仪角度控制
			if((gimbal_set_control->Visual_Control.visual_switch_by_user == 1) && (gimbal_set_control->visual_data->command == 1))
			{
				visual_absolute_pitch_set(gimbal_set_control->visual_data,&gimbal_set_control->gimbal_pitch_motor);
			}
			else
			{
				GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
			}
				pitch_limit(&gimbal_set_control->gimbal_pitch_motor.absolute_angle_set,0.48f,-0.7053f);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
			if((gimbal_set_control->Visual_Control.visual_switch_by_user == 1) && (gimbal_set_control->visual_data->command == 1))
			{
				visual_relative_pitch_set(gimbal_set_control->visual_data,&gimbal_set_control->gimbal_pitch_motor);
			}
			else
			{
				GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
			}
				pitch_limit(&gimbal_set_control->gimbal_pitch_motor.relative_angle_set,0.48f,-0.7053f);
    }
}
//pitch轴角度限制
static void pitch_limit(fp32 *num, fp32 max, fp32 min)
{
	  if (*num > max)
    {
        *num = max;
    }
    else if (*num < min)
    {
        *num = min;
    }
}
//陀螺仪 控制量限制+设定
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
			
    }
    gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}
//编码器 控制量设定
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
		gimbal_motor->relative_angle_set = rad_format(gimbal_motor->relative_angle_set + add);
}
//云台控制状态使用不同控制pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //yaw不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro角度控制
        //角度环，速度环串级pid调试										                              	PID参数结构体									                               设定值			                                   实际值——相对角度					
			gimbal_control_loop->gimbal_yaw_motor.motor_gyro_set = Amb_PID_cail(&AMB_yaw_tuoluo_pid, gimbal_control_loop->gimbal_yaw_motor.absolute_angle_set, gimbal_control_loop->gimbal_yaw_motor.absolute_angle, LOOP_ERR);
			//速度环																		                     			PID参数结构体						                          		设定值                                         实际值——角速度
			gimbal_control_loop->gimbal_yaw_motor.current_set    = Amb_PID_cail(&AMB_yaw_s_pid, gimbal_control_loop->gimbal_yaw_motor.motor_gyro_set, gimbal_control_loop->gimbal_yaw_motor.motor_gyro, LINE_ERR);
			//控制值赋值
			gimbal_control_loop->gimbal_yaw_motor.given_current  = (int16_t)(gimbal_control_loop->gimbal_yaw_motor.current_set);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde角度控制
      gimbal_control_loop->gimbal_yaw_motor.motor_gyro_set = Amb_PID_cail(&AMB_yaw_encode_pid, gimbal_control_loop->gimbal_yaw_motor.relative_angle_set, gimbal_control_loop->gimbal_yaw_motor.relative_angle, LOOP_ERR);
			//速度环																		                     			PID参数结构体						                          		设定值                                         实际值——角速度
			gimbal_control_loop->gimbal_yaw_motor.current_set = Amb_PID_cail(&AMB_yaw_s_pid, gimbal_control_loop->gimbal_yaw_motor.motor_gyro_set, gimbal_control_loop->gimbal_yaw_motor.motor_gyro, LINE_ERR);
			//控制值赋值
			gimbal_control_loop->gimbal_yaw_motor.given_current = (int16_t)(gimbal_control_loop->gimbal_yaw_motor.current_set);
    }

		
		
		
		
    //pitch不同模式对于不同的控制函数
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw控制
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
			//角度环，速度环串级pid调试										                              	PID参数结构体									                               设定值			                                   实际值——相对角度					
			gimbal_control_loop->gimbal_pitch_motor.motor_gyro_set = Amb_PID_cail(&AMB_pitch_tuoluo_pid, gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set, gimbal_control_loop->gimbal_pitch_motor.absolute_angle, LOOP_ERR);
			//速度环																		                     			PID参数结构体						                          		设定值                                         实际值——角速度
			gimbal_control_loop->gimbal_pitch_motor.current_set = Amb_PID_cail(&AMB_pitch_s_pid, gimbal_control_loop->gimbal_pitch_motor.motor_gyro_set, gimbal_control_loop->gimbal_pitch_motor.motor_gyro, LINE_ERR);
			//控制值赋值
			gimbal_control_loop->gimbal_pitch_motor.given_current = (int16_t)(gimbal_control_loop->gimbal_pitch_motor.current_set);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
			//角度环，速度环串级pid调试										                              	PID参数结构体									                               设定值			                                   实际值——相对角度					
			gimbal_control_loop->gimbal_pitch_motor.motor_gyro_set = Amb_PID_cail(&AMB_pitch_encode_pid, gimbal_control_loop->gimbal_pitch_motor.relative_angle_set, gimbal_control_loop->gimbal_pitch_motor.relative_angle, LOOP_ERR);
			//速度环																		                     			PID参数结构体						                          		设定值                                         实际值——角速度
			gimbal_control_loop->gimbal_pitch_motor.current_set = Amb_PID_cail(&AMB_pitch_s_pid, gimbal_control_loop->gimbal_pitch_motor.motor_gyro_set, gimbal_control_loop->gimbal_pitch_motor.motor_gyro, LINE_ERR);
			//控制值赋值
			gimbal_control_loop->gimbal_pitch_motor.given_current = (int16_t)(gimbal_control_loop->gimbal_pitch_motor.current_set);
    }
}
//直接给电流值
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = 0;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}


static void start_sw_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOI, &GPIO_InitStructure);//
}

static void get_cmd_id_2(void)
{
	if(return_whip_open())//bit3小陀螺开关
	{
		user_cmd_2 = user_cmd_2 & 0xfff7;
		user_cmd_2 = user_cmd_2 | 0x0008;
	}
	else
	{
		user_cmd_2 = user_cmd_2 & 0xfff7;
		user_cmd_2 = user_cmd_2 | 0x0000;
	}
	
	if(gimbal_control.Visual_Control.visual_switch_by_user == 1)//bit2,视觉开关
	{
		user_cmd_2 = user_cmd_2 & 0xfffb;
		user_cmd_2 = user_cmd_2 | 0x0004;
	}
	else
	{
		user_cmd_2 = user_cmd_2 & 0xfffb;
		user_cmd_2 = user_cmd_2 | 0x0000;
	}
	
	if(gimbal_control.Visual_Control.visual_mode_by_user == 1)//bit1,自瞄模式
	{
		user_cmd_2 = user_cmd_2 & 0xfffd;
		user_cmd_2 = user_cmd_2 | 0x0002;
	}
	else
	{
		user_cmd_2 = user_cmd_2 & 0xfffd;
		user_cmd_2 = user_cmd_2 | 0x0000;
	}
	
	if(gimbal_control.visual_data->command != 0)//bit0,自瞄存在
	{
		user_cmd_2 = user_cmd_2 & 0xfffe;
		user_cmd_2 = user_cmd_2 | 0x0001;
	}
	else
	{
		user_cmd_2 = user_cmd_2 & 0xfffe;
		user_cmd_2 = user_cmd_2 | 0x0000;
	}
}



//pitch轴绝对
void visual_absolute_pitch_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor)
{
	fp32 add;
	add = in->pitch_angle * ANG_TO_RAD;
	gimbal_motor->absolute_angle_set = rad_format(add);
}
void visual_relative_pitch_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor)
{
	fp32 add;
	add = in->pitch_angle * ANG_TO_RAD;
	gimbal_motor->relative_angle_set = rad_format(add);
}

//yaw轴增量
void visual_relative_yaw_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor)
{
	fp32 add;
	add = in->yaw_angle * ANG_TO_RAD * -0.0075f;
	gimbal_motor->relative_angle_set = rad_format(gimbal_motor->relative_angle_set + add);
}
void visual_absolute_yaw_set(visual_data_t * in,Gimbal_Motor_t *gimbal_motor)
{
	fp32 add;
	add = in->yaw_angle * ANG_TO_RAD * -0.075f;
	gimbal_motor->absolute_angle_set = rad_format(gimbal_motor->absolute_angle_set + add);
}

Visual_Control_t* return_visual_control_data(void)
{
	return &(gimbal_control.Visual_Control);
}

