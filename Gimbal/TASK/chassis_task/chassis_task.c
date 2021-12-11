
#include "chassis_task.h"

#include "rc.h"
#include "Ambition_pid.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "pid.h"

#include "Remote_Control.h"
#include "INS_Task.h"

#include "chassis_behaviour.h"

#include "gimbal_behaviour.h"

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

//底盘运动数据
static chassis_move_t chassis_move;

//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
		

static void init_s_filter(s_filter_t* t);
		
static int16_t data_rate_change_fp32_to_int16(fp32 max_fp32 , int16_t max_int16_t , fp32 input);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

_Amb_Pid_t follow_pid;		
fp32 i_block_f[2] = {0.5,1}; 

uint8_t press_q = 0;
uint8_t last_press_q = 0;
uint8_t open_whip = 0;


		
//主任务
void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    while (1)
    {
			//遥控器设置状态
			chassis_set_mode(&chassis_move);
			//遥控器状态切换数据保存
			chassis_mode_change_control_transit(&chassis_move);
			//底盘数据更新
			chassis_feedback_update(&chassis_move);
			//底盘控制量设置
			chassis_set_contorl(&chassis_move);
			//发送
			chassis_move.vx_can_set = data_rate_change_fp32_to_int16(NORMAL_MAX_CHASSIS_SPEED_X, 32767, chassis_move.vx_set);
			chassis_move.vy_can_set = data_rate_change_fp32_to_int16(NORMAL_MAX_CHASSIS_SPEED_Y, 32767, chassis_move.vy_set);
			chassis_move.wz_can_set = data_rate_change_fp32_to_int16(CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, 32767, chassis_move.wz_set);


				CAN_CMD_CH(chassis_move.vx_can_set, chassis_move.vy_can_set, chassis_move.wz_can_set, chassis_move.chassis_mode);
			
      vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘旋转环pid值
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //获得云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //初始化旋转PID
    Abm_PID_init(&follow_pid,CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,i_block_f,0.0f,0.5f,0.0f,0.0f);
		//用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);
		
		init_s_filter(&chassis_move_init->s_vx_set);
		init_s_filter(&chassis_move_init->s_vy_set);
		

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }
		


			
    //切入跟随底盘角度模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
		
		last_press_q = press_q;
		press_q = chassis_move_update->chassis_RC->key.v & KEY_PRESSED_OFFSET_Q;
		if((last_press_q == 0)&&(press_q == 64))
			open_whip++;
			
    //计算底盘姿态角度,从云台映射过来
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
			if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
			{
        vx_set_channel = NORMAL_MAX_SET_CHASSIS_SPEED_X;				
			}
			else
			{
        vx_set_channel = NORMAL_NAL_CHASSIS_SPEED_X;
			}

    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
			if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
			{
        vx_set_channel = -NORMAL_MAX_SET_CHASSIS_SPEED_X;				
			}
			else
			{
        vx_set_channel = -NORMAL_NAL_CHASSIS_SPEED_X;				
			}

    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
			if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
			{
        vy_set_channel = NORMAL_MAX_SET_CHASSIS_SPEED_Y;				
			}
			else
			{
        vy_set_channel = NORMAL_NAL_CHASSIS_SPEED_Y;				
			}

    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
			if(chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
			{
        vy_set_channel = -NORMAL_MAX_SET_CHASSIS_SPEED_Y;				
			}
			else
			{
				vy_set_channel = -NORMAL_NAL_CHASSIS_SPEED_Y;
			}

    }



	
			//一阶低通滤波代替斜波作为底盘速度输入
			first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
			first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

		
//S型曲线
//		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = cail_s_filter(vx_set_channel, &chassis_move_rc_to_vector->s_vx_set);
//		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = cail_s_filter(vy_set_channel, &chassis_move_rc_to_vector->s_vy_set);
		
		
    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
//				chassis_move_rc_to_vector->s_vx_set.cnt = 0;
//				chassis_move_rc_to_vector->s_vx_set.start_up = 0;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
//				chassis_move_rc_to_vector->s_vy_set.cnt = 0;
//				chassis_move_rc_to_vector->s_vy_set.start_up = 0;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

//设置遥控器输入控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)//跟随云台
    {
			fp32 sin_yaw = 0.0f;
      fp32 cos_yaw = 0.0f;
			fp32 whipp = 0.0f;
						//设置底盘控制的角度
						
			if(open_whip & 0x01)
			{
				whipp = chassis_move_control->chassis_yaw_set + 3.14/800;//小陀螺速度
				chassis_move_control->chassis_yaw_set = rad_format(whipp);
			}
			else
			{
				chassis_move_control->chassis_yaw_set = rad_format(*(chassis_move_control->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
			}
				//角度换PID
				chassis_move_control->wz_set = Amb_PID_cail(&follow_pid,chassis_move_control->chassis_yaw_set,chassis_move_control->chassis_yaw,LOOP_ERR);
        //设置底盘运动的速度
				//映射底盘移动方向
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        //旋转矩阵
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set =  -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);		
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)//不跟随云台
    {
        fp32 delat_angle = 0.0f;
        fp32 sin_yaw = 0.0f;
        fp32 cos_yaw = 0.0f;
				//设置底盘控制的角度
				chassis_move_control->chassis_yaw_set = rad_format(angle_set);
				delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
				//角度环PID
				chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
        //映射底盘移动方向
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        //旋转矩阵
        chassis_move_control->vx_set =  cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set =  -sin_yaw * vx_set + cos_yaw * vy_set;
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)//底盘无力
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}


static int16_t data_rate_change_fp32_to_int16(fp32 max_fp32 , int16_t max_int16_t , fp32 input)
{
	int16_t output;
	output = (int16_t)(max_int16_t * input / max_fp32);
	return output;
}

static void init_s_filter(s_filter_t* t)
{
	t->start_up   = 0.0f;
	t->cnt        = 0.0f;
	t->input      = 0.0f;
	t->last_input = 0.0f;
	t->start      = 0.0f;
	t->end        = 0.0f;
	t->out        = 0.0f;
}


uint8_t return_whip_open(void)
{
	return open_whip & 0x01;
}

