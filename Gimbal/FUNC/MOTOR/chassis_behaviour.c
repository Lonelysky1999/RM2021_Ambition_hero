#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "Gimbal_Task.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "led_task.h"


static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_whip_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

//设定底盘初始状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
	static char c_s_0 = 1;
	static char c_s_0_q = 1;
	c_s_0_q = c_s_0;
	c_s_0 = chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL];
	
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //根据遥控器右上角拨码设定状态机
    if (switch_is_mid(c_s_0))
    {
			chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW ;//中档底盘跟随云台yaw，正常行驶模式
    }
    else if (switch_is_mid(c_s_0_q) && switch_is_down(c_s_0))
    {
			chassis_behaviour_mode = CHASSIS_FOLLOW_CHASSIS_YAW ;//下档底盘不跟随云台yaw，进入抛射模式
    }
    else if (switch_is_up(c_s_0))
    {
			chassis_behaviour_mode = CHASSIS_ZERO_FORCE;//上档底盘无力
    }
		
		if((return_run_away_sign() == 1) && (chassis_behaviour_mode == CHASSIS_FOLLOW_CHASSIS_YAW))
		{
			chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW ;
		}

		if(return_init_mode() || (return_mpu_offset_is_ok() == 0))
		{
			chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
		}

    //底盘无力
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
			chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //底盘速度模式为原生模式
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_CHASSIS_YAW)
    {
      chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //底盘速度跟随底盘yaw，
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //底盘跟随云台yaw
    }
}

//根据不同的状态机设定不同的底盘运动设定值
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)//跟随云台yaw
    {
        chassis_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_FOLLOW_CHASSIS_YAW)//抛射
    {
        chassis_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
		else if(0)//小陀螺
		{
			chassis_whip_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
		}
}

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}


static void chassis_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

static void chassis_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
	if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
	{
			return;
	}
	chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
	*angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set);//角度设定值不变
}

static void chassis_whip_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - 0.00314f);//角度增加
}
