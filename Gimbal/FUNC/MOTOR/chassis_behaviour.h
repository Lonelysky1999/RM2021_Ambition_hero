#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "main.h"
#include "chassis_task.h"

typedef enum
{
  CHASSIS_ZERO_FORCE,                  	//底盘无力
  CHASSIS_WHIP,                        	//底盘小陀螺
  CHASSIS_FOLLOW_CHASSIS_YAW,          	//云台底盘分开闭环
  CHASSIS_FOLLOW_GIMBAL_YAW,           	//云台底盘一起闭环
	CHASSIS_STOP													//底盘
} chassis_behaviour_e;


extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
