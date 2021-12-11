#include "chassis_task.h"
#include "Ambition_pid.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "CAN_Receive.h"
#include "pid.h"


//底盘运动数据
static chassis_move_t chassis_move;

//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);

//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static fp32 data_rate_change_int16_to_fp32(fp32 max_fp32 , int16_t max_int16_t , int16_t input);

_Amb_Pid_t CM_pid_1;
_Amb_Pid_t CM_pid_2;
_Amb_Pid_t CM_pid_3;
_Amb_Pid_t CM_pid_4;
fp32 i_block[2] = {0.2,0.7};

const cap_data_t * cap_data_ch;

//主任务
void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    while (1)
    {
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);
        CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }
		//AMB_PID
		Abm_PID_init(&CM_pid_1, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, i_block, 0.0f, 0.6f, 0.0f, 0.0f);
		Abm_PID_init(&CM_pid_2, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, i_block, 0.0f, 0.6f, 0.0f, 0.0f);
		Abm_PID_init(&CM_pid_3, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, i_block, 0.0f, 0.6f, 0.0f, 0.0f);
		Abm_PID_init(&CM_pid_4, M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT, i_block, 0.0f, 0.6f, 0.0f, 0.0f);

    uint8_t i;
    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取底盘设定值
    chassis_move_init->chrx_data = get_CHRX_data_Point();
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i);
    }
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

		cap_data_ch = get_cap_data_Point();
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    }
		chassis_move_update->chassis_mode = (chassis_mode_e)(chassis_move_update->chrx_data->ch_mode);
    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    chassis_move_update->vx_set = data_rate_change_int16_to_fp32(NORMAL_MAX_CHASSIS_SPEED_X				, 32767, chassis_move_update->chrx_data->vx_set);
    chassis_move_update->vy_set = data_rate_change_int16_to_fp32(NORMAL_MAX_CHASSIS_SPEED_Y				, 32767, chassis_move_update->chrx_data->vy_set);
    chassis_move_update->wz_set = data_rate_change_int16_to_fp32(CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, 32767, chassis_move_update->chrx_data->wz_set);

		
//		if(cap_data_ch->cap_u < 18.0f)
//		{
//			abs_limit(&(chassis_move_update->vx_set),NORMAL_NAL_CHASSIS_SPEED_X);
//			abs_limit(&(chassis_move_update->vy_set),NORMAL_NAL_CHASSIS_SPEED_Y);
//		}
		
}

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
			//赋值电流值
			for (i = 0; i < 4; i++)
			{
				chassis_move_control_loop->motor_chassis[i].give_current = 0;
			}
			return;
    }
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
			chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
			temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
			if (max_vector < temp)
			{
				max_vector = temp;
			}
    }
				if (max_vector > MAX_WHEEL_SPEED)
				{
					vector_rate = MAX_WHEEL_SPEED / max_vector;
					for (i = 0; i < 4; i++)
					{
						chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
					}
				}
    //计算pid
		Amb_PID_cail(&CM_pid_1, chassis_move_control_loop->motor_chassis[0].speed_set, chassis_move_control_loop->motor_chassis[0].speed,LINE_ERR);
		Amb_PID_cail(&CM_pid_2, chassis_move_control_loop->motor_chassis[1].speed_set, chassis_move_control_loop->motor_chassis[1].speed,LINE_ERR);
		Amb_PID_cail(&CM_pid_3, chassis_move_control_loop->motor_chassis[2].speed_set, chassis_move_control_loop->motor_chassis[2].speed,LINE_ERR);
		Amb_PID_cail(&CM_pid_4, chassis_move_control_loop->motor_chassis[3].speed_set, chassis_move_control_loop->motor_chassis[3].speed,LINE_ERR);
    //赋值电流值
		chassis_move_control_loop->motor_chassis[0].give_current = (int16_t)CM_pid_1.out;
		chassis_move_control_loop->motor_chassis[1].give_current = (int16_t)CM_pid_2.out;
		chassis_move_control_loop->motor_chassis[2].give_current = (int16_t)CM_pid_3.out;
		chassis_move_control_loop->motor_chassis[3].give_current = (int16_t)CM_pid_4.out;

}

static fp32 data_rate_change_int16_to_fp32(fp32 max_fp32 , int16_t max_int16_t , int16_t input)
{
	fp32 output;
	output = (fp32)(max_fp32 * input / max_int16_t);
	return output;
}


