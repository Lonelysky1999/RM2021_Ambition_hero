#include "CAN_Receive.h"

#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//底盘电机数据读取
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//云台电机数据读取
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//统一处理can接收函数
static void CAN_hook(CanRxMsg *rx_message);
//声明电机变量
static motor_measure_t motor_yaw, motor_pit, motor_trigger, shoot_motor_L, shoot_motor_R, small_trigger;
static CanTxMsg GIMBAL_TxMessage;
		
_judg_data_t judg_rx_data;

		
		
//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN_hook(&rx1_message);
    }
}
//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN_hook(&rx2_message);
    }
}


//发送云台控制命令，其中rev为保留字节
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, uint16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (shoot >> 8);
    GIMBAL_TxMessage.Data[5] = shoot;
    GIMBAL_TxMessage.Data[6] = (rev >> 8);
    GIMBAL_TxMessage.Data[7] = rev;
    CAN_Transmit(CAN_B,  &GIMBAL_TxMessage );
}
//摩擦轮和小拨弹盘电机控制命令
void CAN_CMD_CHASSIS(int16_t motorL, int16_t motorR, int16_t trigger, int16_t rev)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motorL >> 8;
    TxMessage.Data[1] = motorL;
    TxMessage.Data[2] = motorR >> 8;
    TxMessage.Data[3] = motorR;
    TxMessage.Data[4] = trigger >> 8;
    TxMessage.Data[5] = trigger;
    TxMessage.Data[6] = rev >> 8;
    TxMessage.Data[7] = rev;
    CAN_Transmit(CAN_C, &TxMessage);
}

void CAN_CMD_CH(int16_t vx, int16_t vy, int16_t wz, int16_t rev)
{
		CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CH_VXVYWZ;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = (vx >> 8);
    TxMessage.Data[1] = vx;
    TxMessage.Data[2] = (vy >> 8);
    TxMessage.Data[3] = vy;
    TxMessage.Data[4] = (wz >> 8);
    TxMessage.Data[5] = wz;
    TxMessage.Data[6] = (rev >> 8);
    TxMessage.Data[7] = rev;
    CAN_Transmit(CAN_B,  &TxMessage );
}


//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void)
{
    return &motor_pit;
}
//返回trigger电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Trigger_Motor_Measure_Point(void)
{
    return &motor_trigger;
}
//返回左右摩擦轮数据
const motor_measure_t *get_Shoot_Motor_L_Measure_Point(void)
{
    return &shoot_motor_L;
}
const motor_measure_t *get_Shoot_Motor_R_Measure_Point(void)
{
    return &shoot_motor_R;
}
//返回小拨弹轮轮数据
const motor_measure_t *get_Small_Trigger_Measure_Point(void)
{
    return &small_trigger;
}

extern _judg_data_t* get_judg_data_point(void)
{
	return &judg_rx_data;
}


//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_YAW_MOTOR_ID:
			{
					get_gimbal_motor_measuer(&motor_yaw, rx_message);
					break;
			}
			case CAN_PIT_MOTOR_ID:
			{
					get_gimbal_motor_measuer(&motor_pit, rx_message);
					break;
			}
			case CAN_TRIGGER_MOTOR_ID:
			{
					get_motor_measure(&motor_trigger, rx_message);
					break;
			}
			case CAN_3508_L_ID:
			{
					get_motor_measure(&shoot_motor_L, rx_message);
					break;
			}
			case CAN_3508_R_ID:
			{
					get_motor_measure(&shoot_motor_R, rx_message);
					break;
			}
			case CAN_2006_M3_ID:
			{
					get_motor_measure(&small_trigger, rx_message);
					break;
			}
			case CAN_DESYS_DATA:
			{
					judg_rx_data.shoot_lock_42     =  ((rx_message)->Data[0]) & 0x01;
					judg_rx_data.shoot_lock_17     = (((rx_message)->Data[0]) & 0x02)>>1;
					//judg_rx_data.robot_id          = (((rx_message)->Data[0]) & 0x20)>>5;
					judg_rx_data.remain_42mm_num   = (((rx_message)->Data[0]) & 0x40)>>6;
					judg_rx_data.small_shoot_power = (((rx_message)->Data[0]) & 0x80)>>7;
					
				if((((rx_message)->Data[0]) & 0x20)>>5 == 0)
				{
					judg_rx_data.robot_id = 1;
				}
				else if((((rx_message)->Data[0]) & 0x20)>>5 == 1)
				{
					judg_rx_data.robot_id = 101;
				}

				switch (((rx_message)->Data[0]) & 0x0c)
          {
          	case 0x00:
							judg_rx_data.max_17_speed = 15;
          		break;
          	case 0x04:
							judg_rx_data.max_17_speed = 18;
          		break;
						case 0x08:
							judg_rx_data.max_17_speed = 30;
          		break;
          	default:
							judg_rx_data.max_17_speed = 15;
          		break;
          }
					switch (((rx_message)->Data[0]) & 0x10)
          {
          	case 0x00:
							judg_rx_data.max_42_speed = 10;
          		break;
          	case 0x10:
							judg_rx_data.max_42_speed = 16;
          		break;
          	default:
							judg_rx_data.max_42_speed = 10;
          		break;
          }
					break;
			}
			default:
			{
					break;
			}
    }
}
