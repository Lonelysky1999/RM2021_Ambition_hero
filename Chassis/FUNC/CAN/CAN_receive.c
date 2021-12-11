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
		
		
#define get_vxvywz_data(ptr, rx_message)																											\
		{																																													\
				(ptr)->vx_set  = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);    		\
        (ptr)->vy_set  = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);				\
				(ptr)->wz_set  = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);				\
				(ptr)->ch_mode = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);				\
		}

//统一处理can接收函数
static void CAN_hook(CanRxMsg *rx_message);
//声明电机变量
static motor_measure_t motor_chassis[4];
//云台返回变量数据
static chassis_rx_data_t CHRX_data;
//工控返回的数据
static cap_data_t cap_data; 
		
user_rx_data_t user_rx_data;
		
void get_cap_data(cap_data_t* data, CanRxMsg * rx_message);

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








/*can发送函数*******************************************************************************/
//CAN 发送 0x700的ID的数据，会引发M3508进入快速设置ID模式
void CAN_CMD_CHASSIS_RESET_ID(void)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = 0x700;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = 0;
    TxMessage.Data[4] = 0;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;
    CAN_Transmit(CAN_A, &TxMessage);
}
//发送底盘电机控制命令0x200
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;
    CAN_Transmit(CAN_A, &TxMessage);
}

//
void CAN_CMD_JUDGMENT_DATA(uint8_t cmd1)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_DESYS_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = cmd1;
    TxMessage.Data[1] = 1;
    TxMessage.Data[2] = 2;
    TxMessage.Data[3] = 3;
    TxMessage.Data[4] = 4;
    TxMessage.Data[5] = 5;
    TxMessage.Data[6] = 6;
    TxMessage.Data[7] = 7;
    CAN_Transmit(CAN_B, &TxMessage);
}


void CAN_CMD_POWER_DATA(uint16_t max_power)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_SEND_TO_POWC;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = 0;
    TxMessage.Data[1] = 0;
    TxMessage.Data[2] = 0;
    TxMessage.Data[3] = max_power >> 8;
    TxMessage.Data[4] = max_power;
    TxMessage.Data[5] = 0;
    TxMessage.Data[6] = 0;
    TxMessage.Data[7] = 0;
    CAN_Transmit(CAN_A, &TxMessage);
}


/******************************************************************************************/






/*数据返回函数******************************************************************************/
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
//返回底盘设定数据
const chassis_rx_data_t *get_CHRX_data_Point(void)
{
    return &CHRX_data;
}

const user_rx_data_t *get_user_rx_data_Point(void)
{
    return &user_rx_data;
}

const cap_data_t *get_cap_data_Point(void)
{
    return &cap_data;
}
/*****************************************************************************************/





static uint16_t cmd_id_2;

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
static void CAN_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
    {
			case CAN_RX_FROM_POWC://从功率控制模块返回的数据//NULL
			{
					get_cap_data(&cap_data,rx_message);
					break;
			}
			case CAN_GIMBAL_ID://
			{

				cmd_id_2 = (uint16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);
				user_rx_data.whip_switch    = (cmd_id_2 & 0x0008) >> 3;
				user_rx_data.visual_switch  = (cmd_id_2 & 0x0004) >> 2;
				user_rx_data.visual_mode    = (cmd_id_2 & 0x0002) >> 1;
				user_rx_data.visual_exist   = (cmd_id_2 & 0x0001);
				
				break;
			}
			case CAN_VXVYWZ_SET://从云台传过来的VXVYWZ的控制数据
			{
					get_vxvywz_data(&CHRX_data,rx_message);
					break;
			}
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			{
					static uint8_t i = 0;
					i = rx_message->StdId - CAN_3508_M1_ID;
					get_motor_measure(&motor_chassis[i], rx_message);
					break;
			}
			default:
			{
					break;
			}
    }
}

static rx_float_u rx_float;

void get_cap_data(cap_data_t* data, CanRxMsg * rx_message)
{
	for(int i = 0; i < 4; i++)
	{
		rx_float.rx_buffer[i] = rx_message->Data[i];
	}
	data->cap_i = rx_float.f;
	
	for(int i = 0; i < 4; i++)
	{
		rx_float.rx_buffer[i] = rx_message->Data[i + 4];
	}
	data->cap_u = rx_float.f;
	
	data->cap_p = (data->cap_i)*(data->cap_u);
}
