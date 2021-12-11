
#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CAN_A CAN2
#define CAN_B CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
		CAN_GIMBAL_ID  = 0x1ff,

    CAN_SEND_TO_POWC    = 0x303,    //发送到功率控制板的消息
    CAN_RX_FROM_POWC    = 0x008,    //从功率控制板返回回来的消息
    CAN_VXVYWZ_SET	   	= 0x252,    //
    CAN_DESYS_DATA      = 0x255    //裁判系统数据

} can_msg_id_e;

typedef union 
{  
    uint8_t rx_buffer[4];
	  float f; 
	
}rx_float_u;

typedef struct
{
	
	float cap_i;
	float cap_u;
	float cap_p;
	
}cap_data_t;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

//底盘姿态和控制量结构体
typedef struct
{
    fp32 ch_yaw;
    fp32 ch_pitch;
    fp32 ch_roll;

    int16_t wz_set;
    int16_t vx_set;
    int16_t vy_set;
		int16_t ch_mode;
} chassis_rx_data_t;


typedef struct
{
	uint8_t whip_switch;
	uint8_t visual_switch;
	uint8_t visual_mode;
	uint8_t visual_exist;
} user_rx_data_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//发送底盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_CMD_JUDGMENT_DATA(uint8_t cmd1);
extern void CAN_CMD_POWER_DATA(uint16_t max_power);
//返回云台过来的设定值
extern const chassis_rx_data_t *get_CHRX_data_Point(void);
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

extern const user_rx_data_t *get_user_rx_data_Point(void);

extern const cap_data_t *get_cap_data_Point(void);

#endif
