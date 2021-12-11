#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CAN_B CAN1
#define CAN_C CAN2

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_L_ID = 0x201,
    CAN_3508_R_ID = 0x202,
    CAN_2006_M3_ID = 0x203,
		
		CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,//大拨弹轮3508

    CAN_CH_VXVYWZ = 0x252,        //发送给底盘x和y轴的设定值
		CAN_DESYS_DATA = 0x255    //裁判系统数据
} can_msg_id_e;

//rm电机统一数据结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;


typedef struct
{
	uint8_t shoot_lock_42;
  uint8_t shoot_lock_17;
  uint8_t max_42_speed;
  uint8_t max_17_speed;
  uint8_t robot_id;
  uint8_t remain_42mm_num;
  uint8_t small_shoot_power;
  uint8_t rev_7;
}_judg_data_t;



//发送云台控制命令，其中rev为保留字节
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, uint16_t rev);
//大摩擦轮和小拨弹盘电机控制命令
extern void CAN_CMD_CHASSIS(int16_t motorL, int16_t motorR, int16_t trigger, int16_t rev);
//底盘控制量发送
extern void CAN_CMD_CH(int16_t vx, int16_t vy, int16_t wz, int16_t rev);


//返回yaw电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//返回trigger电机变量地址，通过指针方式获取原始数据
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
extern const motor_measure_t *get_Shoot_Motor_L_Measure_Point(void);
extern const motor_measure_t *get_Shoot_Motor_R_Measure_Point(void);
extern const motor_measure_t *get_Small_Trigger_Measure_Point(void);

extern _judg_data_t* get_judg_data_point(void);

#endif
