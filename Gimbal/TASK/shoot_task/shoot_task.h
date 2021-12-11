#ifndef SHOOT_TASK
#define SHOOT_TASK
#include "main.h"
#include "sys.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "Remote_Control.h"
#include "Gimbal_Task.h"
#include "user_lib.h"
#include "Ambition_pid.h"


//鼠标长按判断
#define PRESS_LONG_TIME 400

//微动开关IO
#define Butten_BIG GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10) //Q1
#define Butten_SMA GPIO_ReadInputDataBit(GPIOI, GPIO_Pin_9)  //Q2
//大弹丸发射步骤
#define BIGSHOT_IST_SHOOT 1
#define BIGSHOT_ING_SHOOT 2
#define BIGSHOT_WAS_SHOOT 3
//小弹丸发射步骤
#define SMASHOT_IST_SHOOT 1
#define SMASHOT_ING_SHOOT 2
#define SMASHOT_WAS_SHOOT 3

//电机rmp旋转速度转化比例3508
#define Motor_RMP_TO_SPEED_3508 0.00551156605892946182186428663734f
#define Motor_ECD_TO_ANGLE_3508 0.000040367915470674769203107568144525f

//电机rmp 变化成 旋转速度的比例2006
#define Motor_RMP_TO_SPEED_2006 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE_2006 0.000021305288720633905968306772076277f

//PID宏
//大摩擦轮
#define BIG_SHOOT_SPEED_PID_KP 55.1f//低速29
#define BIG_SHOOT_SPEED_PID_KI 0.0f
#define BIG_SHOOT_SPEED_PID_KD 0.0f//15.0
#define BIG_SHOOT_SPEED_PID_MAX_OUT 16000.0f
#define BIG_SHOOT_SPEED_PID_MAX_IOUT 4000.0f

#define BIG_SHOOT_L_SPEED_PID_KP 65.1f
#define BIG_SHOOT_L_SPEED_PID_KI 0.00f
#define BIG_SHOOT_L_SPEED_PID_KD 5.0f
#define BIG_SHOOT_L_SPEED_PID_MAX_OUT 16000.0f
#define BIG_SHOOT_L_SPEED_PID_MAX_IOUT 4000.0f

#define BIG_SHOOT_R_SPEED_PID_KP 65.1f
#define BIG_SHOOT_R_SPEED_PID_KI 0.00f
#define BIG_SHOOT_R_SPEED_PID_KD 5.0f
#define BIG_SHOOT_R_SPEED_PID_MAX_OUT 16000.0f
#define BIG_SHOOT_R_SPEED_PID_MAX_IOUT 4000.0f

//大摩擦轮差速
#define BIG_SHOOT_D_SPEED_PID_KP 0.18f
#define BIG_SHOOT_D_SPEED_PID_KI 0.009f
#define BIG_SHOOT_D_SPEED_PID_KD 1.2f
#define BIG_SHOOT_D_SPEED_PID_MAX_OUT 100.0f
#define BIG_SHOOT_D_SPEED_PID_MAX_IOUT 10.0f







//大拨弹轮速度环PID
#define BIG_TRIGGER_SPEED_PID_KP 1500.0f
#define BIG_TRIGGER_SPEED_PID_KI 0.05f
#define BIG_TRIGGER_SPEED_PID_KD 0.0f//10000.0f
#define BIG_TRIGGER_SPEED_PID_MAX_OUT 16384.0f
#define BIG_TRIGGER_SPEED_PID_MAX_IOUT 6000.0f
//大拨弹轮位置环PID
#define BIG_TRIGGER_ANGLE_PID_KP 15.0f
#define BIG_TRIGGER_ANGLE_PID_KI 0.2f
#define BIG_TRIGGER_ANGLE_PID_KD 1.0f
#define BIG_TRIGGER_ANGLE_PID_MAX_OUT 20.0f
#define BIG_TRIGGER_ANGLE_PID_MAX_IOUT 5.0f







//小拨弹轮速度环pid
#define SMALL_TRIGGER_SPEED_PID_KP 1400.0f
#define SMALL_TRIGGER_SPEED_PID_KI 0.1f
#define SMALL_TRIGGER_SPEED_PID_KD 20.0f
#define SMALL_TRIGGER_SPEED_PID_MAX_OUT 15000.0f
#define SMALL_TRIGGER_SPEED_PID_MAX_IOUT 5000.0f
//小拨弹轮位置环pid
#define SMALL_TRIGGER_ANGLE_PID_KP 10.0f
#define SMALL_TRIGGER_ANGLE_PID_KI 0.09f
#define SMALL_TRIGGER_ANGLE_PID_KD 10.0f
#define SMALL_TRIGGER_ANGLE_PID_MAX_OUT 40.0f
#define SMALL_TRIGGER_ANGLE_PID_MAX_IOUT 1.0f


//#define SMALL_TRIGGER_DIAL_PIECE 1
#define BIG_TRIGGER_DIAL_PIECE 4.9475f

//#define SINGLE_SMALL_TRIGGER_ADD_ANGLE ((2)*(PI)/(SMALL_TRIGGER_DIAL_PIECE))
#define SINGLE_BIG_TRIGGER_ADD_ANGLE ((2)*(PI)/(BIG_TRIGGER_DIAL_PIECE))

//大发射机构状态机
typedef enum
{
  Shoot_stop,

  Shoot_ready,

  Shoot_clean
} big_shoot_mode_e;

//小发射机构状态机
typedef enum
{
  shoot_stop,

  shoot_ready,

  shoot_clean
} small_shoot_mode_e;



typedef struct//角度环pid
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;
		fp32 derr;
		fp32 last_err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Trigger_angle_PID_t;

//两个拨弹轮
typedef struct 
{
  const motor_measure_t *trigger_motor;   //拨弹轮
  PidTypeDef trigger_motor_speed_pid;           //pid
	Trigger_angle_PID_t trigger_motor_angle_pid;
	int edc;
	int last_edc;
  fp32 speed;
  fp32 speed_set;
  fp32 angle;
  fp32 angle_set;
  bool_t angle_set_lock;
	bool_t move_set_lock;
  int16_t given_current;

  //拨弹轮角度计算
  int8_t ecd_count;

  //速度的二阶低通滤波
  fp32 speed_fliter_1;
  fp32 speed_fliter_2;
  fp32 speed_fliter_3;
}my_trigger_t;



//发射任务函数总结构体
typedef struct
{
  //数据指针
  const RC_ctrl_t *shoot_RC;
  const motor_measure_t *shoot_motor_L;   //左大摩擦轮
  const motor_measure_t *shoot_motor_R;   //右大摩擦轮

  //拨弹轮
  my_trigger_t big_trigger;       //大拨弹轮
  my_trigger_t small_trigger;     //小拨弹轮

	//amb_pid
	_Amb_Pid_t shoot_motor_L_Apid;
	_Amb_Pid_t shoot_motor_R_Apid;
	
	

  //大摩擦轮设定值
  fp32 speed_L;                   //左摩擦轮转速
  fp32 speed_raw_set_L;						//左摩擦轮转速原始设定值
  fp32 speed_set_L;               //左摩擦轮转速缓启动设定值
  int16_t given_current_L;        //左摩擦轮总线发送值
  fp32 speed_R;                   //右摩擦轮转速
  fp32 speed_raw_set_R;						//右摩擦轮转速原始设定值
  fp32 speed_set_R;               //右摩擦轮总线缓启动设定值
  int16_t given_current_R;        //右摩擦轮总线发送值

  //小摩擦轮设定值
  uint16_t snail_pwm_set_L;
  uint16_t snail_pwm_set_R;
	uint16_t snail_pwm_raw_set_L;
  uint16_t snail_pwm_raw_set_R;


  //大摩擦轮缓启动滤波器
  first_order_filter_type_t shoot_motor_L_filter;
  first_order_filter_type_t shoot_motor_R_filter;
	
	//小摩擦轮缓启动滤波器
	first_order_filter_type_t snail_L_filter;
  first_order_filter_type_t snail_R_filter;
	
  //状态机
  big_shoot_mode_e bigshoot_mode;         //大发射机构状态
  small_shoot_mode_e smallshoot_mode;     //小发射机构状态

  //鼠标控制变量
  bool_t press_l;                             //鼠标左键状态
  bool_t press_r;                             //鼠标右键状态
  bool_t last_press_l;                        //鼠标左键上一次状态
  bool_t last_press_r;                        //鼠标右键上一次状态
  uint16_t press_l_time;                      //鼠标左键按下的时间
  uint16_t press_r_time;                      //鼠标右键按下的时间


} shoot_sys_t;



extern void Shoot_task(void *pvParameters);
extern int16_t get_big_trigger_motor_current(void);
extern uint16_t return_snail_set(void);
extern uint16_t return_debug_snail_set(void);
extern shoot_sys_t* return_shoot_data_p(void);

#endif

