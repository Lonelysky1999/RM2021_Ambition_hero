#ifndef DECISION_H
#define DECISION_H
#include "main.h"


#define BUFFER_MAX_LEN 32  //缓冲区最大长度

#define FRAME_HEADER_LEN 5  //帧头段长度
#define FRAME_CMD_LEN 2     //命令段长度
#define FRAME_TAIL 2        //帧尾段长度

#define END_OF_SEG (Buffer_point == Segment_len)
#define SOF_DATA 0xa5

//状态机枚举变量
typedef enum
{
    e_idle,
    e_frame_header,
    e_cmd_id,
    e_frame_data,
    e_frame_tail
} frame_mode_e;

//帧头段结构体
typedef struct
{
    uint8_t d_SOF;          //起始字节
    uint16_t d_data_length; //数据段长度
    uint8_t d_seq;          //包序号
    uint8_t d_CRC8;         //CRC8校验
}farme_header_data_t;

/**********************begin****************************/
//数据段数据结构体
//组1 全场发送数据
typedef __packed struct //0x0001 比赛状态数据
{                               //1HZ
		uint8_t game_type : 4;			//比赛类型
		uint8_t game_progress : 4;	//当前比赛阶段
		uint16_t stage_remain_time;	//当前阶段剩余时间
		uint64_t SyncTimeStamp;			//精确时间
} ext_game_state_t;
typedef __packed union 
{
    ext_game_state_t game_state;
    uint8_t usart_data[12];
}ext_game_state_u;


typedef __packed struct //0x0002 比赛结果数据
{                   //比赛结束后
    uint8_t winner; //比赛结果
} ext_game_result_t;
typedef __packed union 
{
    ext_game_result_t game_result;
    uint8_t usart_data[1];
}ext_game_result_u;

typedef __packed struct //0x0003 比赛机器人血量数据
{                               //1HZ
    uint16_t red_1_robot_HP;    //红1英雄机器人血量
    uint16_t red_2_robot_HP;    //红2工程机器人血量
    uint16_t red_3_robot_HP;    //红3步兵机器人血量
    uint16_t red_4_robot_HP;    //红4步兵机器人血量
    uint16_t red_5_robot_HP;    //红5步兵机器人血量
    uint16_t red_7_robot_HP;    //红7哨兵机器人血量
    uint16_t red_sentry_HP;     //红方前哨站血量
    uint16_t red_base_HP;       //红方基地血量
    uint16_t blue_1_robot_HP;   //蓝1英雄机器人血量
    uint16_t blue_2_robot_HP;   //蓝2工程机器人血量
    uint16_t blue_3_robot_HP;   //蓝3步兵机器人血量
    uint16_t blue_4_robot_HP;   //蓝4步兵机器人血量
    uint16_t blue_5_robot_HP;   //蓝5步兵机器人血量
    uint16_t blue_7_robot_HP;   //蓝7哨兵机器人血量
    uint16_t blue_sentry_HP;    //蓝方前哨站血量
    uint16_t blue_base_HP;      //蓝方基地血量
} ext_game_robot_HP_t;
typedef __packed union 
{
    ext_game_robot_HP_t game_robot_HP;
    uint8_t usart_data[32];
}ext_game_robot_HP_u;

typedef __packed struct //0x0004 全场飞镖发射状态
{                               //飞镖发射后
    uint8_t dart_shoot_team;    //飞镖发射队伍
    uint16_t dart_shoot_time;   //发射时剩余比赛时间
} ext_dart_data_t;
typedef __packed union 
{
    ext_dart_data_t dart_data;
    uint8_t usart_data[3];
}ext_dart_data_u;

//组2 己方发送的数据
typedef __packed struct //0x0101 场地事件数据
{                       //1HZ
    uint32_t event_type;//bit0-1：己方停机坪占领状态 
                        //bit2  ：小能量机关激活状态
                        //bit3  ：大能量机关激活状态
                        //bit4  ：虚拟护盾状态
} ext_event_data_t;
typedef __packed union 
{
    ext_event_data_t event_data;
    uint8_t usart_data[4];
}ext_event_data_u;

typedef __packed struct //0x0102 补给站动作标志数据
{                                   //动作触发后
    uint8_t supply_projectile_id;   //补给站ID
    uint8_t supply_robot_id;        //补弹机器人ID
    uint8_t supply_projectile_step; //出弹口打开状态
    uint8_t supply_projectile_num;  //补弹数量
} ext_supply_projectile_action_t;
typedef __packed union 
{
    ext_supply_projectile_action_t supply_projectile_action;
    uint8_t usart_data[4];
}ext_supply_projectile_action_u;

typedef __packed struct //0x0104 裁判警告数据
{                           //警告发生后触发
    uint8_t level;          //警告等级
    uint8_t foul_robot_id;  //犯规机器人ID
} ext_referee_warning_t;
typedef __packed union 
{
    ext_referee_warning_t referee_warning;
    uint8_t usart_data[2];
}ext_referee_warning_u;

typedef __packed struct //0x0105 飞镖发射口倒计时
{                                   //1HZ
    uint8_t dart_remaining_time;    //15s倒计时
} ext_dart_remaining_time_t;
typedef __packed union 
{
    ext_dart_remaining_time_t dart_remaining_time;
    uint8_t usart_data[1];
}ext_dart_remaining_time_u;

//组3单一机器人发送的数据
typedef __packed struct //0x0201 机器人状态数据
{                                           	//10HZ
    uint8_t robot_id;                       	//机器人ID
    uint8_t robot_level;                    	//机器人等级
    uint16_t remain_HP;                     	//机器人剩余血量
    uint16_t max_HP;                        	//机器人上限血量
		uint16_t shooter_id1_17mm_cooling_rate;  	//机器人 1 号 17mm 枪口每秒冷却值
		uint16_t shooter_id1_17mm_cooling_limit; 	//机器人 1 号 17mm 枪口热量上限
		uint16_t shooter_id1_17mm_speed_limit;   	//机器人 1 号 17mm 枪口上限速度 单位 m/s
		uint16_t shooter_id2_17mm_cooling_rate;  	//机器人 2 号 17mm 枪口每秒冷却值
		uint16_t shooter_id2_17mm_cooling_limit; 	//机器人 2 号 17mm 枪口热量上限
		uint16_t shooter_id2_17mm_speed_limit;   	//机器人 2 号 17mm 枪口上限速度 单位 m/s
		uint16_t shooter_id1_42mm_cooling_rate;		//机器人 42mm 枪口每秒冷却值
		uint16_t shooter_id1_42mm_cooling_limit;	//机器人 42mm 枪口热量上限
		uint16_t shooter_id1_42mm_speed_limit;		//机器人 42mm 枪口上限速度 单位 m/s
		uint16_t chassis_power_limit;							//机器人底盘功率限制上限
		uint8_t mains_power_gimbal_output : 1;		//0 bit： gimbal 口输出： 1 为有 24V 输出， 0 为无 24v 输出；
		uint8_t mains_power_chassis_output : 1;		//1 bit： chassis 口输出： 1 为有 24V 输出， 0 为无 24v 输出；
		uint8_t mains_power_shooter_output : 1;		//2 bit： shooter 口输出： 1 为有 24V 输出， 0 为无 24v 输出；

} ext_game_robot_state_t;
typedef __packed union 
{
    ext_game_robot_state_t game_robot_state;
    uint8_t usart_data[27];
}ext_game_robot_state_u;

typedef __packed struct //0x0202 实时功率热量数据
{                                   //50HZ
    uint16_t chassis_volt;          //底盘输出电压mv
    uint16_t chassis_current;       //底盘输出电流ma
    float chassis_power;            //底盘输出功率w
    uint16_t chassis_power_buffer;  //底盘缓冲功率j
		uint16_t shooter_id1_17mm_cooling_heat;	//1 号 17mm 枪口热量
		uint16_t shooter_id2_17mm_cooling_heat;	//2 号 17mm 枪口热量
		uint16_t shooter_id1_42mm_cooling_heat;	//42mm 枪口热量
} ext_power_heat_data_t;
typedef __packed union 
{
    ext_power_heat_data_t power_heat_data;
    uint8_t usart_data[16];
}ext_power_heat_data_u;

typedef __packed struct //0x0203 机器人位置数据
{               //10HZ
    float x;    //x轴坐标
    float y;    //y轴坐标
    float z;    //z轴坐标
    float yaw;  //yaw轴坐标
} ext_game_robot_pos_t;
typedef __packed union 
{
    ext_game_robot_pos_t game_robot_pos;
    uint8_t usart_data[16];
}ext_game_robot_pos_u;

typedef __packed struct //0x0204 机器人增益数据
{                               //1HZ
    uint8_t power_rune_buff;    //bit0：机器人血量补血状态
                                //bit1：枪口热量冷却加速
                                //bit2：机器人防御加成
                                //bit3：机器人攻击加成
} ext_buff_musk_t;
typedef __packed union 
{
    ext_buff_musk_t buff_musk;
    uint8_t usart_data[1];
}ext_buff_musk_u;

typedef __packed struct //0x0205 空中机器人能量状态
{                           //10HZ
    uint16_t attack_time;    //可攻击时间
} aerial_robot_energy_t;
typedef __packed union 
{
    aerial_robot_energy_t aerial_robot_energy;
    uint8_t usart_data[2];
}aerial_robot_energy_u;

typedef __packed struct //0x0206 伤害状态数据
{                           //伤害发生后
    uint8_t armor_type : 4; //收到伤害的装甲ID
    uint8_t hurt_type : 4;  //扣血类型
} ext_robot_hurt_t;
typedef __packed union 
{
    ext_robot_hurt_t robot_hurt;
    uint8_t usart_data[1];
}ext_robot_hurt_u;

typedef __packed struct //0x0207 实时射击数据
{                       //射击后
    uint8_t bullet_type;//子弹类型
		uint8_t shooter_id; //发射机构 ID：
    uint8_t bullet_freq;//子弹射频
    float bullet_speed; //子弹射速
} ext_shoot_data_t;
typedef __packed union 
{
    ext_shoot_data_t shoot_data;
    uint8_t usart_data[7];
}ext_shoot_data_u;

//////////////////////////////////////////////////////////////////////////////////////////////////////
typedef __packed struct //0x0208 弹丸剩余发射数，仅空中机器人，哨兵机器人，自动步兵发送
{                                   //1HZ
	uint16_t bullet_remaining_num_17mm;
	uint16_t bullet_remaining_num_42mm;
	uint16_t coin_remaining_num;
} ext_bullet_remaining_t;
typedef __packed union 
{
    ext_bullet_remaining_t bullet_remaining;
    uint8_t usart_data[6];
}ext_bullet_remaining_u;
///////////////////////////////////////////////////////////////////////////////////////////////////////

typedef __packed struct //0x0209 机器人RFID状态
{                       //1HZ
    uint32_t RFID_state;//bit0：基地增益点RFID状态
                        //bit1：高地增益点RFID状态
                        //bit2：能量机关激活点RFID状态
                        //bit3：飞坡增益点RFID状态
                        //bit4：前哨站增益点RFID状态
                        //bit5：资源岛增益点RFID状态
                        //bit6：补血点增益点RFID状态
                        //bit7：工程机器人补血卡RFID状态
} ext_RFID_state_t;
typedef __packed union 
{
    ext_RFID_state_t RFID_state;
    uint8_t usart_data[4];
}ext_RFID_state_u;

typedef __packed struct //0x0301 
{
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint16_t data_cmd_id;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef struct
{
    ext_game_state_u                _game_state;                //0x0001
    ext_game_result_u               _game_result;               //0x0002
    ext_game_robot_HP_u             _game_robot_HP;             //0x0003
    ext_dart_data_u                 _dart_data;                 //0x0004

    ext_event_data_u                _event_data;                //0x0101
    ext_supply_projectile_action_u  _supply_projectile_action;  //0x0102
    ext_referee_warning_u           _referee_warning;           //0x0104
    ext_dart_remaining_time_u       _dart_remaining_time;       //0x0105

    ext_game_robot_state_u          _game_robot_state;          //0x0201
    ext_power_heat_data_u           _power_heat_data;           //0x0202
    ext_game_robot_pos_u            _game_robot_pos;            //0x0203
    ext_buff_musk_u                 _buff_musk;                 //0x0204
    aerial_robot_energy_u           _aerial_robot_energy;       //0x0205
    ext_robot_hurt_u                _robot_hurt;                //0x0206
    ext_shoot_data_u                _shoot_data;                //0x0207
    ext_bullet_remaining_u          _bullet_remaining;          //0x0208
    ext_RFID_state_u                _RFID_state;                //0x0209

}decision_all_data_t;



decision_all_data_t* return_decision_data_point(void);

fp32 return_now_power(void);
uint8_t return_power_limit(void);
uint8_t return_shanghai_armor_type(void);
uint8_t return_shanghai_hurt_type(void);
uint16_t return_42_heat(void);


#endif
