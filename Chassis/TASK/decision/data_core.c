#include "data_core.h"
#include "decision.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "CAN_Receive.h"
#include "user_lib.h"
#include "ui.h"


#define HEAT_42 (task_dec_data->_power_heat_data.power_heat_data.shooter_id1_42mm_cooling_heat)
#define LIMIT_HEAT_42 (task_dec_data->_game_robot_state.game_robot_state.shooter_id1_42mm_cooling_limit)

#define HEAT_17 (task_dec_data->_power_heat_data.power_heat_data.shooter_id1_17mm_cooling_heat)
#define LIMIT_HEAT_17 (task_dec_data->_game_robot_state.game_robot_state.shooter_id1_17mm_cooling_limit)

#define MAX_SPEED_17 (task_dec_data->_game_robot_state.game_robot_state.shooter_id1_17mm_speed_limit)
#define MAX_SPEED_42 (task_dec_data->_game_robot_state.game_robot_state.shooter_id1_42mm_speed_limit)

void test_ui(void);
void user_ui_1(void);
void ui_float(float f,uint8_t my_color);
void scale_ui(uint16_t cnt);

decision_all_data_t* task_dec_data;
const cap_data_t * cap_data_ui;
//
const user_rx_data_t * user_rx_data_ui;

//
uint8_t can_cmd_1 = 0x00;


//42mm热量值
uint16_t _42mm_remain_heat;
uint8_t shoot_lock_42 = 0;
//17mm热量值
uint16_t _17mm_remain_heat;
uint8_t shoot_lock_17 = 0;

//
uint16_t send_data_cnt = 0;
uint16_t send_data_cnt2 = 0;
//

graphic_data_struct_t LINE1;
u16 tx_robot;
u16 rx_client;

//
uint16_t _remain_42mm_num = 0;
uint8_t  _small_shoot_power = 0;

void data_core_task(void *pvParameters)
{
	task_dec_data = return_decision_data_point();
	user_rx_data_ui = get_user_rx_data_Point();
	cap_data_ui = get_cap_data_Point();
	
	tx_robot  = task_dec_data->_game_robot_state.game_robot_state.robot_id;
	rx_client = tx_robot + 0x100;
	//UI
	clean_ui(tx_robot,rx_client);
	test_ui();
	while(1)
	{
		tx_robot  = task_dec_data->_game_robot_state.game_robot_state.robot_id;
		rx_client = tx_robot + 0x100;
		
		//42mm剩余热量
		_42mm_remain_heat = LIMIT_HEAT_42 - HEAT_42;
		//17mm剩余热量
		_17mm_remain_heat = LIMIT_HEAT_17 - HEAT_17;
		//42mm弹丸剩余数量
		_remain_42mm_num = task_dec_data->_bullet_remaining.bullet_remaining.bullet_remaining_num_42mm;
		//小摩擦轮电源
		_small_shoot_power = task_dec_data->_game_robot_state.game_robot_state.mains_power_gimbal_output;
		
		
		//bit0: 42mm热量锁
		if(_42mm_remain_heat >= 100)
		{
			can_cmd_1 = can_cmd_1 & 0xfe;
		}
		else
		{
			can_cmd_1 = can_cmd_1 | 0x01;
			//can_cmd_1 = can_cmd_1 & 0xfe;
		}
		
		//bit1: 17mm热量锁
		if(_17mm_remain_heat > 20)
		{
			can_cmd_1 = can_cmd_1 & 0xfd;
		}
		else
		{
			can_cmd_1 = can_cmd_1 | 0x02;
		}
		
		//bit2,3: 17mm射速上限
		switch (MAX_SPEED_17)
    {
    	case 15:
				can_cmd_1 = can_cmd_1 & 0xf3; 
				can_cmd_1 = can_cmd_1 | 0x00; //00
    		break;
    	case 18:
				can_cmd_1 = can_cmd_1 & 0xf3; 
				can_cmd_1 = can_cmd_1 | 0x04; //01
    		break;
			case 30:
				can_cmd_1 = can_cmd_1 & 0xf3; 
				can_cmd_1 = can_cmd_1 | 0x08; //10
    		break;
    	default:
				can_cmd_1 = can_cmd_1 & 0xf3; 
				can_cmd_1 = can_cmd_1 | 0x0c; //11
    		break;
    }
		
		//bit4: 42mm射速上限
		switch (MAX_SPEED_42)
    {
    	case 10:
				can_cmd_1 = can_cmd_1 & 0xef; 
				can_cmd_1 = can_cmd_1 | 0x00; //0
    		break;
    	case 16:
				can_cmd_1 = can_cmd_1 & 0xef; 
				can_cmd_1 = can_cmd_1 | 0x10; //0001
    		break;
    	default:
				can_cmd_1 = can_cmd_1 & 0xf3; 
				can_cmd_1 = can_cmd_1 | 0x0c; //11
    		break;
    }
		
		//bit5: 红蓝方
		if(tx_robot == 1)//红方英雄
		{
			can_cmd_1 = can_cmd_1 & 0xdf;
		}
		else if(tx_robot == 101)//蓝方英雄
		{
			can_cmd_1 = can_cmd_1 | 0x20;
		}
		
		//bit6: 剩余大弹丸数量
		if(_remain_42mm_num == 0)
		{
			can_cmd_1 = can_cmd_1 & 0xbf;
		}
		else
		{
			can_cmd_1 = can_cmd_1 | 0x40;
		}
		
		//bit7: 小摩擦轮供电
		if(_small_shoot_power == 0)
		{
			can_cmd_1 = can_cmd_1 & 0x7f;
		}
		else
		{
			can_cmd_1 = can_cmd_1 | 0x80;
		}
		
		
		CAN_CMD_JUDGMENT_DATA(can_cmd_1);
		
//客户端UI
		if(send_data_cnt >= 100)
		{
			user_ui_1();//状态提示
			ui_float(cap_data_ui->cap_u,1);
			send_data_cnt = 0;
		}
		else
		{
			send_data_cnt = send_data_cnt + 2;
		}
		
		
		if(send_data_cnt2 >= 2000)
		{
			test_ui();//车筐
			send_data_cnt2 = 0;
		}
		else
		{
			send_data_cnt2 = send_data_cnt2 + 2;
		}
		scale_ui(send_data_cnt2);//刻度
		
		vTaskDelay(2);
	}
}

//图层5
void scale_ui(uint16_t cnt)
{
	graphic_data_struct_t t;
	//无作用
	t.start_angle=0;
	t.end_angle=0;
	t.radius=20;
	//静态
	t.operate_tpye=1;
	t.graphic_tpye=0;
	t.layer=5;
	t.color=7;
	t.width=3;

	switch (cnt)
  {
  	case 100:
		{
			//水平线
			t.graphic_name[0] = '5';
			t.graphic_name[1] = '0';
			t.graphic_name[2] = '1';
			
			t.end_y=540;
			
			
			
			
			t.start_y=540;
									t.start_x=760;    /*400*/  	t.end_x=1160;
			drawOnePicture(draw1P,tx_robot,rx_client,t);
  		break;
		}
		case 200:
		{
			//竖线
			t.graphic_name[0] = '5';
			t.graphic_name[1] = '0';
			t.graphic_name[2] = '2';
			
			t.end_y=540;
			
			//380
			
			
			t.start_y=160;
									t.start_x=960;      	t.end_x=960;
			drawOnePicture(draw1P,tx_robot,rx_client,t);
  		break;
		}
		case 300:
		{
			//1米
			t.graphic_name[0] = '5';
			t.graphic_name[1] = '1';
			t.graphic_name[2] = '0';
			
			t.end_y=490;
			
			
			
			
			t.start_y=490;
									t.start_x=940;      	t.end_x=980;
			drawOnePicture(draw1P,tx_robot,rx_client,t);
  		break;
		}
//		case 400:
//		{
//			//2米
//			t.graphic_name[0] = '5';
//			t.graphic_name[1] = '2';
//			t.graphic_name[2] = '0';
//			
//			t.end_y=460;
//			
//			
//			
//			
//			t.start_y=460;
//									t.start_x=820;      	t.end_x=1100;
//			drawOnePicture(draw1P,tx_robot,rx_client,t);
//  		break;
//		}
		case 500:
		{
			//3米    新6米
			t.graphic_name[0] = '5';
			t.graphic_name[1] = '3';
			t.graphic_name[2] = '0';
			
			t.end_y=420;
			
			
			
			
			t.start_y=420;
									t.start_x=940;      	t.end_x=980;
			drawOnePicture(draw1P,tx_robot,rx_client,t);
  		break;
		}
//		case 600:
//		{
//			//4米
//			t.graphic_name[0] = '5';
//			t.graphic_name[1] = '4';
//			t.graphic_name[2] = '0';
//			
//			t.end_y=380;
//			
//			
//			
//			
//			t.start_y=380;
//									t.start_x=880;      	t.end_x=1040;
//			drawOnePicture(draw1P,tx_robot,rx_client,t);
//  		break;
//		}
//		case 700:
//		{
//			//5米
//			t.graphic_name[0] = '5';
//			t.graphic_name[1] = '5';
//			t.graphic_name[2] = '0';
//			
//			t.end_y=340;
//			
//			
//			
//			
//			t.start_y=340;
//									t.start_x=910;      	t.end_x=1010;
//			drawOnePicture(draw1P,tx_robot,rx_client,t);
//  		break;
//		}
//		case 800:
//		{
//			//6米
//			t.graphic_name[0] = '5';
//			t.graphic_name[1] = '6';
//			t.graphic_name[2] = '0';
//			
//			t.end_y=300;
//			
//			
//			
//			
//			t.start_y=300;
//									t.start_x=940;      	t.end_x=980;
//			drawOnePicture(draw1P,tx_robot,rx_client,t);
//  		break;
//		}
  	default:
  		break;
  }
							

	
	
}


//图层9
void test_ui(void)
{
			LINE1.graphic_name[0] = '0';
			LINE1.graphic_name[1] = '1';
			LINE1.graphic_name[2] = '2';
			LINE1.operate_tpye=1;
			LINE1.graphic_tpye=0;
			LINE1.layer=9;
			LINE1.color=1;
			LINE1.start_angle=0;
			LINE1.end_angle=0;
			LINE1.width=5;
			LINE1.start_x=570;
			LINE1.start_y=0;
			LINE1.radius=20;
			LINE1.end_x=670;
			LINE1.end_y=160;
			drawOnePicture(draw1P,tx_robot,rx_client,LINE1);
			LINE1.graphic_name[0] = '0';
			LINE1.graphic_name[1] = '1';
			LINE1.graphic_name[2] = '4';
			LINE1.operate_tpye=1;
			LINE1.graphic_tpye=0;
			LINE1.layer=9;
			LINE1.color=1;
			LINE1.start_angle=0;
			LINE1.end_angle=0;
			LINE1.width=5;
			LINE1.start_x=1350;
			LINE1.start_y=0;
			LINE1.radius=20;
			LINE1.end_x=1250;
			LINE1.end_y=160;
			drawOnePicture(draw1P,tx_robot,rx_client,LINE1);
}




//图层8
void user_ui_1(void)
{
	graphic_data_struct_t whip_;
	graphic_data_struct_t visual_switch_;
	
	if(user_rx_data_ui->whip_switch == 1)
	{
		whip_.graphic_name[0] = '1';
		whip_.graphic_name[1] = '0';
		whip_.graphic_name[2] = '0';
		whip_.operate_tpye=1;
		whip_.graphic_tpye=1;//矩形
		whip_.layer=8;
		whip_.color=0;
		whip_.start_angle=0;
		whip_.end_angle=0;
		whip_.width=5;
		whip_.start_x=1500;
		whip_.start_y=800;
		whip_.radius=20;
		whip_.end_x=1550;
		whip_.end_y=850;
	}
	else
	{
		whip_.graphic_name[0] = '1';
		whip_.graphic_name[1] = '0';
		whip_.graphic_name[2] = '0';
		whip_.operate_tpye=3;
		whip_.graphic_tpye=1;//矩形
		whip_.layer=8;
		whip_.color=0;
		whip_.start_angle=0;
		whip_.end_angle=0;
		whip_.width=5;
		whip_.start_x=1500;
		whip_.start_y=800;
		whip_.radius=20;
		whip_.end_x=1550;
		whip_.end_y=850;
	}
	drawOnePicture(draw1P,tx_robot,rx_client,whip_);
	
	if(user_rx_data_ui->visual_switch == 1)
	{
		visual_switch_.graphic_name[0] = '1';
		visual_switch_.graphic_name[1] = '0';
		visual_switch_.graphic_name[2] = '1';
		visual_switch_.operate_tpye=1;
		visual_switch_.graphic_tpye=1;//矩形
		visual_switch_.layer=8;
		visual_switch_.color=2;
		visual_switch_.start_angle=0;
		visual_switch_.end_angle=0;
		visual_switch_.width=20;
		visual_switch_.start_x=1600;
		visual_switch_.start_y=800;
		visual_switch_.radius=20;
		visual_switch_.end_x=1650;
		visual_switch_.end_y=850;
	}
	else
	{
		visual_switch_.graphic_name[0] = '1';
		visual_switch_.graphic_name[1] = '0';
		visual_switch_.graphic_name[2] = '1';
		visual_switch_.operate_tpye=2;
		visual_switch_.graphic_tpye=1;//矩形
		visual_switch_.layer=8;
		visual_switch_.color=2;
		visual_switch_.start_angle=0;
		visual_switch_.end_angle=0;
		if(user_rx_data_ui->visual_exist == 1)
		{
			visual_switch_.width=10;
		}
		else
		{
			visual_switch_.width=0;
		}
		visual_switch_.start_x=1600;
		visual_switch_.start_y=800;
		visual_switch_.radius=20;
		visual_switch_.end_x=1650;
		visual_switch_.end_y=850;
	}
	drawOnePicture(draw1P,tx_robot,rx_client,visual_switch_);
	
}

static uint8_t float_ui_start = 0;

//图层7
void ui_float(float f,uint8_t my_color)
{
	graphic_data_struct_t FLOAT_1;
	float_buffer_u float_buffer;
	float_buffer.f = (int32_t)(f * 1000);
	FLOAT_1.graphic_name[0] = 'c';
	FLOAT_1.graphic_name[1] = 'a';
	FLOAT_1.graphic_name[2] = 'p';
	if(float_ui_start == 0)
	{
		FLOAT_1.operate_tpye=1;
		float_ui_start++;
	}
	else
	{
		FLOAT_1.operate_tpye=2;
	}
	FLOAT_1.graphic_tpye=6;//浮点数
	FLOAT_1.layer=7;
	FLOAT_1.color=5;
	FLOAT_1.start_angle=20;//字体大小
	FLOAT_1.end_angle=2;		//小数点有效位数
	FLOAT_1.width=2;
	FLOAT_1.start_x=1700;
	FLOAT_1.start_y=700;
	FLOAT_1.radius=float_buffer.t_buffer.radius;
	FLOAT_1.end_x =float_buffer.t_buffer.end_x;
	FLOAT_1.end_y =float_buffer.t_buffer.end_y;
	drawOnePicture(draw1P,tx_robot,rx_client,FLOAT_1);
}


