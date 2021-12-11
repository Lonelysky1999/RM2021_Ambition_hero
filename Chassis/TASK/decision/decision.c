#include "decision.h"
#include "usart.h"
#include "main.h"
#include "sys.h"
#include "stdio.h"

//pritnf
//struct __FILE 
//{ 
//	int handle; 
//}; 

//FILE __stdout;

//int fputc(int ch, FILE *f)
//{ 	
//	while((USART1->SR&0X40)==0);
//	USART1->DR = (u8) ch;      
//	return ch;
//}
//


//设置状态机，并在状态切换的时候写入段长度
static void decision_mode_set(uint8_t Res);
//在段切换的时候重置缓冲区指针
static void decision_reset_point(void);
//将数据写入buffer，并在传输完成后传入结构体
static void decision_write_buffer(uint8_t Res);
//写入数据段数据
static void decision_write_datasegment_struct(uint16_t cmd_id);

//从缓冲区写入共用体
static void write_data_from_buffer_to_uion(uint8_t*a,uint16_t data_max_len);
static void write_data_from_buffer_to_uion_201(uint8_t*a,uint16_t data_max_len);


uint8_t RX_buffer[BUFFER_MAX_LEN] = {0};//数据缓冲区
uint16_t Buffer_point = 0;   //数据缓冲区指针
uint16_t Segment_len = 0;    //段长度

//接收状态机
frame_mode_e farme_mode = e_idle;
frame_mode_e farme_mode_q = e_idle;

//各个段的数据
farme_header_data_t farme_header_data;//帧头段数据
uint16_t d_cmd_id = 0x0000;//命令段数据
uint16_t d_CRC16 = 0x0000;//结尾段数据即CRC16

//数据段总结构体
decision_all_data_t decision_all_data;

decision_all_data_t* return_decision_data_point(void)
{
	return &decision_all_data;
}

void decision_init(void)
{
    //初始化状态机
    farme_mode = e_idle;
    farme_mode_q = e_idle;
}

//设置状态机，并在状态切换的时候写入段长度
static void decision_mode_set(uint8_t Res)
{
    farme_mode_q = farme_mode;
    switch (farme_mode)
    {
        case  e_idle:
        {
            if(Res == SOF_DATA)
            {
                farme_mode = e_frame_header;    //状态机跳转到帧头段
                Segment_len = FRAME_HEADER_LEN;//设定帧头段的长度为5个字节
            }
            break; 
        }
        case  e_frame_header:
        {
            if(Buffer_point == Segment_len)
            {
                farme_mode = e_cmd_id;      //状态机跳转到命令段
                Segment_len = FRAME_CMD_LEN;//设定命令段的长度为2个字节
            }
            break; 
        }
        case  e_cmd_id:
        {
            if(Buffer_point == Segment_len)
            {
                farme_mode = e_frame_data;  //跳转到数据段
                Segment_len = farme_header_data.d_data_length;//设置数据段长度
            }
            break; 
        }
        case  e_frame_data:
        {
            if(Buffer_point == Segment_len)
            {
                farme_mode = e_frame_tail;  //跳转到帧尾段
                Segment_len = FRAME_TAIL;   //设定帧尾段的长度为2个字节
            }
            break; 
        }
        case  e_frame_tail:
        {
            if((Buffer_point == Segment_len) && Res == SOF_DATA)
            {
                farme_mode = e_frame_header;
                Segment_len = FRAME_HEADER_LEN;//设定帧头段的长度为5个字节
            }
            break; 
        }
        default:
            break;
    }
}
//在段切换的时候重置缓冲区指针
static void decision_reset_point(void)
{
    if(farme_mode_q != farme_mode)
    {
        Buffer_point = 0;
        return;
    }
    else
    {
        return;
    }   
}
//将数据写入buffer，并在传输完成后传入结构体
static void decision_write_buffer(uint8_t Res)
{
    if(farme_mode == e_idle)
    {
        return;//在总线空闲状态下直接返回
    }
    else
    {
        //写入缓冲区
        if(Buffer_point < Segment_len)
        {
            RX_buffer[Buffer_point] = Res;//数据写入缓冲区
            Buffer_point++; //写入缓冲区完成后,指针向前移一个位置
        }
        else
        {
            return;
        }
        //写入结构体
        if(END_OF_SEG)
        {
            switch (farme_mode)
            {
                case e_frame_header://写入起始段结构体
                {
                    farme_header_data.d_SOF = RX_buffer[0];
                    farme_header_data.d_data_length = (uint16_t)(RX_buffer[2] << 8 | RX_buffer[1]);
                    farme_header_data.d_seq = RX_buffer[3];
                    farme_header_data.d_CRC8 = RX_buffer[4];
                    break;
                }
                case e_cmd_id://写入命令段结构体
                {
                    d_cmd_id = (uint16_t)(RX_buffer[1] << 8 | RX_buffer[0]);
                    break;
                }
                case e_frame_data://写入数据段结构体
                {
                    decision_write_datasegment_struct(d_cmd_id);
                    break;
                }
                case e_frame_tail://写入结尾段结构体
                {
                    d_CRC16 = (uint16_t)(RX_buffer[0] << 8 | RX_buffer[1]);
                    break;
                }
                default:
                    break;
            }
        }
    }
}
//写入数据段数据
static void decision_write_datasegment_struct(uint16_t cmd_id)
{
    switch (cmd_id)
    {
        case 0x0001:
            write_data_from_buffer_to_uion(decision_all_data._game_state.usart_data,3);
        break;
        case 0x0002:
            write_data_from_buffer_to_uion(decision_all_data._game_result.usart_data,1);
        break;
        case 0x0003:
            write_data_from_buffer_to_uion(decision_all_data._game_robot_HP.usart_data,32);
        break;
        case 0x0004:
            write_data_from_buffer_to_uion(decision_all_data._dart_data.usart_data,3);
        break;
        
        case 0x0101:
            write_data_from_buffer_to_uion(decision_all_data._event_data.usart_data,4);
        break;
        case 0x0102:
            write_data_from_buffer_to_uion(decision_all_data._supply_projectile_action.usart_data,4);
        break;
        case 0x0104:
            write_data_from_buffer_to_uion(decision_all_data._referee_warning.usart_data,2);
        break;
        case 0x0105:
            write_data_from_buffer_to_uion(decision_all_data._dart_remaining_time.usart_data,1);
        break;

        case 0x0201:
            write_data_from_buffer_to_uion(decision_all_data._game_robot_state.usart_data,27);
        break;
        case 0x0202:
            write_data_from_buffer_to_uion(decision_all_data._power_heat_data.usart_data,16);
        break;
        case 0x0203:
            write_data_from_buffer_to_uion(decision_all_data._game_robot_pos.usart_data,16);
        break;
        case 0x0204:
            write_data_from_buffer_to_uion(decision_all_data._buff_musk.usart_data,1);
        break;
        case 0x0205:
            write_data_from_buffer_to_uion(decision_all_data._aerial_robot_energy.usart_data,2);
        break;
        case 0x0206:
            write_data_from_buffer_to_uion(decision_all_data._robot_hurt.usart_data,1);
        break;
        case 0x0207:
            write_data_from_buffer_to_uion(decision_all_data._shoot_data.usart_data,7);
        break;
        case 0x0208:
            write_data_from_buffer_to_uion(decision_all_data._bullet_remaining.usart_data,6);
        break;
        case 0x0209:
            write_data_from_buffer_to_uion(decision_all_data._RFID_state.usart_data,4);
        break;    
        default:
            break;
    }
}

static void write_data_from_buffer_to_uion(uint8_t*a,uint16_t data_max_len)
{
    uint16_t i = 0;
    for(;i < data_max_len;i++)
    {
        a[i] = RX_buffer[i];
    }
}


static void write_data_from_buffer_to_uion_201(uint8_t*a,uint16_t data_max_len)
{
	uint16_t i = 0;
	for(;i < data_max_len;i++)
	{
			a[i] = RX_buffer[i];
	}
	
	a[25] = RX_buffer[24];
	a[24] = RX_buffer[25];
	
}



//uint8_t i = 0;
//uint8_t test_buffer[125] = {0};

void USART2_IRQHandler(void)
{
	static uint8_t Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		Res = USART_ReceiveData(USART2);
		decision_mode_set(Res);
		decision_reset_point();
		decision_write_buffer(Res);
	}
}

//返回
fp32 return_now_power(void)
{
	return decision_all_data._power_heat_data.power_heat_data.chassis_power;
}

uint16_t return_42_heat(void)
{
	return decision_all_data._power_heat_data.power_heat_data.shooter_id1_42mm_cooling_heat;
}

//返回
uint8_t return_power_limit(void)
{
	return decision_all_data._game_robot_state.game_robot_state.chassis_power_limit;
}

uint8_t return_shanghai_armor_type(void)
{
	return decision_all_data._robot_hurt.robot_hurt.armor_type;
}

uint8_t return_shanghai_hurt_type(void)
{
	return decision_all_data._robot_hurt.robot_hurt.hurt_type;
}
