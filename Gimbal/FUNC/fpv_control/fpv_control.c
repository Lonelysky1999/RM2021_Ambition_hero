#include "fpv_control.h"


//设置状态机，并在状态切换的时候写入段长度
static void decision_mode_set(uint8_t Res);
//在段切换的时候重置缓冲区指针
static void decision_reset_point(void);
//将数据写入buffer，并在传输完成后传入结构体
static void decision_write_buffer(uint8_t Res);

//从缓冲区写入共用体
static void write_data_from_buffer_to_uion(uint8_t*a,uint16_t data_max_len);

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
ext_fpv_control_u fpv_control_u;

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
                Segment_len = 12;//设置数据段长度
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
                    write_data_from_buffer_to_uion(fpv_control_u.usart_data,12);
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

static void write_data_from_buffer_to_uion(uint8_t*a,uint16_t data_max_len)
{
    uint16_t i = 0;
    for(;i < data_max_len;i++)
    {
        a[i] = RX_buffer[i];
    }
}


void UART8_IRQHandler(void)
{
	static uint8_t Res;
	if(USART_GetITStatus(UART8, USART_IT_RXNE) != RESET)
	{
		Res = USART_ReceiveData(UART8);
		decision_mode_set(Res);
		decision_reset_point();
		decision_write_buffer(Res);
	}
}


int16_t return_mouse_x(void)
{
	return fpv_control_u.robot_command_t.mouse_x;
}
int16_t return_mouse_y(void)
{
	return fpv_control_u.robot_command_t.mouse_y;
}
int16_t return_mouse_z(void)
{
	return fpv_control_u.robot_command_t.mouse_z;
}

