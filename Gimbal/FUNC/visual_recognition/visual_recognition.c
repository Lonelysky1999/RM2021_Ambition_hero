#include "visual_recognition.h"
#include "main.h"
#include "usart.h"
#include "sys.h"
#include "CAN_Receive.h"

static void deal_data(void);


static uint8_t rx_buffer[VISUAL_DATA_LEN];
visual_data_u visual_data_buffer;
visual_data_t visual_data;

float_u yaw_angle;
float_u pitch_angle;
float_u distance;


//static uint8_t error_flag = 0;

void visual_recognition_init(void)
{
	usart6_dma_init(rx_buffer,VISUAL_DATA_LEN);
}



void DMA2_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)!=RESET)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE); //关闭DMA,防止处理其间有数据		
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//清除DMA2_Steam7传输完成标志
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		
		deal_data();
		DMA_Cmd(DMA2_Stream1, ENABLE); 
	}
}

uint8_t tx_buffer[TX_BUFFER_LEN];
_judg_data_t * judg_data_visual_p;
void send_data_to_pc(uint8_t mmode)
{
	judg_data_visual_p = get_judg_data_point();
	if(judg_data_visual_p->robot_id == 1)
	{
		tx_buffer[2] = 'r';
	}
	else if(judg_data_visual_p->robot_id == 101)
	{
		tx_buffer[2] = 'b';
	}
	tx_buffer[0] = 0xa5;
	tx_buffer[9] = 0xee;
	if(mmode == 1)
	{
			//small
	tx_buffer[1] = 'f';
	}
	else
	{
			//big
	tx_buffer[1] = 'm';
	}


	usart_to_pc(tx_buffer,TX_BUFFER_LEN);
}


void usart_to_pc(uint8_t * txbuffer , int len)
{
	for(int i = 0 ; i < len; i++)
	{
		USART_SendData(USART6,txbuffer[i]);                        //
		while(USART_GetFlagStatus(USART6,USART_FLAG_TC)!=SET)
			;
	}
}


static void deal_data(void)
{
	yaw_angle.rx_buffer[0] = rx_buffer[2];
	yaw_angle.rx_buffer[1] = rx_buffer[3];
	yaw_angle.rx_buffer[2] = rx_buffer[4];
	yaw_angle.rx_buffer[3] = rx_buffer[5];
	
	pitch_angle.rx_buffer[0] = rx_buffer[6];
	pitch_angle.rx_buffer[1] = rx_buffer[7];
	pitch_angle.rx_buffer[2] = rx_buffer[8];
	pitch_angle.rx_buffer[3] = rx_buffer[9];
	
	distance.rx_buffer[0] = rx_buffer[10];
	distance.rx_buffer[1] = rx_buffer[11];
	distance.rx_buffer[2] = rx_buffer[12];
	distance.rx_buffer[3] = rx_buffer[13];
	
	
	
	
		visual_data.sof         = rx_buffer[0];
		visual_data.mode        = rx_buffer[1];
		visual_data.yaw_angle   = yaw_angle.f;
		visual_data.pitch_angle = pitch_angle.f;
		visual_data.distance    = distance.f;
		visual_data.command     = rx_buffer[14];
		visual_data.eof         = rx_buffer[15];
		visual_data.number++;

}

visual_data_t* return_visual_data_point(void)
{
	return &visual_data;
}
