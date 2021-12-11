i#include "ui.h"
#include "sys.h"
#include "main.h"
#include "crc8_crc16.h"



#define   frame_header_offset    5      //frame_header


static void copy(u8* buffer,u8* mubiao,int changdu);
static void copy_s(u8* buffer,u8* buffer_t,int lenth);
static void sendData (u8 *buffer,int lenth);
static void addCRC(u8* buffer,int length);

static void sendData (u8 *buffer,int lenth)
{
	for(int i = 0; i < lenth; i++)
	{
		USART_SendData(USART2,buffer[i]);                        //
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=SET)
			;
	}
}
static void addCRC(u8* buffer,int length)
{
	    append_CRC8_check_sum(buffer,5);
	    append_CRC16_check_sum(buffer,length);
}

u8 bao=0;
  

void drawOnePicture(u16 ID,u16 sendID,u16 receptionID,graphic_data_struct_t PICTURE)
{

	draw1picture tx_buffer_t;
	uint8_t tx_buffer[30];
	
	  tx_buffer_t.data_length=0x0015;                //
	  tx_buffer_t.seq=bao++;                         //
	  tx_buffer_t.CRC8=0;                            //
	  tx_buffer_t.cmd_id= RtoR;                      //
	  tx_buffer_t.ID = ID;                           //
	  tx_buffer_t.sendID=sendID;                     //
	  tx_buffer_t.receptionID=receptionID;           //
	  tx_buffer_t.picture1=PICTURE;                  //
    tx_buffer_t.CRC16=0;                           //

	  tx_buffer[0]=0xA5;
	  copy(&tx_buffer[1],(u8*)&tx_buffer_t.data_length,2);
	  tx_buffer[3]=tx_buffer_t.seq;
	  copy(&tx_buffer[frame_header_offset]  , (u8*)&tx_buffer_t.cmd_id     , 2);
	  copy(&tx_buffer[frame_header_offset+2], (u8*)&tx_buffer_t.ID         , 2);
	  copy(&tx_buffer[frame_header_offset+4], (u8*)&tx_buffer_t.sendID     , 2);
	  copy(&tx_buffer[frame_header_offset+6], (u8*)&tx_buffer_t.receptionID, 2);
    copy_s(&tx_buffer[frame_header_offset+8], (u8*)&tx_buffer_t.picture1   , 15);
		 
    addCRC(tx_buffer,30);
	  sendData(tx_buffer,30);
}





void clean_ui(u16 sendID,u16 receptionID)
{
		clean_ui_t tx_buffer_t;
		uint8_t tx_buffer[17];
	
	  tx_buffer_t.data_length=0x0008;                //
	  tx_buffer_t.seq=bao++;                         //
	  tx_buffer_t.CRC8=0;                            //
	  tx_buffer_t.cmd_id= RtoR;                      //
	  tx_buffer_t.ID = 0x0100;                           //
	  tx_buffer_t.sendID=sendID;                     //
	  tx_buffer_t.receptionID=receptionID;           //
		tx_buffer_t.operate_tpye = 2;
		tx_buffer_t.layer = 0;
    tx_buffer_t.CRC16=0;                           //

	  tx_buffer[0]=0xA5;
	  copy(&tx_buffer[1],(u8*)&tx_buffer_t.data_length,2);
	  tx_buffer[3]=tx_buffer_t.seq;
	  copy(&tx_buffer[frame_header_offset]  , (u8*)&tx_buffer_t.cmd_id       , 2);
	  copy(&tx_buffer[frame_header_offset+2], (u8*)&tx_buffer_t.ID           , 2);
	  copy(&tx_buffer[frame_header_offset+4], (u8*)&tx_buffer_t.sendID       , 2);
	  copy(&tx_buffer[frame_header_offset+6], (u8*)&tx_buffer_t.receptionID  , 2);
    copy(&tx_buffer[frame_header_offset+8], (u8*)&tx_buffer_t.operate_tpye , 1);
		copy(&tx_buffer[frame_header_offset+9], (u8*)&tx_buffer_t.layer        , 1);
		 
    addCRC(tx_buffer,17);
	  sendData(tx_buffer,17);
}


void copy(u8* buffer,u8* buffer_t,int lenth)
{
	for(int i = 0; i < lenth; i++)
	{
		buffer[i] = buffer_t[i];
	}
}

void copy_s(u8* buffer,u8* buffer_t,int lenth)
{
	for(int i = 0; i < lenth; i++)
	{
		if(i < 3)
		{
			buffer[i] = buffer_t[i];
		}
		else if(i >= 3)
			
		buffer[i] = buffer_t[i + 1];
	}
}

