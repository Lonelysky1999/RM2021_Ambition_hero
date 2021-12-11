#ifndef VISUAL_RECOGNITION
#define VISUAL_RECOGNITION
#include "main.h"

#define VISUAL_DATA_LEN 18
#define TX_BUFFER_LEN 10

void visual_recognition_init(void);

typedef struct
{
	uint8_t sof;
	uint8_t mode;
	fp32 yaw_angle;
	fp32 pitch_angle;
	fp32 distance;
	uint8_t command;
	uint8_t eof;
	uint16_t res;
}_visual_data_t;

typedef union
{
	fp32 f;
	uint8_t rx_buffer[4];
}float_u;



typedef union
{
	_visual_data_t visual_data_t;
	uint8_t rx_buffer[VISUAL_DATA_LEN];
}visual_data_u;



typedef struct
{
	uint32_t number;
	uint8_t sof;
	uint8_t mode;
	fp32 yaw_angle;
	fp32 pitch_angle;
	fp32 distance;
	uint8_t command;
	uint8_t eof;
	uint16_t res;
}visual_data_t;


visual_data_t* return_visual_data_point(void);
void usart_to_pc(uint8_t * txbuffer , int len);
void send_data_to_pc(uint8_t mmode);

#endif

