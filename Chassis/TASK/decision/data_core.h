#ifndef DATA_CORE_H
#define DATA_CORE_H

#include "main.h"

typedef enum
{
    l_speed_0,
    l_speed_1,
    l_speed_2,
    h_speed_0,
    h_speed_1,
		h_speed_2
} speed_level_e;

typedef struct
{
	fp32      now_speed;
	uint16_t  max_speed;
}_42mm_shoot_data_t;



typedef struct
{
	uint32_t radius:10; 
  uint32_t end_x:11; 
  uint32_t end_y:11;
}float_buffer_1_t;

typedef union
{
	float_buffer_1_t t_buffer;
	int32_t f;
}float_buffer_u;

extern void data_core_task(void *pvParameters);

#endif

