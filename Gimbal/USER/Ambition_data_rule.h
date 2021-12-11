#ifndef AMBITION_DATA_RULE
#define AMBITION_DATA_RULE
#include "sys.h"
#include "flash.h"

#define FLASH_DATA_SIZE (3*21+1)

#ifdef STM32F40XX
  #define AMBITION_DATA_DAAR_BEGIN ((uint32_t)0x080E0000)
#endif 

#ifdef STM32F427X
  #define AMBITION_DATA_DAAR_BEGIN ((uint32_t)0x081E0000)
#endif

#define DEBUG_MODE_C         0x00000000
#define BEGIN_GAME_MODE_C    0x00000001
#define UNDERWAY_GAME_MODE_C 0x00000002

typedef struct 
{
  //陀螺仪角速度零飘
	fp32 gyro_cali_temp_40_20 [21][3];
	//
	uint32_t board_sate;
}flash_data_list_t;

typedef union
{
  uint32_t flash_data_buffer[FLASH_DATA_SIZE];
  flash_data_list_t flash_data_list;
}flash_data_list_u;

uint32_t* get_flash_data_buffer_point(void);
uint32_t get_flash_data_buffer_len(void);
flash_data_list_t* get_flash_data_list_point(void);

flash_data_list_u* get_flash_union_data_point(void);


#endif
