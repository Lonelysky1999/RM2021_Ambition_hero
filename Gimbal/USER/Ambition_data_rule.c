#include "main.h"
#include "sys.h"
#include "Ambition_data_rule.h"
#include "flash.h"

flash_data_list_u flash_data_u;

flash_data_list_u* get_flash_union_data_point(void)
{
	return &flash_data_u;
}

uint32_t* get_flash_data_buffer_point(void)
{
    return flash_data_u.flash_data_buffer;
}

uint32_t get_flash_data_buffer_len(void)
{
    return FLASH_DATA_SIZE;
}

flash_data_list_t* get_flash_data_list_point(void)
{
    return &(flash_data_u.flash_data_list);
}


