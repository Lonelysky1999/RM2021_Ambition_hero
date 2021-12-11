#ifndef FPV_CONTROL_H
#define FPV_CONTROL_H
#include "main.h"
#include "sys.h"
#include "stdint.h"
#include "stdio.h"



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

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t left_button_down;
    uint8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} ext_robot_command_t;
typedef union 
{
    ext_robot_command_t robot_command_t;
    uint8_t usart_data[12];
}ext_fpv_control_u;


void decision_init(void);

int16_t return_mouse_x(void);
int16_t return_mouse_y(void);
int16_t return_mouse_z(void);

#endif

