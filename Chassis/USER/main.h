#ifndef MAIN_H
#define MAIN_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#define SysCoreClock 180

#define CAN1_NVIC 4
#define CAN2_NVIC 4

#define OLED_DC_Pin GPIO_Pin_9
#define OLED_DC_GPIO_Port GPIOB

#define OLED_RST_Pin GPIO_Pin_10
#define OLED_RST_GPIO_Port GPIOB

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif



#endif

