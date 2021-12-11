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

#define GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE 0

#define SysCoreClock 180

#define RC_NVIC 4
#define VISUAL_NVIC 6

#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 4
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5

#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef RAD_TO_ANG
#define RAD_TO_ANG 57.295779513082320876798154814105f
#endif


#ifndef ANG_TO_RAD
#define ANG_TO_RAD 0.01745329251994329576923690768489f
#endif

#ifndef EDC_TO_ANG
#define EDC_TO_ANG 0.0439453125f
#endif


#define OLED_DC_Pin GPIO_Pin_9
#define OLED_DC_GPIO_Port GPIOB

#define OLED_RST_Pin GPIO_Pin_10
#define OLED_RST_GPIO_Port GPIOB

#define USER_KEY GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2)



#endif

