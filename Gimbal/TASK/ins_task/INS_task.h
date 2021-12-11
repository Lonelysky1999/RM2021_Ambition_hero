#ifndef INS_Task_H
#define INS_Task_H
#include "main.h"
#include "Ambition_data_rule.h"

#define USE_IST8310 //????IST8310???,???????

#define MPU6500_USE_DATA_READY_EXIT //????MPU6500?????,???????

#define MPU6500_USE_SPI_DMA //????SPI?DMA??,???????

//????IST8310,DMA??23???,????,?7???,?16???
#if defined(USE_IST8310)
#define DMA_RX_NUM 23
#else
#define DMA_RX_NUM 16
#endif

//mpu6500????????buf???
#ifdef MPU6500_USE_SPI_DMA
#define MPU6500_RX_BUF_DATA_OFFSET 1
#else
#define MPU6500_RX_BUF_DATA_OFFSET 0
#endif

//ist83100????????buf???
#ifdef MPU6500_USE_SPI_DMA
#define IST8310_RX_BUF_DATA_OFFSET 16
#else
#define IST8310_RX_BUF_DATA_OFFSET 15
#endif

#define MPU6500_TEMPERATURE_PID_KP 1800.0f //????PID?kp
#define MPU6500_TEMPERATURE_PID_KI 0.4f    //????PID?ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //????PID?kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //????PID?max_out
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //????PID?max_iout

#define INS_DELTA_TICK 1 //???????

#define INS_TASK_INIT_TIME 7 //?????? delay ????

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500???????TIM????,??PWM??? MPU6500_TEMP_PWM_MAX - 1

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 2
#define INS_ROLL_ADDRESS_OFFSET 1

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2


#define TEMP_40 0
#define TEMP_39 1
#define TEMP_38 2
#define TEMP_37 3
#define TEMP_36 4
#define TEMP_35 5
#define TEMP_34 6
#define TEMP_33 7
#define TEMP_32 8
#define TEMP_31 9
#define TEMP_30 10
#define TEMP_29 11
#define TEMP_28 12
#define TEMP_27 13
#define TEMP_26 14
#define TEMP_25 15
#define TEMP_24 16
#define TEMP_23 17
#define TEMP_22 18
#define TEMP_21 19
#define TEMP_20 20

extern void INSTask(void *pvParameters);

extern const fp32 *get_gyro_offset_point(void);

extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);
fp32 get_MPU6500_temp(void);

extern void write_data_to_flash(flash_data_list_u* buffer_u);
extern void write_flash_to_data(flash_data_list_u* buffer_u);


#endif

