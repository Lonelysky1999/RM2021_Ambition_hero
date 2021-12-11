#include "INS_task.h"
#include "led_task.h"

#include "stm32f4xx.h"

#include "buzzer.h"
#include "timer.h"
#include "led.h"
#include "spi.h"
#include "exit_init.h"
#include "IST8310driver.h"
#include "mpu6500driver.h"
#include "mpu6500reg.h"
#include "mpu6500driver_middleware.h"
#include "Ambition_data_rule.h"

#include "AHRS.h"

#include "pid.h"


#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#define IMUWarnBuzzerOn() buzzer_on(95, 10000) //开机陀螺仪校准蜂鸣器

#define IMUWarnBuzzerOFF() buzzer_off() //开机陀螺仪校准蜂鸣器关闭

#define SET_TEMP 36.5f //35度以下跑不下去

#define MPU6500_TEMPERATURE_PWM_INIT() TIM3_Init(MPU6500_TEMP_PWM_MAX, 1) //陀螺仪温度控制PWM初始化
#define IMUTempPWM(pwm) TIM_SetCompare2(TIM3, (pwm))                      //pwm给定
#define INS_GET_CONTROL_TEMPERATURE() get_control_temperate()             //获取控制温度的目标值

#if defined(MPU6500_USE_DATA_READY_EXIT)

#define MPU6500_DATA_READY_EXIT_INIT() GPIOB_Exti8_GPIO_Init() //初始化mpu6500的 外部中断 使用PB8 外部中断线 8

#define MPU6500_DATA_READY_EXIT_IRQHandler EXTI9_5_IRQHandler //宏定义外部中断函数，使用了line8外部中断

#define MPU6500_DATA_READY_EXIT_Line EXTI_Line8 //宏定义外部中断线
#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

//宏定义初始化SPI的DMA，同时设置SPI为8位，4分频
#define MPU6500_SPI_DMA_Init(txbuf, rxbuf)                                 \
    {                                                                      \
        SPI5_DMA_Init((uint32_t)txbuf, (uint32_t)rxbuf, DMA_RX_NUM);       \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Rx, ENABLE);                   \
        SPI_I2S_DMACmd(SPI5, SPI_I2S_DMAReq_Tx, ENABLE);                   \
        SPI5SetSpeedAndDataSize(SPI_BaudRatePrescaler_8, SPI_DataSize_8b); \
    }

#define MPU6500_SPI_DMA_Enable() SPI5_DMA_Enable(DMA_RX_NUM) // 开始一次SPI的DMA传输
//宏定义SPI的DMA传输中断函数以及传输中断标志位
#define MPU6500_DMA_IRQHandler DMA2_Stream5_IRQHandler
#define MPU6500_DMA_Stream DMA2_Stream5
#define MPU6500_DMA_FLAG DMA_FLAG_TCIF5
#elif defined(MPU6500_USE_SPI_DMA)
#error "the communication of mpu6500 is not SPI, can't use the DMA"
#endif


//如果使用mpu6500的数据准备外部中断，可以使用任务通知方法唤醒任务
#if defined(MPU6500_USE_DATA_READY_EXIT) || defined(MPU6500_USE_SPI_DMA)
static TaskHandle_t INSTask_Local_Handler;
#endif

//DMA的SPI 发送的buf，以INT_STATUS开始连续读取 DMA_RX_NUM大小地址的值
#if defined(MPU6500_USE_SPI_DMA)
static const uint8_t mpu6500_spi_DMA_txbuf[DMA_RX_NUM] =
    {
        MPU_INT_STATUS | MPU_SPI_READ_MSB};
#endif

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INSTaskStack;
#endif

#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \

//处理陀螺仪，加速度计，磁力计数据的线性度，零漂
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310);
static void IMU_temp_Control(fp32 temp);

static uint8_t mpu6500_spi_rxbuf[DMA_RX_NUM]; //保存接收的原始数据
static mpu6500_real_data_t mpu6500_real_data; //转换成国际单位的MPU6500数据
static ist8310_real_data_t ist8310_real_data;                //转换成国际单位的IST8310数据

static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //陀螺仪校准线性度
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //加速度校准线性度
static fp32 Mag_Scale_Factor[3][3] = {{1.0f, 0.0f, 0.0f},
                                      {0.0f, 1.0f, 0.0f},
                                      {0.0f, 0.0f, 1.0f}}; //磁力计校准线性度

//从flash获得的零飘，临时存放点
//计算时用到的零飘
static uint16_t start_gyro_cali_time = 0;//静态变量初始化一次
static fp32 Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};           //陀螺仪零漂
static fp32 Accel_Offset[3] = {0.0f, 0.0f, 0.0f};          //加速度零漂
static fp32 Mag_Offset[3] = {0.0f, 0.0f, 0.0f};            //磁力计零漂

//温度比例零飘
fp32 gyro_cali_temp_40_20 [21][3] = {0.0f};
static void INS_set_cali_gyro(void);
flash_data_list_t* flash_data;

//

static const float TimingTime = INS_DELTA_TICK * 0.001f;   //任务运行的时间 单位 s

//陀螺仪数据
static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};

//欧拉角和四元数
static fp32 INS_Angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; //四元数

//温控pid
static const fp32 imuTempPID[3] = {MPU6500_TEMPERATURE_PID_KP, MPU6500_TEMPERATURE_PID_KI, MPU6500_TEMPERATURE_PID_KD};
static PidTypeDef imuTempPid;

static uint8_t first_temperate = 1;
	



void INSTask(void *pvParameters)
{
    vTaskDelay(INS_TASK_INIT_TIME);
		flash_data = get_flash_data_list_point();//获得flash数据的指针

    //初始化mpu6500，失败进入死循环
    while (mpu6500_init() != MPU6500_NO_ERROR)
    {
        ;
    }

//初始化ist8310，失败进入死循环
#if defined(USE_IST8310)
    while (ist8310_init() != IST8310_NO_ERROR)
    {
        ;
    }
#endif


#if defined(MPU6500_USE_DATA_READY_EXIT) || defined(MPU6500_USE_SPI_DMA)
    //获取当前任务的任务句柄，用于任务通知
    INSTask_Local_Handler = xTaskGetHandle(pcTaskGetName(NULL));
#endif

#if defined(MPU6500_USE_DATA_READY_EXIT)
    //初始化mpu6500的数据准备的外部中断
    MPU6500_DATA_READY_EXIT_INIT();
#else

    //如果不使用外部中断唤醒任务的方法，则使用传统的任务切换的方法
    TickType_t INS_LastWakeTime;
    INS_LastWakeTime = xTaskGetTickCount();

#endif

//初始化SPI的DMA传输的方法
#if defined(MPU6500_USE_SPI_DMA) && defined(MPU6500_USE_SPI)
    MPU6500_SPI_DMA_Init(mpu6500_spi_DMA_txbuf, mpu6500_spi_rxbuf);

#endif

    while (1)
    {

#if defined(MPU6500_USE_DATA_READY_EXIT)
        //等待外部中断中断唤醒任务
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
#else
        //任务延时切换任务
        vTaskDelayUntil(&INS_LastWakeTime, INS_DELTA_TICK);
//在延时任务切换的情况，开启DMA传输
#ifdef MPU6500_USE_SPI_DMA
        MPU6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
        while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        {
        }
#endif

#endif

//如果不使用SPI的方法，则使用普通SPI通信的方法
#ifndef MPU6500_USE_SPI_DMA
        mpu6500_read_muli_reg(MPU_INT_STATUS, mpu6500_spi_rxbuf, DMA_RX_NUM);
#endif

        //将读取到的mpu6500原始数据处理成国际单位的数据
        mpu6500_read_over((mpu6500_spi_rxbuf + MPU6500_RX_BUF_DATA_OFFSET), &mpu6500_real_data);

//将读取到的ist8310原始数据处理成国际单位的数据
#if defined(USE_IST8310)
        ist8310_read_over((mpu6500_spi_rxbuf + IST8310_RX_BUF_DATA_OFFSET), &ist8310_real_data);
#endif
        //减去零漂以及旋转坐标系
				INS_set_cali_gyro();
        IMU_Cali_Slove(INS_gyro, INS_accel, INS_mag, &mpu6500_real_data, &ist8310_real_data);


        //加速度计低通滤波
        static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
        static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
        static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};


        //判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
        static uint8_t updata_count = 0;//静态变量只初始化一次

        if( mpu6500_real_data.status & 1 << MPU_DATA_READY_BIT)
        {
            if (updata_count == 0)//第一次进入
            {
                MPU6500_TEMPERATURE_PWM_INIT();//温度pwm初始化
                PID_Init(&imuTempPid, PID_DELTA, imuTempPID, MPU6500_TEMPERATURE_PID_MAX_OUT, MPU6500_TEMPERATURE_PID_MAX_IOUT);//温度PID

                //初始化四元数
                AHRS_init(INS_quat, INS_accel, INS_mag);
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
                accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
                accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
                updata_count++;//
            }
            else
            {
                //加速度计低通滤波
                accel_fliter_1[0] = accel_fliter_2[0];
                accel_fliter_2[0] = accel_fliter_3[0];

                accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

                accel_fliter_1[1] = accel_fliter_2[1];
                accel_fliter_2[1] = accel_fliter_3[1];

                accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

                accel_fliter_1[2] = accel_fliter_2[2];
                accel_fliter_2[2] = accel_fliter_3[2];

                accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

                //更新四元数
                AHRS_update(INS_quat, TimingTime, INS_gyro, accel_fliter_3, INS_mag);
                get_angle(INS_quat, INS_Angle, INS_Angle + 1, INS_Angle + 2);

                //陀螺仪开机校准
								if(flash_data->board_sate == BEGIN_GAME_MODE_C)
                {
                    if (start_gyro_cali_time < GYRO_OFFSET_START_TIME)//校准计时
                    {
                        if( first_temperate)//是否达到设定温度判断
                        {
													led_red_off();
                          //当进入gyro_offset函数，如果无运动start_gyro_cali_time++，如果有运动 start_gyro_cali_time = 0
                          gyro_offset(Gyro_Offset, INS_gyro, mpu6500_real_data.status, &start_gyro_cali_time);//零飘计算
                        }
                    }
                    else if (start_gyro_cali_time == GYRO_OFFSET_START_TIME)
                    {
												amb_write_flash();
												led_red_on();
                        start_gyro_cali_time++;
												set_offset_cail_is_ok();
                    }
                }//陀螺仪开机校准   code end
								else if((flash_data->board_sate == UNDERWAY_GAME_MODE_C) || (flash_data->board_sate == DEBUG_MODE_C))
								{
									led_red_on();
                  start_gyro_cali_time = GYRO_OFFSET_START_TIME+1;
									set_offset_cail_is_ok();
								}
            }//update count if   code end
        }//mpu6500 status  if end
        IMU_temp_Control(mpu6500_real_data.temp);	
#if INCLUDE_uxTaskGetStackHighWaterMark
        INSTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }//while(1) end
}//task function end


//设置flash中预存的角速度零飘值
static void INS_set_cali_gyro(void)
{
	static int32_t temp = 0;
	int32_t addr = 0;
	temp = (int32_t)mpu6500_real_data.temp;
	if(temp<20)
		temp = 20;
	if(temp>40)
		temp = 40;
	addr = 40 - temp;
	
	if(start_gyro_cali_time == 0)
	{
	  Gyro_Offset[0] = gyro_cali_temp_40_20[addr][0];
	  Gyro_Offset[1] = gyro_cali_temp_40_20[addr][1];
	  Gyro_Offset[2] = gyro_cali_temp_40_20[addr][2];
	}
	else if(start_gyro_cali_time == GYRO_OFFSET_START_TIME)
	{
		gyro_cali_temp_40_20[addr][0] = Gyro_Offset[0];
	  gyro_cali_temp_40_20[addr][1] = Gyro_Offset[1];
	  gyro_cali_temp_40_20[addr][2] = Gyro_Offset[2];
	}
	else if(start_gyro_cali_time > GYRO_OFFSET_START_TIME)
	{
		return;
	}
}

void write_data_to_flash(flash_data_list_u* buffer_u)
{
	for(int i = 0; i < 21; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			buffer_u->flash_data_list.gyro_cali_temp_40_20[i][j] = gyro_cali_temp_40_20[i][j];
		}
	}
	
}

void write_flash_to_data(flash_data_list_u* buffer_u)
{
	for(int i = 0; i < 21; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			gyro_cali_temp_40_20[i][j] = buffer_u->flash_data_list.gyro_cali_temp_40_20[i][j];
		}
	}
}


//获得零飘的数据指针
const fp32 *get_gyro_offset_point(void)
{
    return Gyro_Offset;
}

//获得欧拉角数据的指针
const fp32 *get_INS_angle_point(void)
{
    return INS_Angle;
}

//获得角速度数据的指针
const fp32 *get_MPU6500_Gyro_Data_Point(void)
{
    return INS_gyro;
}

//获得加速度数据的指针
const fp32 *get_MPU6500_Accel_Data_Point(void)
{
    return INS_accel;
}

fp32 get_MPU6500_temp(void)
{
    return mpu6500_real_data.temp;
}

//处理零飘
static void IMU_Cali_Slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], mpu6500_real_data_t *mpu6500, ist8310_real_data_t *ist8310)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = mpu6500->gyro[0] * Gyro_Scale_Factor[i][0] + mpu6500->gyro[1] * Gyro_Scale_Factor[i][1] + mpu6500->gyro[2] * Gyro_Scale_Factor[i][2] + Gyro_Offset[i];
        accel[i] = mpu6500->accel[0] * Accel_Scale_Factor[i][0] + mpu6500->accel[1] * Accel_Scale_Factor[i][1] + mpu6500->accel[2] * Accel_Scale_Factor[i][2] + Accel_Offset[i];
        mag[i] = ist8310->mag[0] * Mag_Scale_Factor[i][0] + ist8310->mag[1] * Mag_Scale_Factor[i][1] + ist8310->mag[2] * Mag_Scale_Factor[i][2] + Mag_Offset[i];
    }
}

//温度控制
static void IMU_temp_Control(fp32 temp)
{
    uint16_t tempPWM;
    if (first_temperate)//达到温度的话
    {
        PID_Calc(&imuTempPid, temp, SET_TEMP);
        if (imuTempPid.out < 0.0f)
        {
            imuTempPid.out = 0.0f;
        }
        tempPWM = (uint16_t)imuTempPid.out;
        IMUTempPWM(tempPWM);
    }
    else//没到设定温度时
    {
        if (temp > SET_TEMP)//判断是否到达了最大温度
        {
						//达到设置温度，将积分项设置为一半最大功率，加速收敛
						first_temperate = 1;
						imuTempPid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
						IMUTempPWM(imuTempPid.Iout);
        }
				else
				{
						IMUTempPWM(MPU6500_TEMP_PWM_MAX - 1);//在没有达到设置的温度，一直最大功率加热
				}
				

    }
}

#if defined(MPU6500_USE_DATA_READY_EXIT)

void MPU6500_DATA_READY_EXIT_IRQHandler(void)
{
    if (EXTI_GetITStatus(MPU6500_DATA_READY_EXIT_Line) != RESET)
    {

        EXTI_ClearITPendingBit(MPU6500_DATA_READY_EXIT_Line);

//如果开启DMA传输 唤醒任务由DMA中断完成
#if defined(MPU6500_USE_SPI_DMA)
        mpu6500_SPI_NS_L();
        MPU6500_SPI_DMA_Enable();
#else

        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR((INSTask_Local_Handler), &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

#endif
    }
}

#endif

#if defined(MPU6500_USE_SPI) && defined(MPU6500_USE_SPI_DMA)

void MPU6500_DMA_IRQHandler(void)
{
    if (DMA_GetFlagStatus(MPU6500_DMA_Stream, MPU6500_DMA_FLAG))
    {
        DMA_ClearFlag(MPU6500_DMA_Stream, MPU6500_DMA_FLAG);
        mpu6500_SPI_NS_H();

        //唤醒任务
        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
        {
            static BaseType_t xHigherPriorityTaskWoken;
            vTaskNotifyGiveFromISR(INSTask_Local_Handler, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

#endif



