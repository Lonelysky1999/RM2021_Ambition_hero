#ifndef AMBITION_PID
#define AMBITION_PID


#include "main.h"

#define LINE_ERR 0
#define LOOP_ERR 1

#define FIFO_DEEP 50




typedef struct
{
    uint8_t mode;
    
    //pid基本参数
    fp32 kp;
    fp32 ki;
    fp32 kd;
    //pid基本数据
    fp32 set;
    fp32 rel;
    fp32 out;
    //pid基本参数比例
    fp32 rate_kp;
    fp32 rate_ki;
    fp32 rate_kd;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
		fp32 Ufout;

		//输入处理
		fp32 last_set;
		fp32 last2_set;
    //偏差处理
    fp32 perr;
    fp32 ierr;
    fp32 derr;
		fp32 last_derr;
		fp32 d2err;
    fp32 err[3];//err_fifo_3

    //积分处理
    fp32 trapezoidal_integration_block[2];
    fp32 trapezoidal_integration_block_size;
		fp32 trapezoidal_integration_block_midpoint;
		
		//微分处理
		fp32 alphe;
		fp32 beita;
		
		//输入滤波器系数
		fp32 set_filter_kp;
		//不完全微分滤波系数
		fp32 incomplete_differential_filter_kp;

} _Amb_Pid_t;


extern void Abm_PID_init(_Amb_Pid_t* pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout, fp32 *trapezoidal_integration_block, fp32 set_filter, fp32 d_filter, fp32 alphe, fp32 beita);
extern fp32 Amb_PID_cail(_Amb_Pid_t* pid, fp32 set, fp32 rel, uint8_t mode);

#endif

