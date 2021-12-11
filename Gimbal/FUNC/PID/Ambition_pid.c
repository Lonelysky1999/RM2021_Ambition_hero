#include "Ambition_pid.h"
#include "user_lib.h"
#include "arm_math.h"

#ifndef Ambition_2021
	#error "please define Ambition_2021"
#endif


static fp32 activation_function(_Amb_Pid_t* pid, fp32 input);
static fp32 pid_abs(fp32 input);
static fp32 pid_max_limit(fp32 input, fp32 max);
static void deal_err(_Amb_Pid_t* pid, uint8_t mode);

void Abm_PID_init(_Amb_Pid_t* pid, fp32 kp, fp32 ki, fp32 kd, fp32 max_out, fp32 max_iout, fp32 *trapezoidal_integration_block, fp32 set_filter, fp32 d_filter, fp32 alphe, fp32 beita)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
	

    pid->max_iout = max_iout;
    pid->max_out = max_out;

    pid->trapezoidal_integration_block[0] = trapezoidal_integration_block[0];
    pid->trapezoidal_integration_block[1] = trapezoidal_integration_block[1];
    pid->trapezoidal_integration_block_size = trapezoidal_integration_block[1] - trapezoidal_integration_block[0];
		pid->trapezoidal_integration_block_midpoint = (trapezoidal_integration_block[1] + trapezoidal_integration_block[0])/2;

    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
		pid->Ufout = 0.0f;

    pid->ierr = 0.0f;
    pid->derr = 0.0f;
		pid->last_derr = 0.0f;
		pid->d2err = 0.0f;
    pid->perr = 0.0f;

    pid->err[0] = 0.0f;
    pid->err[1] = 0.0f;
    pid->err[2] = 0.0f; 

    pid->rate_kp = 1.0f;
    pid->rate_ki = 1.0f;
    pid->rate_kd = 1.0f;
		
		pid->set_filter_kp = set_filter;
		pid->incomplete_differential_filter_kp = d_filter;
		
		pid->alphe = alphe;
		pid->beita = beita;
		
}

fp32 Amb_PID_cail(_Amb_Pid_t* pid, fp32 set, fp32 rel, uint8_t mode)
{
    fp32 _out;
    fp32 _iout;

		//set_fifo和输入低通滤波
		pid->last2_set = pid->last_set;
		pid->last_set = pid->set;
		pid->set = (1 - pid->set_filter_kp)*set + pid->set_filter_kp*pid->last_set;
	
    pid->rel = rel;
    deal_err(pid,mode);

    //KP
    pid->perr = pid->err[0];
    pid->Pout = pid->kp*pid->perr;    

    //KI
    pid->rate_ki = activation_function(pid, pid->err[0]);
    pid->ierr += pid->err[0]*pid->rate_ki;
    _iout = pid->ki*pid->ierr;
    pid->Iout = pid_max_limit(_iout, pid->max_iout);

    //KD
		if(mode == LOOP_ERR)
		{
			pid->derr = rad_format(pid->err[0] - pid->err[1]);
		}
		else
		{
			pid->derr = pid->err[0] - pid->err[1];
		}
		
    //pid->rate_kd = activation_function(pid, pid->err[0]);//梯形微分
		//pid->Dout = pid->kd*pid->derr*pid->rate_kd;//经典微分
		
		
		pid->Dout = pid->kd*pid->derr*(1 - pid->incomplete_differential_filter_kp) + pid->incomplete_differential_filter_kp*pid->last_derr;//不完全微分
		pid->last_derr = pid->Dout;
		
		if(mode == LOOP_ERR)
		{
			pid->Ufout = pid->alphe*rad_format(pid->set - pid->last_set) + pid->beita*rad_format(pid->set - 2*pid->last_set + pid->last2_set);//前馈控制器
		}
		else
		{
			pid->Ufout = pid->alphe*(pid->set - pid->last_set) + pid->beita*(pid->set - 2*pid->last_set + pid->last2_set);//前馈控制器
		}
		
		
    //OUT
    _out = (pid->Pout + pid->Iout + pid->Dout + pid->Ufout);
    pid->out = pid_max_limit(_out, pid->max_out);

    //return
    return pid->out;
}

static void deal_err(_Amb_Pid_t* pid, uint8_t mode)
{
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    if(mode == LOOP_ERR)
    {
        pid->err[0] = rad_format(pid->set - pid->rel);
    }
    else
    {
        pid->err[0] = pid->set - pid->rel;
    }
}

//激活函数
static fp32 activation_function(_Amb_Pid_t* pid, fp32 input)
{
		fp32 f_x_1;
    fp32 x;
    fp32 y;
		fp32 output;
    if(pid_abs(input) >= (pid->trapezoidal_integration_block[1]))
    {
        f_x_1 = -1.0f;
    }
    else if(pid_abs(input) <= (pid->trapezoidal_integration_block[0]))
    {
        f_x_1 = 1.0f;
    }
    else
    {
        f_x_1 = ((pid->trapezoidal_integration_block_midpoint) - pid_abs(input)) * 2 / pid->trapezoidal_integration_block_size;
    }
		x = f_x_1 * PI / 2.0f;
    y = sin(x);
		output = (y + 1.0f)/2.0f;
    return output;
}

static fp32 pid_abs(fp32 input)
{
    return (input < 0)?(-input):input;
}

static fp32 pid_max_limit(fp32 input, fp32 max)
{
    if(input > max)
    {
        return max;
    }
    else if(input < -max)
    {
        return -max;
    }
    else
    {
        return input;
    }
}

