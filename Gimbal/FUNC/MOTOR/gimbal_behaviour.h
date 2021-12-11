#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����
  GIMBAL_INIT,           //��̨��ʼ��
  GIMBAL_CALI,           //��̨У׼
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
extern uint8_t return_init_mode(void);

#endif
