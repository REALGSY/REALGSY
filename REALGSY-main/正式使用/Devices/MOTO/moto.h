#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h"

void MOTO_Init1(void);    //��ʼ�����Ƶ�����������
void MOTO_Init2(void);//��ʼ�����Ƶ�������IO
void pwm_Init(void); //PWM�����ʼ��
#define PWMA   TIM8->CCR1
#define PWMB   TIM8->CCR2
#define PWMA2  TIM8->CCR3
#define PWMB2  TIM8->CCR4

#define AIN2   PBout(15)
#define AIN1   PBout(14)
#define BIN1   PBout(13)
#define BIN2   PBout(12)
#define AIN22  PFout(1)
#define AIN21  PFout(2)
#define BIN21  PFout(4)
#define BIN22  PFout(3)

#endif

