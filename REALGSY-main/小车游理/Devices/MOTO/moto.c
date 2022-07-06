/**
  ******************************************************************************
  * @file    moto.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   �����ʼ�����
  ******************************************************************************
  */
#include "moto.h"

/**************************************************************************
�������ܣ�����ģ���ʼ�����Ƶ�������IO
��ڲ�������
����  ֵ����
���ţ�PB14 PB15 PB12 PB13
**************************************************************************/
void MOTO_Init1(void)//��ʼ�����Ƶ�������IO
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//PB14 15(���1) PB12 13(���2) �������
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStruct);	
}

void MOTO_Init2(void)//��ʼ�����Ƶ�������IO
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	//PF1 2 3 4 �������
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOF,&GPIO_InitStruct);	
}

/**************************************************************************
�������ܣ���ʼ��pwm�������
��ڲ�������
����  ֵ����
���ţ�
**************************************************************************/
void pwm_Init(void) 
{	
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	//	TIM_DeInit(TIM8);    
	/*���Ĵ�����������Ĭ��ֵ*/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7|GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//����ʱ��
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_Period = 7199;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_Pulse = 0;
	//��ʼ��CCR������ռ�ձȣ�
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	//���ʹ��
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	//PWM����ģʽ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	//�������Ч����ʼ���θߣ�

	/*��ʼ��ͨ��*/
	TIM_OC1Init(TIM8, &TIM_OCInitStruct);
	TIM_OC2Init(TIM8, &TIM_OCInitStruct);
	TIM_OC3Init(TIM8, &TIM_OCInitStruct);
	TIM_OC4Init(TIM8, &TIM_OCInitStruct);

	/*Ԥװ��ʹ��*/
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	/*�����ʹ��*/
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	/*ʹ��TIMx��ARR��Ԥװ�ؼĴ���*/
	TIM_Cmd(TIM8, ENABLE);
}
