#include "Servo.h"
void TIM1_PWM (void) 
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  //ʹ�ܶ�ʱ��1ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); //ʹ��GPIOE��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;   //PA8
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;         //�������
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//��Ƶ����
	TIM_TimeBaseInitStruct.TIM_Period = 19999;                     //�趨�������Զ���װֵ 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = 71;                 //�趨Ԥ��Ƶ��
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //TIM���ϼ���ģʽ
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;                //����ʱ�ӷָ�
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);              //��ʼ����ʱ��
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              //ѡ��PWM2ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse = 0;                             //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;       //�����������
	
	TIM_OC1Init(TIM1,&TIM_OCInitStruct);                         //��ʼ������Ƚϲ���
	TIM_OC2Init(TIM1,&TIM_OCInitStruct);//ͨ��2
	TIM_OC3Init(TIM1,&TIM_OCInitStruct);//ͨ��3
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable); //CH1ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	 
	TIM_CtrlPWMOutputs(TIM1,ENABLE);                 //�߼���ʱ����������������
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);              //ʹ��TIM1��ARR�ϵ�Ԥװ�ؼĴ���
	
	TIM_Cmd(TIM1,ENABLE);                            //ʹ�ܶ�ʱ��1
}









