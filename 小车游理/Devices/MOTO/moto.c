/**
  ******************************************************************************
  * @file    moto.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   电机初始化相关
  ******************************************************************************
  */
#include "moto.h"

/**************************************************************************
函数功能：驱动模块初始化控制电机所需的IO
入口参数：无
返回  值：无
引脚：PB14 PB15 PB12 PB13
**************************************************************************/
void MOTO_Init1(void)//初始化控制电机所需的IO
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//PB14 15(电机1) PB12 13(电机2) 推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStruct);	
}

void MOTO_Init2(void)//初始化控制电机所需的IO
{
  GPIO_InitTypeDef GPIO_InitStruct;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF,ENABLE);
	//PF1 2 3 4 推挽输出
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin =GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOF,&GPIO_InitStruct);	
}

/**************************************************************************
函数功能：初始化pwm输出引脚
入口参数：无
返回  值：无
引脚：
**************************************************************************/
void pwm_Init(void) 
{	
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	//	TIM_DeInit(TIM8);    
	/*将寄存器重新设置默认值*/

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6| GPIO_Pin_7|GPIO_Pin_8| GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	//配置时基
	TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
	TIM_TimeBaseInitStruct.TIM_Period = 7199;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_Pulse = 0;
	//初始化CCR（设置占空比）
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	//输出使能
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	//PWM工作模式
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	//输出高有效（起始波形高）

	/*初始化通道*/
	TIM_OC1Init(TIM8, &TIM_OCInitStruct);
	TIM_OC2Init(TIM8, &TIM_OCInitStruct);
	TIM_OC3Init(TIM8, &TIM_OCInitStruct);
	TIM_OC4Init(TIM8, &TIM_OCInitStruct);

	/*预装载使能*/
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);
	/*主输出使能*/
	TIM_ARRPreloadConfig(TIM8, ENABLE);
	/*使能TIMx在ARR上预装载寄存器*/
	TIM_Cmd(TIM8, ENABLE);
}
