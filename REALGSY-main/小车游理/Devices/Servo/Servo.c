/**
  ******************************************************************************
  * @file    Servo.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   抓取抬升所用舵机
  ******************************************************************************
  */
#include "Servo.h"
void TIM1_PWM (void) 
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
  GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);  //使能定时器1时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); //使能GPIOE的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13;   //PA8
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;         //复用输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE,&GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_Period = 19999;                     //设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = 71;                 //设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;                //设置时钟分割
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);              //初始化定时器
	
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;              //选择PWM2模式
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  //比较输出使能
	TIM_OCInitStruct.TIM_Pulse = 0;                             //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;       //设置输出极性
	
	TIM_OC1Init(TIM1,&TIM_OCInitStruct);                         //初始化输出比较参数
	TIM_OC2Init(TIM1,&TIM_OCInitStruct);//通道2
	TIM_OC3Init(TIM1,&TIM_OCInitStruct);//通道3
	
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable); //CH1使能预装载寄存器
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	 
	TIM_CtrlPWMOutputs(TIM1,ENABLE);                 //高级定时器输出必须设置这句
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);              //使能TIM1在ARR上的预装载寄存器
	
	TIM_Cmd(TIM1,ENABLE);                            //使能定时器1
}









