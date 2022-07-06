/**
  ******************************************************************************
  * @file    led.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   LED灯
  ******************************************************************************
  */
#include "led.h"
 /**************************************************************************
函数功能：LED初始化
入口参数：无
返回  值：无
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能时钟 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  GPIO_SetBits(GPIOE,GPIO_Pin_5);                 //PA12 输出高电平
}
/**************************************************************************
函数功能：LED闪烁
入口参数：闪烁频率 
返回  值：无
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
		if(++temp==time)	LED=~LED,temp=0; //低电平点亮
}
