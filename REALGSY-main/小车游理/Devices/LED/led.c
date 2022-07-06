/**
  ******************************************************************************
  * @file    led.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   LED��
  ******************************************************************************
  */
#include "led.h"
 /**************************************************************************
�������ܣ�LED��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void LED_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //ʹ��ʱ�� 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
  GPIO_SetBits(GPIOE,GPIO_Pin_5);                 //PA12 ����ߵ�ƽ
}
/**************************************************************************
�������ܣ�LED��˸
��ڲ�������˸Ƶ�� 
����  ֵ����
**************************************************************************/
void Led_Flash(u16 time)
{
	  static int temp;
		if(++temp==time)	LED=~LED,temp=0; //�͵�ƽ����
}
