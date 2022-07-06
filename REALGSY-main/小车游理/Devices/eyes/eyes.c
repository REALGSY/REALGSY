/**
  ******************************************************************************
  * @file    eyes.c
  * @author  GSY
  * @version V1.0
  * @date    
  * @brief   �Ӿ���������
  ******************************************************************************
  */
#include "sys.h"
#include "eyes.h"
#include "usart.h"
#include "sys.h"
#include "usart.h"
#include <string.h>
float eKp =1150, eKd = 75;                    //pid����������� 
float eerror = 0, eP = 0, eI = 0, eD = 0, ePID_value = 0;//pidֱ������                                 //Ԫ���ж�
int esensor[9] = {0, 0, 0, 0, 0};                      //5����������ֵ������  
float eprevious_error = 0;           //���ֵ 
int einitial_motor_speed = 800; 
int i;
u8 resdata[LEN];


void eRED  (void) //IO��ʼ��
{ 
	
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);//ʹ��PORTA,PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOE4,3

}
/**************************************************************************
�������ܣ��Ӿ�Ѱ��PID
��ڲ�������
����  ֵ����
**************************************************************************/


void econconcc ()
{
		if(eerror>300)
			eerror=300;

}
	
void stop()
{
	 Target_velocity1 = 0;
	 Target_velocity2 = 0;
	 Target_velocity3 = 0;
	 Target_velocity4 = 0;

}
	
/**************************************************************************
�������ܣ�����ģ����
��ڲ�������
����  ֵ����
**************************************************************************/


void eread_sensor_values(u8 eyesmode)
{

		esensor[0] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1);
		esensor[1] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_2);
		esensor[2] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_3);
		esensor[3] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_4);
		esensor[4] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_5);
		esensor[5] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_6);
		esensor[6] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7);
		esensor[7] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8);
		esensor[8] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9);
    
		//01  2  345  6   78
 if(eyesmode == 1)
 {
	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = 0;      //   00  0  111  0  00
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.3;    //   00  0  111  1  00
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = - 0.3;    //   00  1  111  0  00
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.5;    //   00  0  011  1  00
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.5;    //   00  1  110  0  00
   }
	 	else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.8;    //   00  0  001  1  00
   }
	 	else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.8;    //   00  1  100  0  00
   }
	 else if ((esensor[7] == 1)&& (esensor[8] == 1))
	 {
      stopmark++;
		  if(stopmark == 9)
			{
				stopmark = 0;
				mark =4;
			}
	  }
  }
 
 if(eyesmode == 2)
 {
	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = 0;      //     0  111  0  
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.3;    //     0  111  1  
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = - 0.3;    //   1  111  0  
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.5;    //     0  011  1  
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.5;    //   1  110  0  
   }
	 	else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.8;    //     0  001  1  
   }
	 	else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.8;    //   1  100  0  
   }
	  else if   ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
	 {
		  eerror = - 1.8;
	 }
	 else if ((esensor[7] == 1)&& (esensor[8] == 1))
	 {
      stopmark++;
		  if(stopmark == 9)
			{
				stopmark = 0;
				mark =4;
			}
	  }
  }
	
 if(eyesmode == 3)
 {
	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = 0;      //   00  0  111  0  00
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.3;    //   00  0  111  1  00
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = - 0.3;    //   00  1  111  0  00
   }
	 else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.5;    //   00  0  011  1  00
   }
	 else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.5;    //   00  1  110  0  00
   }
	 	else	 if ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.8;    //   00  0  001  1  00
   }
	 	else	 if ((esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.8;    //   00  1  100  0  00
   }
	  else if   ((esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
	 {
		  eerror =  1.8;
	 }
	 else if ((esensor[7] == 1)&& (esensor[8] == 1))
	 {
      stopmark++;
		  if(stopmark == 9)
			{
				stopmark = 0;
				mark =4;
			}
	  }
  }
	 
	eP = eerror;
  eD = eerror - eprevious_error;
  ePID_value = (eKp * eP) + (eKd * eD);
	
	
  eprevious_error = eerror;	econconcc();
	 
  PS2_KEY=PS2_DataKey();	
	 if(PS2_KEY == 9)
	 {		
			__set_FAULTMASK(1);   //STM32���������λ  
			NVIC_SystemReset();  
	}

	 Target_velocity1 = -(einitial_motor_speed + ePID_value);
	 Target_velocity2 = -(einitial_motor_speed - ePID_value);
	 Target_velocity3 =  (einitial_motor_speed + ePID_value);
	 Target_velocity4 = -(einitial_motor_speed - ePID_value);

 }





 




