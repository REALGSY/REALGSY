#include "sys.h"
float eKp =1150, eKd = 75;                    //pid����������� 
float eerror = 0, eP = 0, eI = 0, eD = 0, ePID_value = 0;//pidֱ������                                 //Ԫ���ж�
int esensor[9] = {0, 0, 0, 0, 0};                      //5����������ֵ������  
float eprevious_error = 0;           //���ֵ 
int einitial_motor_speed = 600; 
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
�������ܣ�����PID
��ڲ�������
����  ֵ����
**************************************************************************/

void ecalc_pid()
{
  eP = eerror;
  eD = eerror - eprevious_error;
  ePID_value = (eKp * eP) + (eKd * eD);
	
	
  eprevious_error = eerror;
}

void econconcc ()
{
		if(eerror>300)
			eerror=300;

}
	
	
	
/**************************************************************************
�������ܣ�����ģ����
��ڲ�������
����  ֵ����
**************************************************************************/
void eread_sensor_values(void)
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
		
		
	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = 0;      //   00  0  111  0  00
   }
	 else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 0) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.3;    //   00  0  111  1  00
   }
	 else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 0))
   {
      eerror = - 0.3;    //   00  1  111  0  00
   }
	 else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 1)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.5;    //   00  0  011  1  00
   }
	 else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 1)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.5;    //   00  1  110  0  00
   }
	 	else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 0) && (esensor[3] == 0) && (esensor[4] == 0)&& (esensor[5] == 1)&& (esensor[6] == 1))
   {
      eerror = 0.8;    //   00  0  001  1  00
   }
	 	else	 if ((esensor[0] == 0) && (esensor[1] == 0) && (esensor[2] == 1) && (esensor[3] == 1) && (esensor[4] == 0)&& (esensor[5] == 0)&& (esensor[6] == 0))
   {
      eerror = - 0.8;    //   00  1  100  0  00
   }

	 
	 
	ecalc_pid();
	econconcc();
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





 




