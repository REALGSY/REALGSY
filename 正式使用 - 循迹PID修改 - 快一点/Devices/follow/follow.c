#include "red.h"
#include "sys.h"
float fKp =1150, fKd = 75;                    //pid����������� 
float foerror = 0, fP = 0, fI = 0, fD = 0, fPID_value = 0;//pidֱ������ 
float fdecide = 0;                                     //Ԫ���ж�
int fsensor[9] = {0, 0, 0, 0, 0};                      //5����������ֵ������  
float fprevious_error = 0, fprevious_I = 0;           //���ֵ 
int finitial_motor_speed = 600; 
int fo;
void fRED  (void) //IO��ʼ��
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

void fcalc_pid()
{
  fP = foerror;
  fD = foerror - fprevious_error;
  fPID_value = (fKp * fP) + (fKd * fD);
	
	
  fprevious_error = foerror;
}

void fconconcc ()
{
		if(foerror>300)
			foerror=300;

}
	
	
	
/**************************************************************************
�������ܣ�����ģ����
��ڲ�������
����  ֵ����
**************************************************************************/
void fread_sensor_values(void)
{

		fsensor[0] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1);
		fsensor[1] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_2);
		fsensor[2] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_3);
		fsensor[3] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_4);
		fsensor[4] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_5);
		fsensor[5] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_6);
		fsensor[6] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7);
		fsensor[7] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8);
		fsensor[8] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9);
    
		//01  2  345  6   78
		
		
	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 0) && (fsensor[3] == 1) && (fsensor[4] == 1)&& (fsensor[5] == 1)&& (fsensor[6] == 0)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = 0;      //   00  0  111  0  00
   }
	 else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 0) && (fsensor[3] == 1) && (fsensor[4] == 1)&& (fsensor[5] == 1)&& (fsensor[6] == 1)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = 0.3;    //   00  0  111  1  00
   }
	 else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 1) && (fsensor[3] == 1) && (fsensor[4] == 1)&& (fsensor[5] == 1)&& (fsensor[6] == 0)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = - 0.3;    //   00  1  111  0  00
   }
	 else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 0) && (fsensor[3] == 0) && (fsensor[4] == 1)&& (fsensor[5] == 1)&& (fsensor[6] == 1)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = 0.5;    //   00  0  011  1  00
   }
	 else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 1) && (fsensor[3] == 1) && (fsensor[4] == 1)&& (fsensor[5] == 0)&& (fsensor[6] == 0)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = - 0.5;    //   00  1  110  0  00
   }
	 	else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 0) && (fsensor[3] == 0) && (fsensor[4] == 0)&& (fsensor[5] == 1)&& (fsensor[6] == 1)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = 0.8;    //   00  0  001  1  00
   }
	 	else	 if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[2] == 1) && (fsensor[3] == 1) && (fsensor[4] == 0)&& (fsensor[5] == 0)&& (fsensor[6] == 0)&& (fsensor[7] == 0)&& (fsensor[8] == 0))
   {
      foerror = - 0.8;    //   00  1  100  0  00
   }
	 else if ((fsensor[0] == 0) && (fsensor[1] == 1) && (fsensor[7] == 0)&& (fsensor[8] == 0))
	{
		if(fo == 0)
		{
			foerror = 1.4;
			fo++;
		}
		else if (fo == 1)
		{
			foerror = -1.4;
			fo++;
		}
			else 
		{
			foerror = -1.4;
		}
	}
		else if ((fsensor[0] == 1) && (fsensor[1] == 1) && (fsensor[7] == 0)&& (fsensor[8] == 0))
	{
			foerror = -2.5 ;
	}
				else if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[7] == 1)&& (fsensor[8] == 0))
	{
			foerror = 1.5;
	}
	 
			else if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[7] == 1)&& (fsensor[8] == 1))
	{
			foerror = 2.5;
	}
	 		else if ((fsensor[0] == 0) && (fsensor[1] == 0) && (fsensor[7] == 0)&& (fsensor[8] == 0))
	{
			foerror = 0 ;
	}
	 
	 
	 
	fcalc_pid();
	fconconcc();
  PS2_KEY=PS2_DataKey();	
	 if(PS2_KEY == 9)
	 {		
			__set_FAULTMASK(1);   //STM32���������λ  
			NVIC_SystemReset();  
	}

	 Target_velocity1 = -(finitial_motor_speed + fPID_value);
	 Target_velocity2 = -(finitial_motor_speed - fPID_value);
	 Target_velocity3 =  (finitial_motor_speed + fPID_value);
	 Target_velocity4 = -(finitial_motor_speed - fPID_value);

 }





 




