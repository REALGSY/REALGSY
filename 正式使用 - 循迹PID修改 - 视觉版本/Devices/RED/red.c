#include "red.h"
float Kp =1050, Kd = 75;                    //pid弯道参数参数 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//pid直道参数 
float decide = 0;                                     //元素判断
int sensor[9] = {0, 0, 0, 0, 0};                      //5个传感器数值的数组  
float previous_error = 0, previous_I = 0;           //误差值 
int initial_motor_speed = 1100; 
void RED  (void) //IO初始化
{ 
	
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);//使能PORTA,PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOE4,3

}
/**************************************************************************
函数功能：红外PID
入口参数：无
返回  值：无
**************************************************************************/

void calc_pid()
{
  P = error;
  D = error - previous_error;
  PID_value = (Kp * P) + (Kd * D);
	
	
  previous_error = error;
}

void conconcc ()
{
		if(error>300)
			error=300;

}
	
	
	
/**************************************************************************
函数功能：红外模块检测
入口参数：无
返回  值：无
**************************************************************************/
void read_sensor_values(void)
{

		sensor[0] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1);
		sensor[1] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_2);
		sensor[2] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_3);
		sensor[3] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_4);
		sensor[4] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_5);
		sensor[5] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_6);
		sensor[6] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7);
		sensor[7] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8);
		sensor[8] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9);
    
		//01  2  345  6   78
		
		
	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)&& (sensor[5] == 1)&& (sensor[6] == 0)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = 0;      //   00  0  111  0  00
   }
	 else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1)&& (sensor[5] == 1)&& (sensor[6] == 1)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = 0.3;    //   00  0  111  1  00
   }
	 else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)&& (sensor[5] == 1)&& (sensor[6] == 0)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = - 0.3;    //   00  1  111  0  00
   }
	 else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1)&& (sensor[5] == 1)&& (sensor[6] == 1)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = 0.5;    //   00  0  011  1  00
   }
	 else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)&& (sensor[5] == 0)&& (sensor[6] == 0)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = - 0.5;    //   00  1  110  0  00
   }
	 	else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0)&& (sensor[5] == 1)&& (sensor[6] == 1)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = 0.8;    //   00  0  001  1  00
   }
	 	else	 if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)&& (sensor[5] == 0)&& (sensor[6] == 0)&& (sensor[7] == 0)&& (sensor[8] == 0))
   {
      error = - 0.8;    //   00  1  100  0  00
   }
	 else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[7] == 0)&& (sensor[8] == 0))
	{
			error = -1.4;
	}
		else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[7] == 0)&& (sensor[8] == 0))
	{
			error = -2.5 ;
	}
				else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[7] == 1)&& (sensor[8] == 0))
	{
			error = 1.5;
	}
	 
			else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[7] == 1)&& (sensor[8] == 1))
	{
			error = 2.5;
	}
	 		else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[7] == 0)&& (sensor[8] == 0))
	{
			error = 0 ;
	}
	 
	 
	 
	calc_pid();
	conconcc();
  PS2_KEY=PS2_DataKey();	
	 if(PS2_KEY == 9)
	 {		
			__set_FAULTMASK(1);   //STM32程序软件复位  
			NVIC_SystemReset();  
	}

	 Target_velocity1 = -(initial_motor_speed + PID_value);
	 Target_velocity2 = -(initial_motor_speed - PID_value);
	 Target_velocity3 =  (initial_motor_speed + PID_value);
	 Target_velocity4 = -(initial_motor_speed - PID_value);

 }





 




