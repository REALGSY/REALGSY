#include "red.h"
float Kp = 10, Ki = 0.5, Kd = 0;                    //pid����������� 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//pidֱ������ 
float decide = 0;                                     //Ԫ���ж�
int sensor[5] = {0, 0, 0, 0, 0};                      //5����������ֵ������  
float previous_error = 0, previous_I = 0;           //���ֵ 
int initial_motor_speed ; 
void RED  (void) //IO��ʼ��
{ 
	
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);//ʹ��PORTA,PORTEʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��GPIOE4,3

}
/**************************************************************************
�������ܣ�����ģ����
��ڲ�������
����  ֵ����
**************************************************************************/
void read_sensor_values()
{
  sensor[0] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1);
  sensor[1] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_2);
  sensor[2] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_3);
  sensor[3] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_4);
  sensor[4] = GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_5);
  
    if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) 
   {
      decide = 1;//ʮ��·�� 1 1 1 1 1   ֱ��
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) 
   {
      decide = 1;//ʮ��·�� 0 1 1 1 0   ֱ��
   }
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) 
   {
      decide = 2;//90��·�� 0 1 1 1 1    ��ת90��
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) 
   {
      decide = 2;//90��·�� 0 0 1 1 0    ��ת90�� 
   } 
   
   else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
   {
      decide = 3;//90��·�� 1 1 1 1 0    ��ת90�� 
   } 
   
   else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      decide = 3;//90��·�� 0 1 1 0 0    ��ת90�� 
   }
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      decide = 3;//������� 0 1 1 0 0    ������� 
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
   {
      error = 2;//          0 0 0 1 1
   }
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))
   {
      error = 1;//          0 0 1 1 1
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      error = 0;//          0 0 1 0 0
   }
   
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      error = -1;//         1 1 1 0 0
   } 
   
    else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      error = -2;//         1 1 0 0 0
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      if (error == -2) 
      {//  0 0 0 0 0
        error = -3;
      }
      else
      {
        error = 3;
      }
    }
}
/**************************************************************************
�������ܣ�����PID
��ڲ�������
����  ֵ����
**************************************************************************/

void calc_pid()
{
  P = error;
  I = I + error;
  D = error - previous_error;
 
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);
 
  previous_error = error;
}


/**************************************************************************
�������ܣ������ٶȿ��ƺ���
��ڲ�������
����  ֵ����
**************************************************************************/

void decidemode (void)
{
	if(decide == 1) //ֱ��
	{
	Target_velocity1 = -800;
	Target_velocity2 = -800;
	Target_velocity3 = +800;
	Target_velocity4 = -800;
	}
	else if (decide == 2) //��ת90��
	{
	Target_velocity1 = -(800);
	Target_velocity2 = -(-800);
	Target_velocity3 = +(-800);
	Target_velocity4 = -(800);
	}
	else if (decide == 3) //��ת90��
	{
	Target_velocity1 = 800;
	Target_velocity2 = -800;
	Target_velocity3 = +800;
	Target_velocity4 = 800;
	}
}

void redcon(void)
{
	Target_velocity1 = -(initial_motor_speed + PID_value);
	Target_velocity2 = -(initial_motor_speed - PID_value);
	Target_velocity3 =  (initial_motor_speed + PID_value);
	Target_velocity4 = -(initial_motor_speed - PID_value);
}
/**************************************************************************
�������ܣ������ٶȿ��ƺ���
��ڲ�������
����  ֵ����
**************************************************************************/
void rerecon(void)
{
	read_sensor_values();
	calc_pid();
	decidemode();
	redcon();
}
