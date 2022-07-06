#include "red.h"
float Kp = 10, Ki = 0.5, Kd = 0;                    //pid弯道参数参数 
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;//pid直道参数 
float decide = 0;                                     //元素判断
int sensor[5] = {0, 0, 0, 0, 0};                      //5个传感器数值的数组  
float previous_error = 0, previous_I = 0;           //误差值 
int initial_motor_speed ; 
void RED  (void) //IO初始化
{ 
	
 	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG,ENABLE);//使能PORTA,PORTE时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_5|GPIO_Pin_6;//KEY0-KEY1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
 	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化GPIOE4,3

}
/**************************************************************************
函数功能：红外模块检测
入口参数：无
返回  值：无
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
      decide = 1;//十字路口 1 1 1 1 1   直行
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) 
   {
      decide = 1;//十字路口 0 1 1 1 0   直行
   }
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) 
   {
      decide = 2;//90度路口 0 1 1 1 1    右转90度
   } 
   
    else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0)) 
   {
      decide = 2;//90度路口 0 0 1 1 0    右转90度 
   } 
   
   else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
   {
      decide = 3;//90度路口 1 1 1 1 0    左转90度 
   } 
   
   else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      decide = 3;//90度路口 0 1 1 0 0    左转90度 
   }
   
    else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
   {
      decide = 3;//向上锐角 0 1 1 0 0    向上锐角 
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
函数功能：红外PID
入口参数：无
返回  值：无
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
函数功能：红外速度控制函数
入口参数：无
返回  值：无
**************************************************************************/

void decidemode (void)
{
	if(decide == 1) //直走
	{
	Target_velocity1 = -800;
	Target_velocity2 = -800;
	Target_velocity3 = +800;
	Target_velocity4 = -800;
	}
	else if (decide == 2) //右转90度
	{
	Target_velocity1 = -(800);
	Target_velocity2 = -(-800);
	Target_velocity3 = +(-800);
	Target_velocity4 = -(800);
	}
	else if (decide == 3) //左转90度
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
函数功能：红外速度控制函数
入口参数：无
返回  值：无
**************************************************************************/
void rerecon(void)
{
	read_sensor_values();
	calc_pid();
	decidemode();
	redcon();
}
