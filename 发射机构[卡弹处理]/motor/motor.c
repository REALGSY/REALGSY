#include "motor.h"
#include "can.h"
#include "remoter.h"


int Target_velocity[4]={0}; 

int moto1;
int moto2;
int moto3;
int moto4;

int Encoder1;
int Encoder2;
int Encoder3;
int Encoder4;
M3508s_t M3508s[4];
M3508s_t GM6020inc;
M6020_t  GM6020;
extern rc_info_t rc;


void moter_TIM2_IRQHandler(void) //底盘
{
	for(int i = 0; i<4; i++)
	{
		hold_control();
		M3508s[i].targetSpeed = Target_velocity[i];
		M3508s[i].realSpeed = motor_chassis[i].speed_rpm;
		//PID计算
		M3508s[i].outCurrent = Incremental_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
		//M3508s[i].outCurrent = Position_PID(&M3508s[i].pid, M3508s[i].targetSpeed, M3508s[i].realSpeed); 
		//清标志位
		
	}
	Motor_SendData(M3508s[0].outCurrent,M3508s[1].outCurrent,M3508s[2].outCurrent,M3508s[3].outCurrent);
}

float target = 0 ;
float realecd ;
void moter_M6020(void)
{
	hold_control(); 
	target += rc.roll * 5 ;
	GM6020inc.realSpeed = motor_chassis[4].speed_rpm;
	realecd = motor_chassis[4].ecd;
	Handle_Angle8191_PID_Over_Zero(&target,&realecd);
	GM6020.outCurrent = Position_PID(&GM6020.pid,target,realecd); 
	GM6020inc.outCurrent  =  Incremental_PID(&GM6020inc.pid,GM6020.outCurrent,GM6020inc.realSpeed); 
  Motor_2_SendData(GM6020inc.outCurrent,0,0,0);
}

void Chassis_Init(void)
{
	for(int i = 0; i < 4; i++)
	{
	IncrementalPID_paraReset(&M3508s[i].pid, 2.5f, 0.8f, 0.0f, 8000, 1000); //3508
	}
	PositionPID_paraReset(&GM6020.pid, 25,0,0, 20000, 1000); //6020
	IncrementalPID_paraReset(&GM6020inc.pid, 0.4f,0,0, 20000, 1000);

}

/**
  * @brief  
  * @param[in]   
  * @retval None
  */
void DJI_GO (void)
{
	CAN_Control(rc.ch4,rc.ch3,rc.ch1);
}
	
/**
  * @brief  运动解算
  * @param[in]   
  * @retval None
  */
void CAN_Control(int X_Move,int Y_Move,int Yaw)
{
	Target_velocity[0] = 	-(X_Move - Y_Move + Yaw)*7.3;
	Target_velocity[1] =   -(-X_Move + Y_Move + Yaw)*7.3;
	Target_velocity[2] = 	 (X_Move + Y_Move + Yaw)*7.3;
	Target_velocity[3]=  	(-X_Move - Y_Move + Yaw)*7.3;	
};

void hold_control (void)
{
		for(int i = 0; i < 4; i++)
	{
	  if(Target_velocity[i]<100&&Target_velocity[i]>-100)
		{
			Target_velocity[i] =  0;
		}
	}
}

/* 角度Pid时，在更新tar和cur之后紧接着调用, 处理完再进行PID计算*/
void Handle_Angle8191_PID_Over_Zero(float *tar, float *cur)
{
	if(*tar - *cur > 4096)    //4096 ：半圈机械角度
	{
		*cur += 8192;        //8191,8192无所谓了，四舍五入
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
	else
	{
		//*cur = *cur;
		// do nothing
	}
}
