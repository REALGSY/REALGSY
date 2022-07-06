#include "motor.h"
#include "can.h"
#include "remoter.h"
#include "IMU_Compensate.h"

int Target_velocity[4]={0}; 

int moto1;
int moto2;
int moto3;
int moto4;

int Encoder1;
int Encoder2;
int Encoder3;
int Encoder4;
M6020_t M6020_t_follow;
M3508s_t M3508s[4];
M3508s_t  roll_inc;
M3508s_t M6020_i_follow;
M6020_t roll_pos;
extern rc_info_t rc;
float	start_imu;
int imu_tim = 0;
int a_start = 0;

void follow_motor(void) //底盘跟随
{ 
	hold_control();
  M6020_i_follow.realSpeed = motor_chassis[4].speed_rpm;
	change_imu_to_angele((int32_t) mpu6500_Exportdata.angle_degree[0]);
	M6020_t_follow.targetAngle = (start_imu + rc.ch4 * 0.5);
	ComputeMinOffset(&M6020_t_follow.targetAngle,& M6020_t_follow.yaw_imu_angle);
	M6020_t_follow.outCurrent = Position_PID(&M6020_t_follow.pid,M6020_t_follow.targetAngle,M6020_t_follow.yaw_imu_angle);  //底盘跟随（位置环）
	M6020_i_follow.outCurrent = Incremental_PID(&M6020_i_follow.pid, M6020_t_follow.outCurrent, M6020_i_follow.realSpeed); 
	Motor_2_SendData(M6020_i_follow.outCurrent,0,0,0);
}

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

void moter_M6020(void)  //头部
{
	hold_control(); 
	target += rc.ch4 * 0.5 ;
  roll_pos.realAngle = motor_chassis[4].ecd ;
	roll_pos.realSpeed = motor_chassis[4].speed_rpm ;
	angle_control();
	roll_pos.outCurrent = Position_PID(&roll_pos.pid,roll_pos.targetAngle,roll_pos.totalAngle);  //拨轮外环（位置环）
	roll_inc.outCurrent = Incremental_PID(&roll_inc.pid, roll_pos.outCurrent, roll_pos.realSpeed); //拨轮内环 （速度环）
}

void Chassis_Init(void)
{
	for(int i = 0; i < 4; i++)
	{
	IncrementalPID_paraReset(&M3508s[i].pid, 2.5f, 0.8f, 0.0f, 8000, 1000); //3508
	}
	PositionPID_paraReset(&roll_pos.pid, 0.34f,0,0, 10000, 1000); //6020
	IncrementalPID_paraReset(&roll_inc.pid, 3.4f,0,0, 10000, 1000);
	
	PositionPID_paraReset(&M6020_t_follow.pid, 0.34f,0,0, 10000, 1000); //yaw轴云台pid
	IncrementalPID_paraReset(&M6020_i_follow.pid, 50.0f,0.3f,0, 10000, 1000);
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

/**
 * @brief 过零处理
 * 
 * @param target 
 * @param value 
 * @return int 
 */
static int ComputeMinOffset(int16_t *target, int16_t *value) //计算最小偏差，底盘跟随应该往哪个方向去完成跟随动作。
{
    int err = target - value;

    if (err > 4096)
    {
        err -= 8191;
    }
    else if (err < -4096)
    {
        err += 8191;
    }
    return err;
}

/**
 * @brief 记圈处理
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void angle_control (void)
{
	if (roll_pos.realAngle - roll_pos.lastAngle < -4092)
    {
        roll_pos.turnCount++;
    }

    if (roll_pos.lastAngle - roll_pos.realAngle < -4092)
    {
        roll_pos.turnCount--;
    }

    roll_pos.totalAngle = roll_pos.realAngle + (8192 * roll_pos.turnCount);
	
	roll_pos.lastAngle = roll_pos.realAngle; //保存上次

			
    //帧率统计，数据更新标志位
    roll_pos.InfoUpdateFrame++;

    roll_pos.InfoUpdateFlag = 1;
}

void change_imu_to_angele (float a_imu )
{
	M6020_t_follow.yaw_imu_angle = a_imu * 22;
}

void read_start_imu (void)
{
	if (a_start == 0)
	{	
		imu_tim ++ ;
		if(imu_tim == 10)
		{
			start_imu = (int32_t) mpu6500_Exportdata.angle_degree[0] * 22.75;
			a_start = 1 ;
			imu_tim = 0;
		}
	}
}

