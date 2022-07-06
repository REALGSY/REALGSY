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
M6020_t M6020_t_follow; //跟随云台yaw轴（位置）
M3508s_t M6020_i_follow;//跟随云台yaw轴（增量）

M3508s_t  roll_inc; //无云台yaw轴（位置）
M6020_t roll_pos;   //无云台yaw轴（增量）

M3508s_t M3508s[4]; //底盘控制（无跟随）

M6020_t  M3508_P_follow; //底盘控制跟随(跟随位置)
M3508s_t M3508_I_follow[4]; //底盘控制跟随(跟随增量)

void rc_dead(int16_t *rc_dead);
	
#define motor_x 123; //底盘机械x轴
#define motor_y 123; //底盘机械y轴
#define motor_z 123; //底盘机械z轴

extern rc_info_t rc;
float	start_imu;
int imu_tim = 0;
int a_start = 0;
int16_t M3508_P_follow_targetAngle = 4024;
int16_t M3508_P_follow_realAngle = 0;
int32_t tar_tar = 0;
int16_t rc_dead_0;
int16_t rc_dead_2;
int16_t rc_dead_3;
int32_t M6020_t_follow_yaw_imu_angle_lastAngle;


void follow_motor(void) //底盘跟随 (头部)
{ 
	rc_dead_0 = - rc.ch0; //获取手柄数值
	rc_dead(&rc_dead_0);  //手柄死区
	tar_tar += rc_dead_0 * 0.2;  //存储手柄数值并积累
  M6020_i_follow.realSpeed = motor_chassis[4].speed_rpm;   //获得6020的速度
	change_imu_to_angele((int32_t) mpu6500_Exportdata.angle_degree[0]);  //转换陀螺仪实际角度制到机械角度制
	M6020_t_follow.targetAngle = (start_imu + tar_tar);  //设置目标速度
	//Handle_Angle8191_PID_Over_Zero(&M6020_t_follow.targetAngle,&M6020_t_follow.yaw_imu_angle);  (过零（未使用）)
	angle_imu(); //记圈处理
	M6020_t_follow.outCurrent = Position_PID(&M6020_t_follow.pid,M6020_t_follow.targetAngle,M6020_t_follow.totalAngle);  //底盘跟随（位置环）
	M6020_i_follow.outCurrent = Incremental_PID(&M6020_i_follow.pid, M6020_t_follow.outCurrent, M6020_i_follow.realSpeed);  //底盘跟随 (速度环)
	Motor_2_SendData(M6020_i_follow.outCurrent,0,0,0); //发送电机电流数据
}


void follow_motor_under(void) //底盘跟随(轮子)
{
	rc_dead_2 = rc.ch2;
	rc_dead_3 = -rc.ch3;
	M3508_P_follow_realAngle = motor_chassis[4].ecd;
	Handle_Angle8191_PID_Over_Zero(&M3508_P_follow_targetAngle,&M3508_P_follow_realAngle);
	M3508_P_follow.outCurrent = Position_PID(&M3508_P_follow.pid,M3508_P_follow_targetAngle,M3508_P_follow_realAngle); 
	rc_dead(&rc_dead_2);
	rc_dead(&rc_dead_3);
		for(int i = 0; i<4; i++)
	{
		CAN_Control(rc_dead_3,rc_dead_2,M3508_P_follow.outCurrent);
		M3508_I_follow[i].realSpeed = motor_chassis[i].speed_rpm;
		M3508_I_follow[i].outCurrent = Incremental_PID(&M3508_I_follow[i].pid, Target_velocity[i], M3508_I_follow[i].realSpeed); 
	}
	Motor_SendData(M3508_I_follow[0].outCurrent,M3508_I_follow[1].outCurrent,M3508_I_follow[2].outCurrent,M3508_I_follow[3].outCurrent);
}

void moter_TIM2_IRQHandler(void) //底盘(无跟随)
{
	for(int i = 0; i<4; i++)
	{
		CAN_Control(-rc.ch3,rc.ch2,rc.ch0);
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

void moter_M6020(void)  //头部（无跟随）  //yes
{
	target += rc.ch3 * 0.5 ;
  roll_pos.realAngle = motor_chassis[4].ecd ;
	roll_pos.realSpeed = motor_chassis[4].speed_rpm ;
	angle_control();
	roll_pos.outCurrent = Position_PID(&roll_pos.pid,roll_pos.targetAngle,roll_pos.totalAngle);  //头部外环（位置环）
	roll_inc.outCurrent = Incremental_PID(&roll_inc.pid, roll_pos.outCurrent, roll_pos.realSpeed); //头部内环 （速度环）
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
	IncrementalPID_paraReset(&M6020_i_follow.pid, 60.0f,0.3f,0, 10000, 1000);

	PositionPID_paraReset(&M3508_P_follow.pid, 0.34f,0,0, 10000, 1000); //底盘跟随位置pid
    for(int i = 0; i < 4; i++)
	{
	IncrementalPID_paraReset(&M3508_I_follow[i].pid, 2.5f, 0.8f,0, 10000, 1000);//底盘跟随增量pid
	}
	
}

/**
  * @brief  
  * @param[in]   
  * @retval None
  */
	
/**
  * @brief  运动解算
  * @param[in]   
  * @retval None
  */
void CAN_Control(int X_Move,int Y_Move,int Yaw)
{
	Target_velocity[0] = 	( X_Move + Y_Move + Yaw)*12;
	Target_velocity[1] =  (-X_Move + Y_Move + Yaw)*12;
	Target_velocity[2] = 	(-X_Move - Y_Move + Yaw)*12;
	Target_velocity[3]=  	( X_Move - Y_Move + Yaw)*12;	
};


/**
 * @brief 过零处理
 * 
 * @param target 
 * @param value 
 * @return int 
 */
void Handle_Angle8191_PID_Over_Zero(int16_t *tar, int16_t *cur)
{
	if(*tar - *cur > 4096)    
	{
		*tar -= 8192;        
	}
	else if(*tar - *cur < -4096)
	{
		*tar = *tar + 8192;
	}
	else
	{
		//*cur = *cur;
		// do nothing
	}
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

void change_imu_to_angele (float a_imu )   //转化为机械角度制
{
	M6020_t_follow.yaw_imu_angle = a_imu * 22.7527f;
}

void read_start_imu (void)  //获取开机后的陀螺仪位置，并设为基准
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

void rc_dead(int16_t *rc_dead)
{
	if(*rc_dead >= 0 && *rc_dead <= 60)
	{
		*rc_dead = 0;
	}
	else if(*rc_dead <= 0 && *rc_dead >= -60)
	{
		*rc_dead = 0;
	}
}


void angle_imu (void)
{
	if (M6020_t_follow.yaw_imu_angle - M6020_t_follow_yaw_imu_angle_lastAngle < -6500)
    {
        M6020_t_follow.turnCount++;
    }

    if (M6020_t_follow_yaw_imu_angle_lastAngle - M6020_t_follow.yaw_imu_angle < -6500)
    {
        M6020_t_follow.turnCount--;
    }

    M6020_t_follow.totalAngle = M6020_t_follow.yaw_imu_angle + (8192 * M6020_t_follow.turnCount);
	
	  M6020_t_follow_yaw_imu_angle_lastAngle = M6020_t_follow.yaw_imu_angle; //保存上次

			
    //帧率统计，数据更新标志位
    M6020_t_follow.InfoUpdateFrame++;

    M6020_t_follow.InfoUpdateFlag = 1;
}
