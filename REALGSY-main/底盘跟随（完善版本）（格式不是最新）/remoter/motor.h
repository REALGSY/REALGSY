#ifndef __MOTOR_H
#define __MOTOR_H
#include "main.h"
#include "PID.h"

typedef struct{
	uint16_t realAngle;			    //读回来的机械角度
	int16_t  realSpeed;			    //读回来的速度
	int16_t  realCurrent;		    //读回来的实际电流
	uint8_t  temperture;        //读回来的电机温度
	int16_t  targetCurrent;			//目标电流
	int16_t  targetSpeed;			  //目标速度
	uint16_t targetAngle;			  //目标角度
	int16_t  outCurrent;				//输出电流
	incrementalpid_t pid;		    //电机pid
	uint8_t  M3508InfoUpdateFlag;		  //信息读取更新标志
	uint16_t M3508InfoUpdateFrame;	  //帧率
	uint8_t  M3508OffLineFlag;		    //设备离线标志
}M3508s_t;

typedef struct{
	uint16_t realAngle;			    //读回来的机械角度
	int16_t  realSpeed;			    //读回来的速度
	int16_t  realCurrent;		    //读回来的实际电流
	uint8_t  temperture;        //读回来的电机温度
	uint16_t lastAngle;  //上次位置
	int16_t  turnCount;   //计数
	int32_t  totalAngle;  //总共角度
	int16_t  InfoUpdateFrame;
	int16_t  InfoUpdateFlag;
	int16_t  targetCurrent;		//目标电流
	int16_t  targetSpeed;			  //目标速度
	int32_t  targetAngle;			  //目标角度
	int16_t  outCurrent;				//输出电流
	positionpid_t pid;		    //电机pid
	uint8_t  M6020InfoUpdateFlag;		  //信息读取更新标志
	uint16_t M6020InfoUpdateFrame;	  //帧率
	uint8_t  M6020OffLineFlag;		    //设备离线标志
	int16_t yaw_imu_angle; //真实角度换算成角度
}M6020_t;

typedef struct{
	uint16_t realAngle;			    //读回来的机械角度
	int16_t  realSpeed;			    //读回来的速度
	int16_t  realCurrent;		    //读回来的实际电流
	uint8_t  temperture;        //读回来的电机温度
	int16_t  targetCurrent;			//目标电流
	int16_t  targetSpeed;			  //目标速度
	uint16_t targetAngle;			  //目标角度
	int16_t  outCurrent;				//输出电流
	GM6020_incrementalpid_t pid;		    //电机pid
	uint8_t  M6020InfoUpdateFlag;		  //信息读取更新标志
	uint16_t M6020InfoUpdateFrame;	  //帧率
	uint8_t  M6020OffLineFlag;		    //设备离线标志
}M6020TWO_t;

void moter_M6020(void);
void CAN_Control(int Y_Move,int X_Move,int Yaw);
void Chassis_Init(void);
void moter_TIM2_IRQHandler(void);
void Handle_Angle8191_PID_Over_Zero(int16_t *tar, int16_t *cur);
void angle_control (void);
void read_start_imu (void);
void follow_motor(void) ;
void change_imu_to_angele (float a_imu);
static int ComputeMinOffset(int16_t target, int16_t value);
void rc_dead(int16_t *rc_dead);
void follow_motor_under(void); //底盘跟随(轮子)
void angle_imu (void);
#endif
