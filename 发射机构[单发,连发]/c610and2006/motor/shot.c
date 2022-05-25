#include "shot.h"
M3508s_t M2006s[3];
M3508s_t  roll_inc;
M6020_t roll_pos;
float roll_real_Angle;
float roll_target_Angle;;
int roll_get_right;
int need_shot;  //需要发送的弹丸数量
int16_t roll_frequency;
//电机6，7为摩擦轮电机，电机8为拨轮电机2

void roll_shot_go (void) //摩擦轮函数
{
	Roll_Init();//摩擦轮初始化
	Roll_Preload();//预装载
	Roll_Mode(); //roll模式
	shot_IRQHandler();

	Motor_2_SendData(0,M2006s[0].outCurrent,M2006s[1].outCurrent,roll_inc.outCurrent);
}

void Roll_Init (void)  //摩擦轮初始化
{
   static int n = 0 ;
	if(n == 0)
   {
	roll_pos.targetAngle = 0;
	M2006s[0].targetSpeed = 0;
	M2006s[1].targetSpeed = 0;
   }
   n++;
}

void shot_IRQHandler(void)
{
	hold_control(); //遥控器显示死区
	for(int i = 0; i<3;i++)
	{
		roll_pos.realAngle = motor_chassis[7].ecd; //获取拨盘电机角度
		M2006s[0].targetSpeed = 1500; //摩擦轮速度
		M2006s[1].targetSpeed = - 1500; 
		M2006s[i].realSpeed = motor_chassis[i+5].speed_rpm;  //获取摩擦轮速度
		M2006s[i].outCurrent = Incremental_PID(&M2006s[i].pid, M2006s[i].targetSpeed, M2006s[i].realSpeed); //摩擦轮计算pid
	}
	angle_control(); //过零记圈
	roll_pos.outCurrent = Position_PID(&roll_pos.pid,roll_pos.targetAngle,roll_pos.totalAngle);  //拨轮外环（位置环）
	roll_inc.outCurrent = Incremental_PID(&roll_inc.pid, roll_pos.outCurrent, M2006s[2].realSpeed); //拨轮内环 （速度环）
}

void shot_run_Output(int16_t output_6 ,int16_t output_7 ,int16_t output_8)
{
	
}

void roll_Init (void)
{
  for(int i = 0; i<3;i++)
	{
	IncrementalPID_paraReset(&M2006s[i].pid, 2.5f, 0.8f, 0.0f, 8000, 1000);
	}
	PositionPID_paraReset(&roll_pos.pid, 0.34f,0,0, 10000, 1000); //6020
	IncrementalPID_paraReset(&roll_inc.pid, 3.4f,0,0, 10000, 1000);
}

void if_or_no (void)
{
	roll_frequency ++ ;//记录发射次数
	roll_pos.targetAngle += one_part_roll ;
	roll_get_right = 0 ;
	
	//Motor_2_SendData(0,M2006s[0].outCurrent,M2006s[1].outCurrent,roll_inc.outCurrent);
}

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

void roll_death_mode (void)
{
	if (roll_pos.realAngle - roll_pos.lastAngle<4500)
	{
		roll_pos.targetAngle -= one_part_roll ;
		roll_frequency --;
	}
	
}
 
void Roll_Preload (void)  //预装载
{
	if(rc.roll > 450 && rc.roll < 600 )
	{
		roll_get_right = 35;
	}
}

void Roll_Mode (void)   //roll模式
{
	if(rc.roll == 660)
	{
		roll_get_right ++;
		if(roll_get_right > 40)
		{
			roll_get_right = 40;
		}
	}
	
	if (roll_get_right == 40)
	{
		if_or_no();
	}
}
