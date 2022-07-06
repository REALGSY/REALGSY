#include "control.h"
#include "red.h"
#include "eyes.h"
#include "follow.h"
int Target_velocity1=0; 
int Target_velocity2=0;
int Target_velocity3=0;
int Target_velocity4=0; 

int eyesgo;
int redmode;
int runrun;
u8 PS2_KEY;

int moto1;
int moto2;
int moto3;
int moto4;

int Encoder1;
int Encoder2;
int Encoder3;
int Encoder4;

float Kp1=8,Ki1=0.9;
float Kp2=8,Ki2=0.9;
float Kp3=8,Ki3=0.9;
float Kp4=8,Ki4=0.9;
/**************************************************************************
函数功能：定时器7中断函数
入口参数：无
返回  值：无
**************************************************************************/

int TIM7_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM7,TIM_FLAG_Update)==SET)
 {
   TIM_ClearITPendingBit(TIM7,TIM_IT_Update);   //===清除定时器7中断标志位
	 Encoder1=Read_Encoder(2);                     //取定时器2计数器的值
	 Encoder2=Read_Encoder(3);
	 Encoder3=Read_Encoder(4);
	 Encoder4=Read_Encoder(5);
	 
   Led_Flash(100);                              //LED闪烁
	 
	 moto1=Incremental_PI1(Encoder1,Target_velocity1);    //===位置PID控制器
	 moto2=Incremental_PI2(Encoder2,Target_velocity2);
	 moto3=Incremental_PI3(Encoder3,Target_velocity3);
	 moto4=Incremental_PI4(Encoder4,Target_velocity4);
	 
	 Xianfu_Pwm1();
	 Xianfu_Pwm2();
	 Xianfu_Pwm3();
	 Xianfu_Pwm4();
	 
   Set_Pwm_1(moto1);
	 Set_Pwm_2(moto2);
	 Set_Pwm_3(moto3);
	 Set_Pwm_4(moto4);
	 
 }
 return 0;
}

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm_1(int Moto1)//赋值给PWM寄存器
{
 if(Moto1>0) AIN1=1,   AIN2=0;
	else       AIN1=0,   AIN2=1;
	PWMA=myabs(Moto1);
}
void Set_Pwm_2(int Moto2)//赋值给PWM寄存器
{
 if(Moto2>0) BIN1=1,   BIN2=0;
	else       BIN1=0,   BIN2=1;
	PWMB=myabs(Moto2);
}
void Set_Pwm_3(int Moto3)//赋值给PWM寄存器
{
 if(Moto3>0) AIN21=1,   AIN22=0;
	else       AIN21=0,   AIN22=1;
	PWMA2=myabs(Moto3);
}
void Set_Pwm_4(int Moto4)//赋值给PWM寄存器
{
 if(Moto4>0) BIN21=1,   BIN22=0;
	else       BIN21=0,   BIN22=1;
	PWMB2=myabs(Moto4);
}
/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
 void Xianfu_Pwm1(void) //限制（第一个）幅度的函数
 {
   int Amplitude=7100;  //===PWM满幅是7200 限制在7100
	 if(moto1<-Amplitude)  moto1 = -Amplitude;
	 if(moto1>Amplitude)   moto1 =  Amplitude;
 }
  void Xianfu_Pwm2(void) //限制（第二个）幅度的函数
 {
   int Amplitude=7100;  //===PWM满幅是7200 限制在7100
	 if(moto2<-Amplitude)  moto2 = -Amplitude;
	 if(moto2>Amplitude)   moto2 =  Amplitude;
 }
  void Xianfu_Pwm3(void) //限制（第三个）幅度的函数
 {
   int Amplitude=7100;  //===PWM满幅是7200 限制在7100
	 if(moto3<-Amplitude)  moto3 = -Amplitude;
	 if(moto3>Amplitude)   moto3 =  Amplitude;
 }
  void Xianfu_Pwm4(void) //限制（第四个）幅度的函数
 {
   int Amplitude=7100;  //===PWM满幅是7200 限制在7100
	 if(moto4<-Amplitude)  moto4 = -Amplitude;
	 if(moto4>Amplitude)   moto4 =  Amplitude;
 }
/**************************************************************************
函数功能：绝对值函数
入口参数：int a
返回  值：unsigned int
**************************************************************************/
int myabs(int a) //取绝对值
{ 		   
	 int temp;
	 if(a<0)  temp=-a;  
	 else temp=a;
	 return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI1 (int Encoder11,int Target1)
{ 	
	 static int Bias1,Pwm1,Last_bias1;
	 Bias1=Encoder11-Target1;                //计算偏差
	 Pwm1+=Kp1*(Bias1-Last_bias1)+Ki1*Bias1;   //增量式PI控制器
	 Last_bias1=Bias1;	                   //保存上一次偏差 
	 return Pwm1;                         //增量输出
}
int Incremental_PI2 (int Encoder22,int Target2)
{ 	
	 static int Bias2,Pwm2,Last_bias2;
	 Bias2=Encoder22-Target2;                //计算偏差
	 Pwm2+=Kp2*(Bias2-Last_bias2)+Ki2*Bias2;   //增量式PI控制器
	 Last_bias2=Bias2;	                   //保存上一次偏差 
	 return Pwm2;                         //增量输出
}

int Incremental_PI3 (int Encoder33,int Target3)
{ 	
	 static int Bias3,Pwm3,Last_bias3;
	 Bias3=Encoder33-Target3;                //计算偏差
	 Pwm3+=Kp3*(Bias3-Last_bias3)+Ki3*Bias3;   //增量式PI控制器
	 Last_bias3=Bias3;	                   //保存上一次偏差 
	 return Pwm3;                         //增量输出
}

int Incremental_PI4 (int Encoder44,int Target4)
{ 	
	 static int Bias4,Pwm4,Last_bias4;
	 Bias4=Encoder44-Target4;                //计算偏差
	 Pwm4+=Kp4*(Bias4-Last_bias4)+Ki4*Bias4;   //增量式PI控制器
	 Last_bias4=Bias4;	                   //保存上一次偏差 
	 return Pwm4;                         //增量输出
}
/**************************************************************************
函数功能：main初始化
入口参数：无
返回  值：无
**************************************************************************/
void mainreturn (void)
{
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 SystemInit();             //配置系统时钟
   delay_init();            
	 uart_init(115200);        //设置波特率为115200
	 MOTO_Init1();              //初始化控制电机所需的IO
	 MOTO_Init2();
	 pwm_Init();//初始化pwm输出
	 Encoder_Init_TIM2();      //初始化定时器（计数器）
	 Encoder_Init_TIM3();
	 Encoder_Init_TIM4();
	 Encoder_Init_TIM5();
	 TIM1_PWM();
	 delay_ms(10);
	 PS2_Init();			 //驱动端口初始化
	 PS2_SetInit();		 //配配置初始化,配置“红绿灯模式”，并选择是否可以修改
	 TIM7_Int_Init(99,7199);   //10ms(99+1=100)一次中断
	 BEEP_Init(); 
   RED();	 
	 delay_ms(10);
}
/**************************************************************************
函数功能：运动解算
入口参数：
返回  值：
**************************************************************************/
void Car_Control(int Y_Move,int X_Move,int Yaw)
{
	Target_velocity1 = 	(-Y_Move - X_Move - Yaw)*30;	
	Target_velocity2 = (Y_Move - X_Move + Yaw)*30;
	Target_velocity3 = 	(-Y_Move + X_Move + Yaw)*30;	
	Target_velocity4= -(Y_Move + X_Move - Yaw)*30;	
};

/**************************************************************************
函数功能：ps2操作
入口参数：无
返回  值：无
**************************************************************************/
void ps2go(void)
{	
	Car_Control((PS2_LX-128)*3/5,(127-PS2_LY)*3/5,(PS2_RX-128)/2);
}
///**************************************************************************
//函数功能：选择模式
//入口参数：无
//返回  值：无
//**************************************************************************/
void changemode (void)
{
	if(runrun == 1)//手动操作模式
	{
		delay_us(10);
		ps2go();
	}
	else if(runrun == 2) //正常寻迹模式
  {
		read_sensor_values();
		delay_us(10);
	}
	else if(runrun == 3) //视觉识别＋寻迹
	{
		delay_us(10);
		USART1_IRQHandler();
	}
	else if(runrun == 4)
  {
		fread_sensor_values(); //第五关自动放置
		delay_us(10);
	}
}

///**************************************************************************
//函数功能：抓取抬升
//入口参数：无
//返回  值：无
//**************************************************************************/

	
void up_down(void)
{
		switch(PS2_KEY)
		{
			case 13: UP();  		
			break;
			case 16: DOWN(); 		
			break;
			case 14: CATCH();		
			break;
			case 15: RELEASE();	
			break;
			default :STOP();BEEP=0;
			break;
		}
}
/**************************************************************************
函数功能：抬升抓取函数
入口参数：
返回  值：
**************************************************************************/
void STOP (void)
{
	TIM_SetCompare3(TIM1,1500); // PE13
	TIM_SetCompare2(TIM1,1500);
}
void UP (void)
{
	TIM_SetCompare3(TIM1,500);
}
void DOWN (void)
{
	TIM_SetCompare3(TIM1,2500);
}
void CATCH (void)
{
	TIM_SetCompare1(TIM1,1800);//55°=499+10.811*55=1093 (抓取)  PE9
}
void RELEASE (void)
{
	TIM_SetCompare1(TIM1,800);//55°=499+10.811*55=1093(抓取)
}
void GOUP (void)
{
	TIM_SetCompare2(TIM1,2500);  //PE11
}
void GODOWN (void)
{
	TIM_SetCompare2(TIM1,500);
}
void mini (void)
{
  TIM_SetCompare1(TIM1,2200);
}
