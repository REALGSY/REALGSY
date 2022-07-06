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
�������ܣ���ʱ��7�жϺ���
��ڲ�������
����  ֵ����
**************************************************************************/

int TIM7_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM7,TIM_FLAG_Update)==SET)
 {
   TIM_ClearITPendingBit(TIM7,TIM_IT_Update);   //===�����ʱ��7�жϱ�־λ
	 Encoder1=Read_Encoder(2);                     //ȡ��ʱ��2��������ֵ
	 Encoder2=Read_Encoder(3);
	 Encoder3=Read_Encoder(4);
	 Encoder4=Read_Encoder(5);
	 
   Led_Flash(100);                              //LED��˸
	 
	 moto1=Incremental_PI1(Encoder1,Target_velocity1);    //===λ��PID������
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
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm_1(int Moto1)//��ֵ��PWM�Ĵ���
{
 if(Moto1>0) AIN1=1,   AIN2=0;
	else       AIN1=0,   AIN2=1;
	PWMA=myabs(Moto1);
}
void Set_Pwm_2(int Moto2)//��ֵ��PWM�Ĵ���
{
 if(Moto2>0) BIN1=1,   BIN2=0;
	else       BIN1=0,   BIN2=1;
	PWMB=myabs(Moto2);
}
void Set_Pwm_3(int Moto3)//��ֵ��PWM�Ĵ���
{
 if(Moto3>0) AIN21=1,   AIN22=0;
	else       AIN21=0,   AIN22=1;
	PWMA2=myabs(Moto3);
}
void Set_Pwm_4(int Moto4)//��ֵ��PWM�Ĵ���
{
 if(Moto4>0) BIN21=1,   BIN22=0;
	else       BIN21=0,   BIN22=1;
	PWMB2=myabs(Moto4);
}
/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
 void Xianfu_Pwm1(void) //���ƣ���һ�������ȵĺ���
 {
   int Amplitude=7100;  //===PWM������7200 ������7100
	 if(moto1<-Amplitude)  moto1 = -Amplitude;
	 if(moto1>Amplitude)   moto1 =  Amplitude;
 }
  void Xianfu_Pwm2(void) //���ƣ��ڶ��������ȵĺ���
 {
   int Amplitude=7100;  //===PWM������7200 ������7100
	 if(moto2<-Amplitude)  moto2 = -Amplitude;
	 if(moto2>Amplitude)   moto2 =  Amplitude;
 }
  void Xianfu_Pwm3(void) //���ƣ������������ȵĺ���
 {
   int Amplitude=7100;  //===PWM������7200 ������7100
	 if(moto3<-Amplitude)  moto3 = -Amplitude;
	 if(moto3>Amplitude)   moto3 =  Amplitude;
 }
  void Xianfu_Pwm4(void) //���ƣ����ĸ������ȵĺ���
 {
   int Amplitude=7100;  //===PWM������7200 ������7100
	 if(moto4<-Amplitude)  moto4 = -Amplitude;
	 if(moto4>Amplitude)   moto4 =  Amplitude;
 }
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int a
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a) //ȡ����ֵ
{ 		   
	 int temp;
	 if(a<0)  temp=-a;  
	 else temp=a;
	 return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI1 (int Encoder11,int Target1)
{ 	
	 static int Bias1,Pwm1,Last_bias1;
	 Bias1=Encoder11-Target1;                //����ƫ��
	 Pwm1+=Kp1*(Bias1-Last_bias1)+Ki1*Bias1;   //����ʽPI������
	 Last_bias1=Bias1;	                   //������һ��ƫ�� 
	 return Pwm1;                         //�������
}
int Incremental_PI2 (int Encoder22,int Target2)
{ 	
	 static int Bias2,Pwm2,Last_bias2;
	 Bias2=Encoder22-Target2;                //����ƫ��
	 Pwm2+=Kp2*(Bias2-Last_bias2)+Ki2*Bias2;   //����ʽPI������
	 Last_bias2=Bias2;	                   //������һ��ƫ�� 
	 return Pwm2;                         //�������
}

int Incremental_PI3 (int Encoder33,int Target3)
{ 	
	 static int Bias3,Pwm3,Last_bias3;
	 Bias3=Encoder33-Target3;                //����ƫ��
	 Pwm3+=Kp3*(Bias3-Last_bias3)+Ki3*Bias3;   //����ʽPI������
	 Last_bias3=Bias3;	                   //������һ��ƫ�� 
	 return Pwm3;                         //�������
}

int Incremental_PI4 (int Encoder44,int Target4)
{ 	
	 static int Bias4,Pwm4,Last_bias4;
	 Bias4=Encoder44-Target4;                //����ƫ��
	 Pwm4+=Kp4*(Bias4-Last_bias4)+Ki4*Bias4;   //����ʽPI������
	 Last_bias4=Bias4;	                   //������һ��ƫ�� 
	 return Pwm4;                         //�������
}
/**************************************************************************
�������ܣ�main��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void mainreturn (void)
{
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 SystemInit();             //����ϵͳʱ��
   delay_init();            
	 uart_init(115200);        //���ò�����Ϊ115200
	 MOTO_Init1();              //��ʼ�����Ƶ�������IO
	 MOTO_Init2();
	 pwm_Init();//��ʼ��pwm���
	 Encoder_Init_TIM2();      //��ʼ����ʱ������������
	 Encoder_Init_TIM3();
	 Encoder_Init_TIM4();
	 Encoder_Init_TIM5();
	 TIM1_PWM();
	 delay_ms(10);
	 PS2_Init();			 //�����˿ڳ�ʼ��
	 PS2_SetInit();		 //�����ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	 TIM7_Int_Init(99,7199);   //10ms(99+1=100)һ���ж�
	 BEEP_Init(); 
   RED();	 
	 delay_ms(10);
}
/**************************************************************************
�������ܣ��˶�����
��ڲ�����
����  ֵ��
**************************************************************************/
void Car_Control(int Y_Move,int X_Move,int Yaw)
{
	Target_velocity1 = 	(-Y_Move - X_Move - Yaw)*30;	
	Target_velocity2 = (Y_Move - X_Move + Yaw)*30;
	Target_velocity3 = 	(-Y_Move + X_Move + Yaw)*30;	
	Target_velocity4= -(Y_Move + X_Move - Yaw)*30;	
};

/**************************************************************************
�������ܣ�ps2����
��ڲ�������
����  ֵ����
**************************************************************************/
void ps2go(void)
{	
	Car_Control((PS2_LX-128)*3/5,(127-PS2_LY)*3/5,(PS2_RX-128)/2);
}
///**************************************************************************
//�������ܣ�ѡ��ģʽ
//��ڲ�������
//����  ֵ����
//**************************************************************************/
void changemode (void)
{
	if(runrun == 1)//�ֶ�����ģʽ
	{
		delay_us(10);
		ps2go();
	}
	else if(runrun == 2) //����Ѱ��ģʽ
  {
		read_sensor_values();
		delay_us(10);
	}
	else if(runrun == 3) //�Ӿ�ʶ��Ѱ��
	{
		delay_us(10);
		USART1_IRQHandler();
	}
	else if(runrun == 4)
  {
		fread_sensor_values(); //������Զ�����
		delay_us(10);
	}
}

///**************************************************************************
//�������ܣ�ץȡ̧��
//��ڲ�������
//����  ֵ����
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
�������ܣ�̧��ץȡ����
��ڲ�����
����  ֵ��
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
	TIM_SetCompare1(TIM1,1800);//55��=499+10.811*55=1093 (ץȡ)  PE9
}
void RELEASE (void)
{
	TIM_SetCompare1(TIM1,800);//55��=499+10.811*55=1093(ץȡ)
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
