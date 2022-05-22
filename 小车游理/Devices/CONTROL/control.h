#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
#include "PS2.h"
#include "red.h"
#include "Servo.h"
#include "beep.h"
#define PI 3.14159265

extern int Encoder1;
extern int Encoder2;
extern int Encoder3;
extern int Encoder4;

extern int Target1;
extern int Target2;
extern int Target3;
extern int Target4;

extern int Target_velocity1;
extern int Target_velocity2;
extern int Target_velocity3;
extern int Target_velocity4;

extern int moto1;
extern int moto2;
extern int moto3;
extern int moto4;

int  TIM7_IRQHandler(void); //定时器7中断函数//

int myabs(int a);

void Set_Pwm_1(int Moto1);
void Set_Pwm_2(int Moto2);
void Set_Pwm_3(int Moto3);
void Set_Pwm_4(int Moto4);

void Xianfu_Pwm1(void);
void Xianfu_Pwm2(void);
void Xianfu_Pwm3(void);
void Xianfu_Pwm4(void);

int Incremental_PI1 (int Encoder1,int Target1);
int Incremental_PI2 (int Encoder2,int Target2);
int Incremental_PI3 (int Encoder3,int Target3);
int Incremental_PI4 (int Encoder4,int Target4);



void STOP (void);
void UP (void);
void DOWN (void);
void CATCH (void);
void RELEASE (void);
void GOUP (void);
void GODOWN (void);
void mini (void);

void changemode (void);
void up_down(void);
void ps2go(void);
extern int runrun;
extern u8 PS2_KEY;
extern int PS2_LY;
extern int PS2_LX;
extern int PS2_RX;
extern int redmode;
int Read_Encoder(u8 TIMX);
void mainreturn (void);

#endif

