#include "motor.h"
#include "remoter.h"
#include "can.h"
void shot_IRQHandler(void);
void roll_Init (void);
void roll_shot_go (void);
void if_or_no (void);
void angle_control (void);
void Roll_Init (void);
void Roll_Preload (void);
void Roll_Mode (void);
#define one_part_roll (8191 * 36 / 9); //转动此角度值发送一个弹丸

