#ifndef __EYES_H
#define __EYES_H
#include "sys.h"
#include "usart.h"
void eread_sensor_values(u8 eyesmode);
void eyesgogo (void);
void USART1_IRQHandler(void);
#define LEN 100
extern int eyesgo;
extern int j;
#endif

