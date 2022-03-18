#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED PAout(12) // PA12

void LED_Init(void);  //≥ı ºªØ
void Led_Flash(u16 time);

#endif
