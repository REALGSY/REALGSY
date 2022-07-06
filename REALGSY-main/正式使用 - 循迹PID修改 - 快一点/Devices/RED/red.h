#include "sys.h"

#define red1 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_1)
#define red2 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_2)
#define red3 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_3)
#define red4 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_4)
#define red5 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_5)
#define red6 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_6)
#define red7 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_7)
#define red8 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_8)
#define red9 GPIO_ReadInputDataBit(GPIOG,GPIO_Pin_9)

void RED (void);
void rerecon(void);
void read_sensor_values(void);
