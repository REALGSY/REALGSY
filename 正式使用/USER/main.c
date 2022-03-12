#include "delay.h"
#include "Debug_DataScope.h"
#include "sys.h"
#include "stm32f10x_it.h"
#include "PS2.h"
#include "Servo.h"
#include "beep.h" 
#include "red.h"
 int PS2_LY,PS2_LX,PS2_RX;
 int main(void)
 {	
	 mainreturn();
	 BEEP_Init(); 
   RED();	 
	 while(1)
	 { 
		u8 PS2_KEY;
		delay_ms(50);
		PS2_KEY=PS2_DataKey();	
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_RX=PS2_AnologData(PSS_RX);
    if(PS2_KEY == 11)
	{
			runrun = 1;
			delay_us(10);
	}
	else  if(PS2_KEY == 9)
	{
			runrun = 3;
	}
	else  if(PS2_KEY == 10)
	{
	    runrun = 4;
	}
	else  if(PS2_KEY == 12)
  {
			runrun = 2;
		  delay_us(10);
	}
	  changemode();
  	delay_us(10);
   }

 }

 
