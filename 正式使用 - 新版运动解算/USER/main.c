#include "delay.h"
#include "Debug_DataScope.h"
#include "sys.h"
#include "stm32f10x_it.h"
#include "PS2.h"
#include "Servo.h"
#include "red.h"
 int PS2_LY,PS2_LX,PS2_RX;
 int main(void)
 {	
	 mainreturn();
	 while(1)
	 { 
		u8 PS2_KEY;
		delay_ms(10);
		PS2_KEY=PS2_DataKey();	
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_RX=PS2_AnologData(PSS_RX);
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
    if(PS2_KEY == 9)
	{
			runrun = 1;
			delay_us(10);
	}
	else  if(PS2_KEY == 10)
	{
			runrun = 2;
	}
	else  if(PS2_KEY == 11)
	{
	    runrun = 3;
	}
	else  if(PS2_KEY == 12)
  {
			runrun = 4;
		  delay_us(10);
	}
	  changemode();
  	delay_us(10);
   }

 }

 
