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
		/*****手柄遥感按键接收*******/
		u8 PS2_KEY; 
		delay_ms(10);
		PS2_KEY=PS2_DataKey();	
		PS2_LY=PS2_AnologData(PSS_LY);
		PS2_LX=PS2_AnologData(PSS_LX);
		PS2_RX=PS2_AnologData(PSS_RX);
		 /*****手柄操作抬升抓取*******/
				switch(PS2_KEY) 
		{
			case 3 : mini();
			break;
			case 13: UP();  		
			break;
			case 16: DOWN(); 		
			break;
			case 14: CATCH();		
			break;
			case 15: RELEASE();	
			break;
			case 5 : GOUP();
			break;
			case 7 : GODOWN();
			break;
			default :STOP();BEEP=0;
			break;
		}
	/*****手柄选择模式*******/
     if(PS2_KEY == 9) //
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
	  changemode(); //选择模式
  	delay_us(10);
  }
}

 
