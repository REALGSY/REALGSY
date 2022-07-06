/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 
u8 Res1;

 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
//TIM2中断函数
void TIM2_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM2,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//清除中断标志位 
 }
}
//TIM3中断函数
void TIM3_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM3,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(TIM3,TIM_IT_Update);//清除中断标志位 
 }
}
//TIM4中断函数
void TIM4_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(TIM4,TIM_IT_Update);//清除中断标志位 
 }
}
//TIM5中断函数
void TIM5_IRQHandler(void)
{
 if(TIM_GetFlagStatus(TIM5,TIM_FLAG_Update)==SET)//溢出中断
 {
	 TIM_ClearITPendingBit(TIM5,TIM_IT_Update);//清除中断标志位 
 }
}
//串口中断函数
void S_USA_Handler(void)
{
	
	 if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
   {
			USART_ClearITPendingBit(USART1,USART_IT_RXNE);

			Res1 =USART_ReceiveData(USART1);	//读取接收到的数据
		 
			USART_SendData(USART1,Res1);
	 }
}
