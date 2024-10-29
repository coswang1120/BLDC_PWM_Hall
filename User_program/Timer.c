//############################################################
// FILE: Timer.c
// Created on: 2017年1月11日
// Author: XQ    
// summary: Timer    
// 定时器1电机控制，
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究    
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com   
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
#include "Timer.h"
#include "GPIO_int.h"
#include "Tim1_PWM.h"

#include "stm32f10x.h"
#include "ADC_int.h"
#include "ThreeHall.h"
#include "IQ_math.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "PI_Cale.h"
#include "Task_function.h"
#include "Usart_RS232.h"
#include "CAN.h"
#include <stdio.h>
#include <math.h>



extern  Hall           Hall_Three;
extern  TaskTime       TaskTimePare;    
extern int16_t IQSin_Cos_Table[256];

 
void SysTickConfig(void)
{
  /* Setup SysTick Timer for 1ms interrupts  */
  if (SysTick_Config(SystemCoreClock /100))   //  10ms
  {
    /* Capture error */ 	
    while (1);
  }
  /* Configure the SysTick handler priority */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}
 
void RunSystimer(void)  //  every loop clear flag but not count 
{
   if(TaskTimePare.IntClock_10ms==1)            // 10ms     stm32f10x_it.c   enable TaskTimePare.IntClock_10ms
    {     
			
     TaskTimePare.IntClock_10ms= 0;
			
	   TaskTimePare.Tim10ms_flag = 1;
			
			
		 if(++TaskTimePare.Tim100ms_count >=10)    // 100ms
			 {
			 TaskTimePare.Tim100ms_count=0;
			 TaskTimePare.Tim100ms_flag=1;
			 }
			 
	    if(++TaskTimePare.Tim500ms_count >=50)    // 500ms
	    {	      
	       TaskTimePare.Tim500ms_count=0;
	       TaskTimePare.Tim500ms_flag=1;
      }				 

			if(++TaskTimePare.Tim300ms_count >=30)    // 300ms
	    {	      
	       TaskTimePare.Tim300ms_count=0;
	       TaskTimePare.Tim300ms_flag=1;
      }

			if(++TaskTimePare.Tim400ms_count >=40)    // 400ms
	    {	      
	       TaskTimePare.Tim400ms_count=0;
	       TaskTimePare.Tim400ms_flag=1;
      }	

			if(++TaskTimePare.Tim200ms_count >=20)    // 200ms
	    {	      
	       TaskTimePare.Tim200ms_count=0;
	       TaskTimePare.Tim200ms_flag=1;
      }	

	    if(++TaskTimePare.Tim1s_count >=100)       // 1s
	    {	        
				 LED1_Toggle( );
	       TaskTimePare.Tim1s_count=0;
	       TaskTimePare.Tim1s_flag=1;	
         //printf("%d \r\n", IQSin_Cos_Table[29]);					
      }

		  if(++TaskTimePare.Tim10s_count >=1000)    // 10s
			{
					TaskTimePare.Tim10s_count = 0;
					TaskTimePare.Tim10s_flag = 1;
			}

			if(++TaskTimePare.Tim1min_count >=6000)   // 1min
			{
					TaskTimePare.Tim1min_count = 0;
			   	TaskTimePare.Tim1min_flag = 1;
			}
			
			 
    }
		
		
		/*
		
   if(TaskTimePare.Tim100ms_flag==1)           // reference 100ms
	   {		   		 
	    if(++TaskTimePare.Tim500ms_count >=5)    // 500ms
	    {	      
	       TaskTimePare.Tim500ms_count=0;
	       TaskTimePare.Tim500ms_flag=1;
      }			
			
			if(++TaskTimePare.Tim300ms_count >=3)    // 300ms
	    {	      
	       TaskTimePare.Tim300ms_count=0;
	       TaskTimePare.Tim300ms_flag=1;
      }	

			if(++TaskTimePare.Tim400ms_count >=4)    // 400ms
	    {	      
	       TaskTimePare.Tim400ms_count=0;
	       TaskTimePare.Tim400ms_flag=1;
      }	
			
			
			
			
			

			if(++TaskTimePare.Tim200ms_count >=2)    // 200ms
	    {	      
	       TaskTimePare.Tim200ms_count=0;
	       TaskTimePare.Tim200ms_flag=1;
      }				
			
			
	   }
		 
		 
   if(TaskTimePare.Tim500ms_flag==1)           // reference 500ms
	   {            
	    if(++TaskTimePare.Tim1s_count >=2)       // 1s
	    {	               
	       TaskTimePare.Tim1s_count=0;
	       TaskTimePare.Tim1s_flag=1;
				
      }
	   }
   if(TaskTimePare.Tim1s_flag == 1)             // reference 1s
	   {      
			 LED1_Toggle( ); 

			 
			 if(++TaskTimePare.Tim10s_count >=10)    // 10s
				{
					TaskTimePare.Tim10s_count = 0;
					TaskTimePare.Tim10s_flag = 1;
				}
	   } 
    if(TaskTimePare.Tim10s_flag == 1)         // reference 10s
	   {                   
			 if(++TaskTimePare.Tim1min_count >=6)   // 1min
				{
					TaskTimePare.Tim1min_count = 0;
					TaskTimePare.Tim1min_flag = 1;
				}
	   }
		 
		 */
}

void CLEAR_flag(void)   //清除事件标志位
{
	
	if(TaskTimePare.Tim10ms_flag == 1){
		TaskTimePare.Tim10ms_flag=0;		
	}	
	
	if(TaskTimePare.Tim100ms_flag == 1){
		TaskTimePare.Tim100ms_flag=0;		
	}
	if(TaskTimePare.Tim500ms_flag == 1){
		TaskTimePare.Tim500ms_flag=0;		
	}
	if(TaskTimePare.Tim400ms_flag == 1){
		TaskTimePare.Tim400ms_flag=0;		
	}
	if(TaskTimePare.Tim300ms_flag == 1){
		TaskTimePare.Tim300ms_flag=0;		
	}
	if(TaskTimePare.Tim200ms_flag == 1){
		TaskTimePare.Tim200ms_flag=0;		
	}
	if(TaskTimePare.Tim1s_flag == 1){
		TaskTimePare.Tim1s_flag=0;		
	}
	if(TaskTimePare.Tim10s_flag == 1){
		TaskTimePare.Tim10s_flag=0;		
	}
	if(TaskTimePare.Tim1min_flag == 1){
		TaskTimePare.Tim1min_flag=0;		
	}
	
	/*
	TaskTimePare.Tim10ms_flag=0;
	TaskTimePare.Tim100ms_flag=0;
	TaskTimePare.Tim500ms_flag=0;
  TaskTimePare.Tim400ms_flag=0;
	TaskTimePare.Tim300ms_flag=0;
	
	TaskTimePare.Tim200ms_flag=0;	
	
	TaskTimePare.Tim1s_flag=0;
	TaskTimePare.Tim10s_flag=0;
	TaskTimePare.Tim1min_flag=0;
	
	*/
	
	
	
	
	
	
}


//===========================================================================
// No more.
//===========================================================================
