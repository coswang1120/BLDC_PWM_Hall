//############################################################
// FILE: Tim1_ISR_MCLoop.c
// Created on: 2017年1月15日
// Author: XQ
// summary: Tim1_ISR_MCLoop
// 定时器1电机控制， 中断环路闭环控制  
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2018/7/12
//版本：V17_3
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#include "Tim1_ISR_MCLoop.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
#include "GPIO_int.h"
#include "ThreeHall.h"
#include "Task_function.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "Usart_RS232.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "stdio.h" 
extern   PI_Control   pi_ICurr; 
extern   PI_Control   pi_spd ;
extern   ADCSamp    ADCSampPare;
extern   uint16_t   PWM_DUTY[3] ;
extern   Hall       Hall_Three;
extern   logic      logicContr;
extern   TaskTime   TaskTimePare;  
extern   Test       TestPare;

extern  uint16_t  DUTY;
extern  u16 spdcmd;

uint16_t  FilK1=328;
uint16_t  FilK2=696;
// 一阶数字低通滤波器   328+696=1024  a=696/1024     (696/1024) /(2*3.14*0.02) = 54hz  cutoff freq.    
// https://wenku.baidu.com/view/2d022b6b7375a417866f8f7e.html?from=search 
 
 
void TIM1_UP_IRQHandler(void)// 触发ADC中断采样和电机环路控制   83.333us
{
	//此程序更新中断一直执行
	   
	 ADC_Sample( ); // 电流母线电流电压采集  PWM中断进入先读取AD电流和位置换向
	
	
   ThreeHall_huanxkz( ); // 根据三霍尔切换PWM驱动MOS   //  control comand
	
	 if(logicContr.drive_car==0)// 
	 {
		   Offset_CurrentReading(); // 电流采样偏执电压读取
	 }
	
	
	 TaskTimePare.PWMZD_count++;
	
   if(TaskTimePare.PWMZD_count>24 ) //2ms时间任务 ,[ 计算24*83.333us=2ms时间任务 12K开关频率 ]
	 {
				 TaskTimePare.PWMZD_count=0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
					 
				 knob_control( );  //转把电位器调速的控制

				//通讯测试参数输出 
				//测试参数输出，以下为打包测试参数包括电压电流转速等参数，以便通讯发送输出参数  
				 TestPare.fact_BUS_Voil= ADCSampPare.BUS_Voltage;
				 TestPare.fact_BUS_Curr= ADCSampPare.BUS_CurrF;
		 
		 
				 TestPare.Speed_fact= pi_spd.Fbk;
				 
				 TestPare.Speed_target= pi_spd.Ref;	 	   
			 
				 //---------------速度环------------------------//    speed resistometer controller
				 pi_spd.Fbk = Hall_Three.Speed_RPMF ;  //  0--4000rpm   	 
				 PI_Controller((p_PI_Control)&pi_spd);  // 速度环PI控制 
				 pi_spd.OutF= _IQ10mpy(FilK1,pi_spd.OutF)+_IQ10mpy(FilK2,pi_spd.Out); 
				 //---------------------------------------------//
		 
				 
				 //---------------母线电流环-----------------------------//
				 pi_ICurr.Ref= ADCSampPare.RP_speed_Voltage/20+300;   //0.8A+1A=1.8A 若放大电流 将20改小，硬件不能超过熔断丝3A

				 pi_ICurr.Fbk= ADCSampPare.BUS_CurrF;       //0.10A是系统抖动
		 
				 PI_Controller((p_PI_Control)&pi_ICurr);  // 电流环PI控制 
				 pi_ICurr.OutF= _IQ10mpy(FilK1,pi_ICurr.OutF)+_IQ10mpy(FilK2,pi_ICurr.Out); 
				 //-------------------------------------------------//
				 
			 
				 if(logicContr.Control_Mode==1)
				 {
						//DUTY = 2*pi_spd.Ref;  // 速度 电压开环 PWM输出    max (pi_spd.Ref)=2500 
					  DUTY = 2*spdcmd; 
					  //printf("%d \r\n",DUTY);
					 
					  if(DUTY>=2500){   //限制最大转速
 	              DUTY=2500;
						}			 
					 
				 } 
				 else if(logicContr.Control_Mode==2)  //电流环速度环闭环，母线电流闭环做限制母线电流控制
				 {
						//---------------电流环 速度环  二个并联环路应用------------------------//
							if ( pi_ICurr.OutF>pi_spd.OutF)	 
							{
								DUTY= pi_spd.OutF;              // 采用速度闭环 
								pi_ICurr.ui  = pi_spd.OutF;
								pi_ICurr.i1  = pi_spd.OutF;
							} 
							else 
							{
								DUTY= pi_ICurr.OutF;          // 采用母线电流限制环路控制
								pi_spd.ui  = pi_ICurr.OutF;
								pi_spd.i1  = pi_ICurr.OutF;
							}
						//----------------------------------------------------//
				 }  
     }
	 
   Protection_software();// 软件保护直接关闭管子  600是3A左右，可调试修改，硬件不准许大于3A
	 
   TIM_SetCompare3(TIM4 , Hall_Three.Speed_RPMF); // DAC通道1   ( demo )  
	 TIM_SetCompare4(TIM4 , DUTY); // DAC通道2    you can use microwave to see figure    ( demo ) 
	 
	 TIM_ClearFlag(TIM1, TIM_FLAG_Update);
} 


//===========================================================================
// No more.
//===========================================================================
