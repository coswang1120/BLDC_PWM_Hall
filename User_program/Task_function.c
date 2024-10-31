//############################################################
// FILE: Task_function.c
// Created on: 2017年1月18日
// Author: XQ
// summary: Task_function
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
t1
t2
t3

#include "Task_function.h"
#include "Timer.h"
#include "PI_Cale.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"

extern  ADCSamp        ADCSampPare;
extern  TaskTime       TaskTimePare;
extern  PI_Control     pi_spd  ;
extern  logic          logicContr;
extern  u16 spdcmd;

void knob_control(void)
{
//  电位器给定0.3V左右开始开启PWM给定速度和电压 
	
	
	if (logicContr.Control_Mode==2){
	
	 if (( ADCSampPare.RP_speed_Voltage<= 350)  ) // 电位器旋转一定位置开始给定速度输出控制电机
     {
	// 启动电机控制需要初始化参数，三个PI环路参数	
		 logicContr.drive_car=0;
			 
			 
		 logicContr.Start_order=0;
		 logicContr.Qiehuan_count=0;

		 pi_spd.Fbk=0;
		 pi_spd.Ref=0;
	   pi_spd.up=0;
	   pi_spd.ui=0;
	   pi_spd.v1=0;
	   pi_spd.i1=0;
	   pi_spd.Out=0;
	   pi_spd.OutF=0;
 
	}
	else
	{
	    logicContr.drive_car=1;
 	    pi_spd.Ref =  ADCSampPare.RP_speed_Voltage-300;   // 电位器转速信号
 	    if( pi_spd.Ref>=2500)   //限制最大转速
 	    pi_spd.Ref=2500;
	}

     if ((logicContr.olddrive_car==0) &&( logicContr.drive_car==1 ))
     {
   	  logicContr.Start_order=1;
     }
     else if ((logicContr.olddrive_car==1) &&( logicContr.drive_car==1 ))
	   { 
	   }
     else     // (logicContr.olddrive_car==1) &&( logicContr.drive_car==0 )        (logicContr.olddrive_car==0) &&( logicContr.drive_car==0 )
   	 {
		  Stop_Motor();
		  logicContr.Start_order=0;
	   }
	  logicContr.olddrive_car=logicContr.drive_car;
		 
	 }
   else{    //  logicContr.Control_Mode==1
	 
				if (( spdcmd < 1 )  ) // 电位器旋转一定位置开始给定速度输出控制电机
					 {
				// 启动电机控制需要初始化参数，三个PI环路参数	
					 logicContr.drive_car=0;
						 
						 
					 logicContr.Start_order=0;
					 logicContr.Qiehuan_count=0;

					 pi_spd.Fbk=0;
					 pi_spd.Ref=0;
					 pi_spd.up=0;
					 pi_spd.ui=0;
					 pi_spd.v1=0;
					 pi_spd.i1=0;
					 pi_spd.Out=0;
					 pi_spd.OutF=0;
			 
				}
				else
				{
						logicContr.drive_car=1;
						pi_spd.Ref =  ADCSampPare.RP_speed_Voltage-300;   // 电位器转速信号
						if( pi_spd.Ref>=2500)   //限制最大转速
						pi_spd.Ref=2500;
				}

					 if ((logicContr.olddrive_car==0) &&( logicContr.drive_car==1 ))
					 {
						logicContr.Start_order=1;
					 }
					 else if ((logicContr.olddrive_car==1) &&( logicContr.drive_car==1 ))
					 { 
					 }
					 else     // (logicContr.olddrive_car==1) &&( logicContr.drive_car==0 )        (logicContr.olddrive_car==0) &&( logicContr.drive_car==0 )
					 {
						Stop_Motor();
						logicContr.Start_order=0;
					 }
					logicContr.olddrive_car=logicContr.drive_car;
				 
				 }	
 
}


//===========================================================================
// No more.
//===========================================================================
