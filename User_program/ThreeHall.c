//############################################################
// FILE: ThreeHall.c
// Created on: 2017年1月18日
// Author: XQ
// summary: ThreeHall
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
#include "ThreeHall.h"
#include "Tim1_PWM.h"
#include "Task_function.h"

extern   Hall   Hall_Three;
extern  logic   logicContr;

  uint16_t   HallK1=355; 		//滤波系数K1: 0.05/(T+0.05);
  uint16_t   HallK2=669 ;  	//滤波系数 K2: T/(T+0.05);
 
 void  ThreeHallPara_init(void )
{
   Hall_Three.Poles=4;  // 12k开关频率
	 Hall_Three.speed_coeff= 120000/Hall_Three.Poles; // 93750测试 一次霍尔换向计算一次速度 
	
	 //  rpm=speed_coeff [=60/(Hall_Three.Speed_count*0.00008333*6(6次换向)*4(4对级数))] /count
	
	
	  // 一次换向60度电角度内所花时间 pwm_count 是Hall_Three.Speed_count
	  // 90950是在开环电机1000RPM时候标定计算 用霍尔加示波器计算时间 一个周期示波器时间    120*1/0.0145/8=1034 rpm
	  //一个周期示波器时间14.5ms假设60*(1/0.015)/p(极对数) = RPM =1000RPM 把90950此位置设置变量，Keil在线调试修改此数值 
    // Hall_Three.Speed_count*0.0000833(一次pwm80us) 一次霍尔换向所用时间
    // 一次霍尔是1/6周期  4对级数 1/24圈 一机械圈所花时间乘24   Hall_Three.Speed_count*0.00008333*24(一次pwm80us)  
    // 计算每秒多少圈1/(Hall_Three.Speed_count*0.00008333*24)
	  // 计算每分钟多少圈 1/(Hall_Three.Speed_count*0.0000833*24)*(1/60) =60/(Hall_Three.Speed_count*0.0000833*24)
	  //=60/(Hall_Three.Speed_count*0.00008333*6(6次换向)*4(4对级数)) 
	  // 电机转速rpm=120000/4/Hall_Three.Speed_count
	
	
}

// CCW  41    16   63   32   25     54
// CW   32    25   54   41   16     63


void ThreeHall_huanxkz(void)  // 一个PWM周期执行一次
{
	   if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)==1)   //C   judge which phase on
     Hall_Three.HallUVW[0]=1;
		 else
		 Hall_Three.HallUVW[0]=0; 
   	 if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_14)==1)      //B
     Hall_Three.HallUVW[1]=1;
		 else
		 Hall_Three.HallUVW[1]=0; 
		 if (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_15)==1)  //A
     Hall_Three.HallUVW[2]=1;
		  else
		 Hall_Three.HallUVW[2]=0;  
	  Hall_Three.Hall_State = Hall_Three.HallUVW[0] +(Hall_Three.HallUVW[1]<<1) +(Hall_Three.HallUVW[2]<<2);
 if(  logicContr.Start_order==1  )
  {
    if ( Hall_Three.Hall_State!=Hall_Three.OldHall_State )  // now state != old state
      {
     	   switch (Hall_Three.Hall_State )
	      {
					case  Hall_num1:     // 0x6
					 {
					 if( logicContr.Run_mode==1)
					 MOS_Q41PWM();
						if( logicContr.Run_mode==2)
						
						 MOS_Q32PWM();
					
					 }
					break;
					case  Hall_num2:      // 0x4
					 {
					if( logicContr.Run_mode==1)
						 MOS_Q16PWM();
					if( logicContr.Run_mode==2)
						
					   MOS_Q25PWM();
					  
					 }
					break;
					case  Hall_num3:     //0x5
					 {
						if( logicContr.Run_mode==1)
						 MOS_Q63PWM();
						if( logicContr.Run_mode==2)
						 MOS_Q54PWM();
						
						
					 }
					 break;
					 case Hall_num4:    // 0x1
					 {
						 if( logicContr.Run_mode==1)
						   MOS_Q32PWM();
						 if( logicContr.Run_mode==2)
							 MOS_Q41PWM();
						  
						}
					 break;
					 case Hall_num5:    // 0x3
					 {
						 if( logicContr.Run_mode==1)
					  	 MOS_Q25PWM();
						 if( logicContr.Run_mode==2)
							 MOS_Q16PWM();
						  
					 }
					 break;
					 case Hall_num6:   // 0x2
						{
						if( logicContr.Run_mode==1)
							 MOS_Q54PWM();
						if( logicContr.Run_mode==2)
							 MOS_Q63PWM();
						  
						
						}
					 break;
						 default:
						 {
							Stop_Motor();
							Hall_Three.Speed_RPM=0;
						 }
					 break;
	    	 }	
 // 一阶数字低通把二个霍尔状态之间时间的计数平滑滤波，HallK1+ HallK2=1024，用y=a*y0+(1-a)*x;		 //  calculate   speed  		 
			
			Hall_Three.Speed_countFitter= _IQ10mpy(HallK2, Hall_Three.Speed_countFitter)+_IQ10mpy(HallK1,  Hall_Three.Speed_count);								  	 
      Hall_Three.Speed_RPM = Hall_Three.speed_coeff/Hall_Three.Speed_countFitter;
		  Hall_Three.Speed_RPMF= _IQ10mpy(HallK2, Hall_Three.Speed_RPMF)+_IQ10mpy(HallK1,  Hall_Three.Speed_RPM);	
  		Hall_Three.Speed_count= 0;        
      }

     else  if ( Hall_Three.Hall_State==Hall_Three.OldHall_State )
     {
    	 Hall_Three.Speed_count++;   
       if( Hall_Three.Speed_count>=8000 )
    	 {
    		 Hall_Three.Speed_count=0;
    		 Hall_Three.Speed_RPMF= 0;
				 Stop_Motor(); 
    		 Hall_Three.step_angleFitter=0;	     //5     %1      3      %5      1    %3      5
				 Hall_Three.Move_State=0;	
				 
				 
				 
				 
			   switch (Hall_Three.Hall_State )
	      {
					case  Hall_num1:     // 0x6
					 {
					 if( logicContr.Run_mode==1)
					 MOS_Q41PWM();
						if( logicContr.Run_mode==2)
						
						 MOS_Q32PWM();
					
					 }
					break;
					case  Hall_num2:      // 0x4
					 {
					if( logicContr.Run_mode==1)
						 MOS_Q16PWM();
					if( logicContr.Run_mode==2)
						
					   MOS_Q25PWM();
					  
					 }
					break;
					case  Hall_num3:     //0x5
					 {
						if( logicContr.Run_mode==1)
						 MOS_Q63PWM();
						if( logicContr.Run_mode==2)
						 MOS_Q54PWM();
						
						
					 }
					 break;
					 case Hall_num4:    // 0x1
					 {
						 if( logicContr.Run_mode==1)
						   MOS_Q32PWM();
						 if( logicContr.Run_mode==2)
							 MOS_Q41PWM();
						  
						}
					 break;
					 case Hall_num5:    // 0x3
					 {
						 if( logicContr.Run_mode==1)
					  	 MOS_Q25PWM();
						 if( logicContr.Run_mode==2)
							 MOS_Q16PWM();
						  
					 }
					 break;
					 case Hall_num6:   // 0x2
						{
						if( logicContr.Run_mode==1)
							 MOS_Q54PWM();
						if( logicContr.Run_mode==2)
							 MOS_Q63PWM();
						  
						
						}
					 break;
						 default:
						 {
							Stop_Motor();
							Hall_Three.Speed_RPM=0;
						 }
					 break;
	    	 }	



					
       }				 
     }
	 }		 
   Hall_Three.OldHall_State=Hall_Three.Hall_State ;
}

 
//===========================================================================
// No more.
//===========================================================================
