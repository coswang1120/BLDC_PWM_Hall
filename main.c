//############################################################
// FILE:  main.c
// Created on: 2017年1月15日
// Author: XQ
// summary: main 主程序的MCU底层初始化，通讯函数
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//修改日期:2018/7/12
//版本：V17_3
//Author-QQ: 616264123
//电机控制QQ群：314306105

//  hall sensor

//  4 poles pair

//HA HB   0      6
//HA 0    0      4
//HA 0    HC     5
//0  0    HC     1
//0  HB   HC     3
//0  HB   0      2


// HA+ (1)  HB+ (3)    HC+ (5)
//
// HA- (2)  HB- (4)    HC- (6)
//
//
// CCW  41    16   63   32   25     54
// CW   32    25   54   41   16     63


//  6 -> 4  -> 5  -> 1  -> 3  -> 2   (Q41) -> (Q16)   -> (Q63)  -> (Q32)   -> (Q25)  -> (Q54)    CCW
//                                   (Q32) -> (Q25)   -> (Q54)  -> (Q41)   -> (Q16)  -> (Q63)    CW

// TIM1   -->  PWM (真正PWM)    TIM4  -->  DEMO PWM  可以拉出來看訊號 [TIM4PWMDAC_Config]

//  sample time=83us   control time=2ms
//  Hall_Three.Speed_RPMF   estimated rpm from hall sensor
//  pi_spd.Ref   rotation button  0 -4000   [target speed]

//  Duty   max-> 4000

//  can use time.c add parameter to debug

//  logicContr.Run_mode=1 -> DUTY = 2*pi_spd.Ref  = 2* rotation button
//  logicContr.Run_mode=2 -> I_kp = (rotation button-Bus_Curr)   V_kp=  (rotation button-Hall_Three.Speed_RPMF[Feedback velocity from hall sensor]) 

//############################################################


#include "stm32f10x.h"
#include "GPIO_int.h"
#include "Timer.h"
#include "ADC_int.h"
#include "Tim1_PWM.h"
#include "ThreeHall.h"
#include "IQ_math.h"
#include "Tim4_Encoder_PWMDAC.h"
#include "PI_Cale.h"
#include "Task_function.h"
#include "Usart_RS232.h"
// #include "CAN.h"
#include <stdio.h>
#include <math.h>

u16 t;
u16 len;
long numdec=0;
float syst=0.0;
PI_Control   pi_spd = PI_Control_DEFAULTS;
PI_Control   pi_ICurr = PI_Control_DEFAULTS;
Test         TestPare=Test_DEFAULTS;
TaskTime     TaskTimePare=TaskTime_DEFAULTS;
logic        logicContr=logic_DEFAULTS;
ADCSamp      ADCSampPare=ADCSamp_DEFAULTS;
Hall         Hall_Three=Hall_DEFAULTS ;
// CANSR        CANSRPare=CANSR_DEFAULTS;
extern u16 Tag; 
u16 USART_RX_STA;
u16 spdcmd;
extern  uint16_t  DUTY;
u8 Res;

int fputc( int ch , FILE *f);
 

int fputc( int ch , FILE *f)  // print function for RS232
{
    USART_SendData( USART3 , (u8)ch ) ;
    while(USART_GetFlagStatus(USART3 , USART_FLAG_TC) == RESET)
    {
    }
    return ch;
}

int main(void) {
    // 2ms  control  knob control
    Delay(10000);
    SysTickConfig();              // 10ms
    logicContr.Control_Mode = 1;  //   1 -> DUTY = 2*pi_spd.Ref    2 ->   close
                                  //   loop    I & rotational speed
    logicContr.Run_mode = 1;      //     1 ->  CCW     2 -> CW
    GPIO_LED485RE_int();          // RS485 和运行LED
    Init_Gpio_ADC();              // ADC的引脚初始化      83us
    InitUSART3_Gpio();            // 串口3IO初始化
    // InitCAN_Gpio();               // CAN通讯的IO初始化
    // CAN_Config();                 // CAN通讯的初始化
    Init_Gpio_TIM1_PWM();         // 高级定时器1的6个IO初始化   // pwm
                                  // 12K       83.333us
    InitThreeHallGpio();          // 霍尔的IO初始化
    Init_PWMDAC_Gpio();           // PWM4的IO作为DAC初始化
    ThreeHallPara_init();         // 三霍尔角度传感器的参数初始化
    Usart3_RS232_init();          // 串口3初始化
    // CAN_Config();                 // CAN通讯的初始化
    DMA_Configuration();          // ADC连接DMA读取数据初始化
    Delay(10000);
    ADC1_Configuration();  // ADC模式初始化      1us
    Delay(10000);
    Tim1_PWM_Init();  // 高级定时器1初始化   Tim1 ini   Tim int  ->   ADC /
                      // offset curr  /PWM   /   6 step     /    -->
                      // TIM1_ISR_MCLoop controler scheme  83 us
    Delay(10000);
    TIM4PWMDAC_Config();  // TIM4的 作为DAC初始化
    Delay(10000);
    Offset_CurrentReading();  // 电机的母线电流采样偏执电压   initial current
                              // U_Curr   V_Curr  Bus_Curr   and minue it
    Delay(10000);
    PI_Pare_init();  // 三个双PID参数初始化

    while (1) {
        RunSystimer();        // 时间任务标志初始化  call 10ms
        // CAN_Sendlen();        // CAN定时发送电机参数     // useless
        Uart3_RS232TX_sen();  // 串口3通讯的定时发送		  // send print
                              // command
        ReceiveData_chuli();  // 串口中断接收数据处理
        // CAN_Receivechuli();   // CAN通讯中断接收数据处理	 // useless
        CLEAR_flag();         // 清除时间任务标志   clear flag
                              // printf("%d \r\n",Hall_Three.Speed_RPMF);




        //printf("OK\r\n");

        // if (USART_RX_STA == 1) {
        //     //

        //     if (Res == '1') {
        //         logicContr.Control_Mode = 1;
        //         spdcmd = 500;
        //     } else if (Res == '2') {
        //         logicContr.Control_Mode = 1;
        //         spdcmd = 800;

        //     } else if (Res == '3') {
        //         logicContr.Control_Mode = 1;
        //         spdcmd = 1000;

        //     } else if (Res == '0') {
        //         logicContr.Control_Mode = 1;
        //         spdcmd = 0;

        //     }

        //     else {
        //         logicContr.Control_Mode = 2;
        //         // printf("Others\r\n");
        //     }
        //     // printf("%d \r\n",logicContr.Control_Mode);
        //     USART_RX_STA = 0;
        // }
}
}

//===========================================================================
// No more.
//===========================================================================

