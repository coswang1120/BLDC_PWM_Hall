//############################################################
// FILE: IQ_math.h
// Created on: 2017年1月18日
// Author: XQ
// summary: IQ_math_
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef __IQ_math_H
#define __IQ_math_H

#include "stm32f10x.h"

//共256个数

#define Abs(A)    ((A>=0)?A:-A)  // 绝对值函数
#define Min(A,B)  ((A<=B)?A:B)   // 求最小函数
#define Max(A,B)  ((A>=B)?A:B)   // 求最大函数

#define _IQ15(A) ((int32_t)((A) * (1 << 15)))    // 左移15位 32768
#define _IQ(A) _IQ15(A)                          // 定义IQ格式 32768
#define _IQmpy(A, B) (int32_t)((A * B) >> 15)    // IQ格式相乘
#define _IQ10mpy(A, B) (int32_t)((A * B) >> 10)  // IQ10格式相乘
#define _IQdiv2(A) (int32_t)((A) >> 1)           // 除2
#define _IQmpy2(A) (int32_t)(A << 1)             // 乘2
#define _IQdiv(A, B) (int32_t)((A << 15) / B)    // IQ格式相除
#define _IQ15int(A) ((A) >> 15)
typedef int32_t _iq15;

#define   _IQ15_MAX      (0x7FFF)
#define   _IQ15_MIN      (-0x8000)

/*float */
typedef float _iq15f;
#define _IQ15f(A)         (A)
#define _IQ15frac(A)      ((A) - (float)((long)(A)))
#define _IQ15fint(A)             ((int32_t)(A))
#define   _IQmpy_SAT(A,B) ({ \
    int64_t temp = (int64_t)(A) * (B); \
    (int32_t)((temp + (1 << 14)) >> 15); \
})	


#define SIN_RAD     0x0300
#define U0_90       0x0000 
#define U90_180     0x0100
#define U180_270    0x0200
#define U270_360    0x0300

typedef struct {
    int32_t IQAngle;  // 电机磁极位置角度0---65536即是0---360度
    int32_t IQSin;    // IQ格式正弦参数，-32768---32767  -1到1
    int32_t IQCos;    // IQ格式余弦参数，-32768---32767  -1到1
} IQSin_Cos, *p_IQSin_Cos;

// static inline int32_t _IQ15_CHECK_RANGE(int32_t x) {
//     if (x > _IQ15_MAX) return _IQ15_MAX;
//     if (x < _IQ15_MIN) return _IQ15_MIN;
//     return x;
// }

#define IQSin_Cos_DEFAULTS  {0,0,0} // 初始化参数

typedef struct {
    int32_t Alpha;      // 二相静止坐标系 Alpha 轴
    int32_t Beta;       // 二相静止坐标系 Beta 轴
    int32_t IQTan;      // IQ格式正切 45度正切是1，IQ的格式是
    int32_t IQAngle;    // IQ格式角度值 0---65536 == 0---360度
    int32_t JZIQAngle;  // 矫正IQ格式角度值
} IQAtan, *p_IQAtan;

#define IQAtan_DEFAULTS  {0,0,0,0,0}  // 初始化参数

uint32_t IQSqrt(uint32_t  M); // 开方函数
void  IQSin_Cos_Cale(p_IQSin_Cos pV); //求取正余弦函数 
void  IQAtan_Cale(p_IQAtan  pV) ;  //求取求反正弦函数
int32_t IQsat( int32_t Uint,int32_t  U_max, int32_t U_min); //限制赋值函数
#endif /* __IQ_math_H */

//===========================================================================
// No more.
//===========================================================================
