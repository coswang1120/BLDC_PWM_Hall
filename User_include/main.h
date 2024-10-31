#ifndef __MAIN_H
#define __MAIN_H
#include "stm32f10x.h"
#include <stdio.h>  
int fputc( int ch , FILE *f);

int fputc( int ch , FILE *f)  // print function for RS232
{
    USART_SendData( USART3 , (u8)ch ) ;
    while(USART_GetFlagStatus(USART3 , USART_FLAG_TC) == RESET)
    {
    }
    return ch;
}


#endif /* __MAIN_H */
