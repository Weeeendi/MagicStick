/**
 * @file UART.c
 * @author your name (you@domain.com)
 * @brief 管理串口设置
 * @version 0.1
 * @date 2022-02-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "include.h"
#include "mcu_api.h"

//#define DEBUG_IN_USART1

/**
 * @brief 初始化UART1
 * 
 * @param BaudRate 
 * @return uint8_t 
 */
uint8_t USART1_Init(uint32_t BaudRate)
{
  USART_InitTypeDef USART_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  if(BaudRate > 115200) 
  {                                            //波特率太高返回失败
    return(ERROR);
  }
  
  // Enable GPIOA clocks 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  // Enable USART1 clocks 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  /*
  *  USART1_TX -> PA9 , USART1_RX -> PA10
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
  //
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
  //
  USART_ClearFlag(USART1,USART_FLAG_TC);
  
  //Enable the USART1 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                             //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                     //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                            //响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                               //中断使能
  NVIC_Init(&NVIC_InitStructure);                                               //初始化中断
  
  USART_Cmd(USART1, ENABLE);
  
  return(SUCCESS);
}

/*****************************************************************************
UART3 Initilization
*****************************************************************************/
uint8_t USART3_Init(uint32_t BaudRate)
{
  USART_InitTypeDef USART_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  if(BaudRate > 115200) 
  {                                           //波特率太高返回失败
    return(ERROR);
  }
  
  // Enable GPIOA clocks 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // Enable USART1 clocks 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  /*
  *  USART3_TX -> PB10 , USART13_RX -> PB11
  */				
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);		   
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //
  USART_InitStructure.USART_BaudRate = BaudRate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure); 
  //
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  //
  USART_ClearFlag(USART3,USART_FLAG_TC);
  
  //Enable the USART3 Interrupt
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;                             //指定中断源
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                     //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                            //响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                               //中断使能
  NVIC_Init(&NVIC_InitStructure);                                               //初始化中断
  
  USART_Cmd(USART3, ENABLE);
  
  return(SUCCESS);
}
extern uint8_t Timer4_Value;
uint16_t uart_input_count;
uint8_t uart_input_finish;



/*****************************************************************************
* 中断响应函数: USART1_IRQHandler
*****************************************************************************/
void USART1_IRQHandler(void)
{
  uint8_t ch;
  
  //中断标志判断
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    ch = USART_ReceiveData(USART1);
    
#ifdef DEBUG_IN_USART1
		
    uart_receive_input(ch);
#endif
  }
}

/*****************************************************************************
 USART3_IRQHandler
*****************************************************************************/
void USART3_IRQHandler(void)
{
  uint8_t ch;
  
  //received
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    ch = USART_ReceiveData(USART3);

#ifndef DEBUG_IN_USART1
    //uart_receive_input
    uart_receive_input(ch);                                                      //中断处理函数
#endif
  }
}
/*****************************************************************************
function : Uart_PutChar
*****************************************************************************/
void Uart_PutChar(uint8_t ch)
{
#ifdef DEBUG_IN_USART1
  //send
  USART_SendData(USART1,ch);
  //
  while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
#else
  //send
  USART_SendData(USART3,ch);
  //
  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
#endif
}
