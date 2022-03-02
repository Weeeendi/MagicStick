/**
 * @file Uart.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef UART_H
#define UART_H

/*****************************************************************************
 @brief USART1_Init
 @param USART1 baudrate
*****************************************************************************/
uint8_t USART1_Init(uint32_t BaudRate);

/*****************************************************************************
 @brief USART3_Init
 @param USART3 baudrate
*****************************************************************************/
uint8_t USART3_Init(uint32_t BaudRate);

/*****************************************************************************
 @brief Uart_PutChar
 @param output byte
*****************************************************************************/
void Uart_PutChar(uint8_t ch);

#endif
