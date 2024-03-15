/*
 * uart.h
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>
#include <math.h>
#include <stm32f4xx.h>
#include "circular_buffer.h"

#define UART_Port GPIOA
#define TX_Pin    2     //GPIO_PIN_2
#define RX_PIN    3     //GPIO_PIN_3

#define UART_Tim TIM11
#define UART_TIME 10 // ms

#define BAUDRATE 115200
#define BUFFER_SIZE ( 128 )
#define START 0xFA
#define STOP  0xFB

#define ODOM_TRANSMIT 0x4F
#define TOF_TRANSMIT  0x54
#define SPEED_RECEIVE 0x53
#define INIT_RECEIVE 0x49

extern circular_buff in_buf;
extern circular_buff out_buf;

void USART2_Init(void);
void USART2_IRQHandler(void);
void UART_Interrupt_Init();


void Send_Byte(uint8_t data);
void Send_Buffer(void);
void Send_Command(uint8_t code, float value[], uint8_t len);
uint8_t* Read_Buffer();

#endif /* INC_UART_H_ */

