/*
 * motor_control.h
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "odom.h"
#include "uart.h"
#include <stdlib.h>
#include <stm32f4xx.h>

#define BI2_Port GPIOB
#define BI2_Pin  10 // GPIO_PIN_10
#define BI1_Port GPIOC
#define BI1_Pin  7 // GPIO_PIN_7
#define AI1_Port GPIOB
#define AI1_Pin  6 // GPIO_PIN_6
#define AI2_Port GPIOA
#define AI2_Pin  5 // GPIO_PIN_5

#define PWM_Tim  TIM2
#define PWM_Port GPIOA
#define PWMA_Pin 0 // GPIO_PIN_0, TIM2 CH1
#define PWMB_Pin 1 // GPIO_PIN_1, TIM2 CH2
#define PWM_ARR  4199

#define PID_ODOM_Tim  TIM10
#define PID_TIME 40  // ms

void PID_Odom_Interrupt_Init();
void Motors_Init();
void Set_Motor_Speed(float left, float right);
void Speed_Loop();
void Set_Motor_PWM(unsigned int left, unsigned int right);
void Set_Motor_Direction(int left, int right);
void Set_Pin(GPIO_TypeDef* port, int pin, int value);

#endif /* INC_MOTOR_CONTROL_H_ */
