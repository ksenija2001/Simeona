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
#include "struct_types.h"
#include <stdlib.h>
#include <stm32f4xx.h>

#define PWM_ARR  4199 // max value for PWM defined by used timer
#define PID_TIME 40  // ms

void PID_Odom_Interrupt_Init();
void Motors_Init();
void Set_Motor_Speed(float left, float right);
void Compute_PID(sMotor_t *self);
void Set_Motor_PWM(unsigned int left, unsigned int right);
void Set_Motor_Direction(int left, int right);
void Set_Pin(sOutput_t output, int value);

#endif /* INC_MOTOR_CONTROL_H_ */
