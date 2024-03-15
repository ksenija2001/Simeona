/*
 * odom.h
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#ifndef INC_ODOM_H_
#define INC_ODOM_H_

#include <math.h>
#include <stm32f4xx.h>

#define ENC_Port        GPIOA
#define ENC_LEFT_Tim    TIM3
#define ENC_RIGHT_Tim   TIM1
#define ENC_LEFTA_Pin  6   // TIM3 Ch1
#define ENC_LEFTB_Pin  7   // TIM3 CH2
#define ENC_RIGHTA_Pin 8  // TIM1 CH1
#define ENC_RIGHTB_Pin 9  // TIM1 CH2

#define PPR            2184 	   //4*546 inc
#define WHEEL_DIAMETER 65
#define WHEEL_DISTANCE 150
#define INC_MM         0.09349978 // (WHEEL_DIAMETER*PI)/PPR
#define INC_RAD        0.00062333 // INC_MM/WHEEL_DISTANCE
#define ODOM_TIME      20 //ms

void Encoders_Init();
void Read_Encoders();
void Reset_Encoders(float odom[]);

extern volatile float left_speed;
extern volatile float right_speed;

extern volatile float x;
extern volatile float y;
extern volatile float theta;

#endif /* INC_ODOM_H_ */
