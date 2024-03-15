/*
 * odom.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "odom.h"

int32_t last_left_enc  = 0; // left increments
int32_t last_right_enc = 0; // right increments
int32_t curr_left_enc  = 0;
int32_t curr_right_enc = 0;

short delta_left  = 0;
short delta_right = 0;
float delta_distance = 0;
float delta_theta    = 0;

volatile float x = 0;
volatile float y = 0;
volatile float theta = M_PI_2;
volatile float left_speed = 0;
volatile float right_speed = 0;


void Encoders_Init(){

	// Enable clock for portTIM3 A
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN );

	// Clock source for TIM3
	RCC->APB1ENR |= ( RCC_APB1ENR_TIM3EN );

	// Clock source for TIM1
	RCC->APB2ENR |= ( RCC_APB2ENR_TIM1EN );

	// Enable alternate function for PA6-9
	ENC_Port->MODER &= ~(0b11 << 2*ENC_LEFTA_Pin | 0b11 << 2*ENC_LEFTB_Pin | \
						0b11 << 2*ENC_RIGHTA_Pin | 0b11 << 2*ENC_RIGHTB_Pin);
	ENC_Port->MODER |=  (0b10 << 2*ENC_LEFTA_Pin | 0b10 << 2*ENC_LEFTB_Pin | \
						0b10 << 2*ENC_RIGHTA_Pin | 0b10 << 2*ENC_RIGHTB_Pin);

	// Set AF2 for PA6 and PA7
	ENC_Port->AFR[0] &= ~(0b1111 << 4*ENC_LEFTA_Pin | 0b1111 << 4*ENC_LEFTB_Pin);
	ENC_Port->AFR[0] |=  (0b0010 << 4*ENC_LEFTA_Pin | 0b0010 << 4*ENC_LEFTB_Pin);

	// Set AF1 for PA8 and PA9
	// AFRH is a separate register, values are linked to pins
	// 8-15
	ENC_Port->AFR[1] &= ~(0b1111 << 4*(ENC_RIGHTA_Pin - 8) | 0b1111 << 4*(ENC_RIGHTB_Pin - 8));
	ENC_Port->AFR[1] |=  (0b0001 << 4*(ENC_RIGHTA_Pin - 8) | 0b0001 << 4*(ENC_RIGHTB_Pin - 8));

	// Clock setup for TIM1
	ENC_RIGHT_Tim->PSC = 0;
	ENC_RIGHT_Tim->ARR = 0xFFFF - 1;

	// Clock setup for TIM3
	ENC_LEFT_Tim->PSC = 0;
	ENC_LEFT_Tim->ARR = 0xFFFF - 1;

	// Enable encoder quadrature mode for TIM1 and TIM3
	ENC_RIGHT_Tim->SMCR &= ~(0b111 << 3*0);
	ENC_RIGHT_Tim->SMCR |=  (0b011 << 3*0);

	ENC_LEFT_Tim->SMCR &= ~(0b111 << 3*0);
	ENC_LEFT_Tim->SMCR |=  (0b011 << 3*0);

	// Set counting direction for TIM1 and TIM3
	ENC_RIGHT_Tim->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	ENC_RIGHT_Tim->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	ENC_LEFT_Tim->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	ENC_LEFT_Tim->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	// Enable counter for TIM1 and TIM3
	ENC_RIGHT_Tim->CR1 |= (1 << 0);
	ENC_LEFT_Tim->CR1 |= (1 << 0);

	// Enable immediate update of register on counter
	ENC_RIGHT_Tim->EGR |= (1 << 0);
	ENC_LEFT_Tim->EGR |= (1 << 0);
}

void Read_Encoders(){
	curr_left_enc  = ENC_LEFT_Tim->CNT;
	curr_right_enc = ENC_RIGHT_Tim->CNT;

	delta_left  = (curr_left_enc  - last_left_enc)  * INC_MM;
	delta_right = (curr_right_enc - last_right_enc) * INC_MM;

	delta_distance = (delta_left + delta_right) / 2;
	delta_theta    = (delta_left - delta_right) / WHEEL_DISTANCE;

	x += delta_distance * cos(theta + delta_theta/2);
	y += delta_distance * sin(theta + delta_theta/2);
	theta += delta_theta;

	left_speed  = delta_left  / ODOM_TIME; // meters per second
	right_speed = delta_right / ODOM_TIME; // meters per second

	last_left_enc  = curr_left_enc;
	last_right_enc = curr_right_enc;
}

void Reset_Encoders(float odom[]){
	last_left_enc  = 0;
	last_right_enc = 0;
	curr_left_enc  = 0;
	curr_right_enc = 0;

	 x     = odom[0];
	 y     = odom[1];
	 theta = odom[2];
	 left_speed  = 0;
	 right_speed = 0;
}

