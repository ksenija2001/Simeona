/*
 * odom.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "odom.h"

sOutput_t LEFTA   = { .port = GPIOA, .pin = 6 };
sOutput_t LEFTB   = { .port = GPIOA, .pin = 7 };
sOutput_t RIGHTA  = { .port = GPIOA, .pin = 8 };
sOutput_t RIGHTB  = { .port = GPIOA, .pin = 9 };

sTimer_t LEFT  = { .tim = TIM1 };
sTimer_t RIGHT = { .tim = TIM3 };

sOdom_t odom = {
		.x = 0,
		.y = 0,
		.theta = M_PI/2,
		.left_speed = 0,
		.right_speed = 0
};

int32_t last_left_enc  = 0; // left increments
int32_t last_right_enc = 0; // right increments
int32_t curr_left_enc  = 0;
int32_t curr_right_enc = 0;

float delta_left  = 0;
float delta_right = 0;
float delta_distance = 0;
float delta_theta    = 0;

float wheel_diameter = 70;
float wheel_distance = 166.42;
float inc_mm = 1;
float inc_rad = 1;

void Encoders_Init(){

	// Enable clock for GPIOA (encoder output)
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN );

	// Clock source for TIM3
	RCC->APB1ENR |= ( RCC_APB1ENR_TIM3EN );

	// Clock source for TIM1
	RCC->APB2ENR |= ( RCC_APB2ENR_TIM1EN );

	// Enable alternate function for PA6-9
	GPIOA->MODER &= ~(0b11 << 2*LEFTA.pin | 0b11 << 2*LEFTB.pin | \
					0b11 << 2*RIGHTA.pin | 0b11 << 2*RIGHTB.pin);
	GPIOA->MODER |=  (0b10 << 2*LEFTA.pin | 0b10 << 2*LEFTB.pin | \
					0b10 << 2*RIGHTA.pin | 0b10 << 2*RIGHTB.pin);

	// Set AF2 for PA6 and PA7
	GPIOA->AFR[0] &= ~(0b1111 << 4*LEFTA.pin | 0b1111 << 4*LEFTB.pin);
	GPIOA->AFR[0] |=  (0b0010 << 4*LEFTA.pin | 0b0010 << 4*LEFTB.pin);

	// Set AF1 for PA8 and PA9
	// AFRH is a separate register, values are linked to pins 8-15
	GPIOA->AFR[1] &= ~(0b1111 << 4*(RIGHTA.pin - 8) | 0b1111 << 4*(RIGHTB.pin - 8));
	GPIOA->AFR[1] |=  (0b0001 << 4*(RIGHTA.pin - 8) | 0b0001 << 4*(RIGHTB.pin - 8));

	// Clock setup for TIM1
	RIGHT.tim->PSC = 0;
	RIGHT.tim->ARR = 0xFFFF - 1;

	// Clock setup for TIM3
	LEFT.tim->PSC = 0;
	LEFT.tim->ARR = 0xFFFF - 1;

	// Enable encoder quadrature mode for TIM1 and TIM3
	RIGHT.tim->SMCR &= ~(0b111 << 3*0);
	RIGHT.tim->SMCR |=  (0b011 << 3*0);

	LEFT.tim->SMCR &= ~(0b111 << 3*0);
	LEFT.tim->SMCR |=  (0b011 << 3*0);

	// Set counting direction for TIM1 and TIM3
	RIGHT.tim->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	RIGHT.tim->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	LEFT.tim->CCMR1 &= ~(0b11 << 0 | 0b11 << 8);
	LEFT.tim->CCMR1 |=  (0b01 << 0 | 0b01 << 8);

	// Enable counter for TIM1 and TIM3
	RIGHT.tim->CR1 |= (1 << 0);
	LEFT.tim->CR1 |= (1 << 0);

	// Enable immediate update of register on counter
	RIGHT.tim->EGR |= (1 << 0);
	LEFT.tim->EGR |= (1 << 0);

	inc_mm = (wheel_diameter*M_PI)/PPR;
	inc_rad = inc_mm/wheel_distance;
}

// Calculates current position and speeds based on encoder increment readings
sOdom_t* Read_Encoders(){
	last_left_enc  = curr_left_enc;
	last_right_enc = curr_right_enc;

	curr_left_enc  = LEFT.tim->CNT;
	curr_right_enc = RIGHT.tim->CNT;

	// The delta is calulated from increments from current and last encoder readings and converted to mm
	// The cast to int16_t ensures that a jump from 0 to 65535 and vice versa won't happen
	delta_left  = (int16_t)(curr_left_enc  - last_left_enc)  * inc_mm;
	delta_right = (int16_t)(curr_right_enc - last_right_enc) * inc_mm;

	// Distance traveled from last encoder reading
	delta_distance = (delta_left + delta_right) / 2;
	// Change in orientation from last encoder reading
	delta_theta    = (delta_left - delta_right) / wheel_distance;

	// Updated odom data
	odom.x += delta_distance * cos(odom.theta + delta_theta/2);
	odom.y += delta_distance * sin(odom.theta + delta_theta/2);
	odom.theta += delta_theta;
	odom.left_speed  = delta_left / ODOM_TIME * 1000; // mm/s
	odom.right_speed = delta_right / ODOM_TIME * 1000; // mm/s

	return &odom;
}

// Resets or initialized odometry data based on input parameter
void Reset_Encoders(sOdom_t* new_odom){
	last_left_enc  = 0;
	last_right_enc = 0;
	curr_left_enc  = 0;
	curr_right_enc = 0;

	odom.x = new_odom->x;
	odom.y = new_odom->y;
	odom.theta = new_odom->theta;
	odom.left_speed = 0;
	odom.right_speed = 0;
}

void Config(float diameter, float distance){
	wheel_diameter = diameter;
	wheel_distance = distance;

	inc_mm = (wheel_diameter*M_PI)/PPR;
	inc_rad = inc_mm/wheel_distance;
}

