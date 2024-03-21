/*
 * struct_types.h
 *
 *  Created on: Mar 15, 2024
 *      Author: xenia
 */

#ifndef INC_STRUCT_TYPES_H_
#define INC_STRUCT_TYPES_H_

#include <stm32f4xx.h>


// Structure for representing output pins
typedef struct {
	GPIO_TypeDef* port;
	uint8_t pin;
} sOutput_t;

// Structure for linking left and right
// motors with their respected timers
typedef struct {
	TIM_TypeDef* tim;
} sTimer_t;

// Structure for representing odometry data
typedef struct {
	float x;
	float y;
	float theta;
	float left_speed;
	float right_speed;
} sOdom_t;

// Structure for representing a motor
// and it's PID values
typedef struct sMotor_t {
	float target_speed;
	float current_speed;
	float control_PWM;
	float errors[3];
	float Kp;
	float Ki;
	float Kd;
	void (*Compute_PID)(struct sMotor_t *self);
} sMotor_t;

// Needed for the above structure to be defined correctly
void Compute_PID(sMotor_t *self);

#endif /* INC_STRUCT_TYPES_H_ */
