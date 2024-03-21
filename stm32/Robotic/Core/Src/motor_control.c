/*
 * motor_control.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "motor_control.h"

float q0;
float q1;
float q2;

volatile uint32_t interrupt_counter = 0;
float send_odom[3];

sOutput_t BI2  = { .port = GPIOB, .pin = 10 };
sOutput_t BI1  = { .port = GPIOC, .pin = 7 };
sOutput_t AI2  = { .port = GPIOA, .pin = 5};
sOutput_t AI1  = { .port = GPIOB, .pin = 6};
sOutput_t PWMA = { .port = GPIOA, .pin = 0};
sOutput_t PWMB = { .port = GPIOA, .pin = 1};

sMotor_t left_motor = {
		.target_speed = 0,
		.current_speed = 0,
		.control_PWM = 0,
		.errors = {0, 0, 0},
		.Kp = 5,  // 3
		.Ki = 0.01,
		.Kd = 0.01,
};
sMotor_t right_motor= {
		.target_speed = 0,
		.current_speed = 0,
		.control_PWM = 0,
		.errors = {0, 0, 0},
		.Kp = 5,  // 3
		.Ki = 0.01,
		.Kd = 0.01,
};

// TIM2 CH1 and CH2 configuration as PWM output
// Driver direction pins configuration
void Motors_Init()
{
	left_motor.Compute_PID = Compute_PID;
	right_motor.Compute_PID = Compute_PID;

	// Enabe GPIOA (PWM port), GPIOB and GPIOC clock source
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN );
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOBEN );
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOCEN );

	// Enable TIM2 (PWM source) clock source - 84Mhz
	RCC->APB1ENR |= ( RCC_APB1ENR_TIM2EN );

	// Set PWM source channel pins PA0 (CH1) and PA1 (CH2)
	// to alternate function mode AF1
	PWMA.port->MODER &= ~(0b11 << 2*PWMA.pin);
	PWMA.port->MODER |=  (0b10 << 2*PWMA.pin);
	PWMB.port->MODER &= ~(0b11 << 2*PWMB.pin);
	PWMB.port->MODER |=  (0b10 << 2*PWMB.pin);

	PWMA.port->AFR[0] &= ~(0b1111 << 4*PWMA.pin);
	PWMA.port->AFR[0] |=  (0b0001 << 4*PWMA.pin);
	PWMB.port->AFR[0] &= ~(0b1111 << 4*PWMB.pin);
	PWMB.port->AFR[0] |=  (0b0001 << 4*PWMB.pin);

	// Autoreload value determines the frequency of
	// the PWM signal - 84MHz -> 20kHz
	TIM2->PSC = 1 - 1;
	TIM2->ARR = PWM_ARR;

	// PWM mode 1 - channel is active as long as
	// TIM2->CNT < TIM2->CCR1 - during the timer period
	TIM2->CCMR1 &= ~(0b111 << TIM_CCMR1_OC1M_Pos | 0b111 << TIM_CCMR1_OC2M_Pos);
	TIM2->CCMR1 |=  (0b110 << TIM_CCMR1_OC1M_Pos | 0b110 << TIM_CCMR1_OC2M_Pos);

	// Enable preload for TIM2
	TIM2->CCMR1 |= (0b1 << TIM_CCMR1_OC1PE_Pos | 0b1 << TIM_CCMR1_OC2PE_Pos);

	// Enable autoreload for TIM2
	TIM2->CR1 |= (0b1 << TIM_CR1_ARPE_Pos);

	// Enable automatic update of registers
	TIM2->EGR |= (0b1 << TIM_EGR_UG_Pos);

	// Enable capture/compare as output
	// (PWM output to driver) for CH1 and CH2
	// Default falling edge mode
	TIM2->CCER |= (0b1 << TIM_CCER_CC1E_Pos | 0b1 << TIM_CCER_CC2E_Pos);

	// Enable PWM generation in default edge aligned mode
	TIM2->CR1 |= (0b1 << TIM_CR1_CEN_Pos);



	// Configure pins for driver direction as
	// general purpose output pins
	BI2.port->MODER &= ~(0b11 << 2*BI2.pin);
	BI2.port->MODER |=  (0b01 << 2*BI2.pin);

	BI1.port->MODER &= ~(0b11 << 2*BI1.pin);
	BI1.port->MODER |=  (0b01 << 2*BI1.pin);

	AI1.port->MODER &= ~(0b11 << 2*AI1.pin);
	AI1.port->MODER |=  (0b01 << 2*AI1.pin);

	AI2.port->MODER &= ~(0b11 << 2*AI2.pin);
	AI2.port->MODER |=  (0b01 << 2*AI2.pin);

	// Configure pins for driver direction as
	// output push pull (reset state)
	BI2.port->OTYPER &=  ~(0b1 << BI2.pin);
	BI1.port->OTYPER &=  ~(0b1 << BI1.pin);
	AI1.port->OTYPER &=  ~(0b1 << AI1.pin);
	AI2.port->OTYPER &=  ~(0b1 << AI2.pin);
}

// Configures timer TIM11 with interrupt every 10ms which executes
// PID computation and updates odometry data
void PID_Odom_Interrupt_Init()
{
	// Lower priority than UART IRQHandler and UART buffer checking
	uint32_t tim10_pri_encoding = NVIC_EncodePriority(0, 1, 2);

	// Clock source for TIM10
	RCC->APB2ENR |= ( RCC_APB2ENR_TIM10EN );

	// Clock setup for TIM10, 10ms
	TIM10->PSC = 13 - 1;
	TIM10->ARR = 60000 - 1;

	// Enable immediate update of register on counter
	TIM10->EGR |= ( TIM_EGR_UG );

	// Enable interrupts for TIM10
	TIM10->DIER |= ( TIM_DIER_UIE );

	// Enable counter for TIM10
	TIM10->CR1 |= ( TIM_CR1_CEN );

	// Setup the NVIC to enable interrupts.
	NVIC_SetPriorityGrouping( 0 );
	NVIC_SetPriority( TIM1_UP_TIM10_IRQn, tim10_pri_encoding );
	NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn );
}

void TIM1_UP_TIM10_IRQHandler(void)
{;
	interrupt_counter += 10;

	// Calculates new position and orientation based on encoder output
	// and sends odometry data via UART
	if( interrupt_counter % ODOM_TIME == 0 ){
		//Send_Byte('2');
		sOdom_t* odom = Read_Encoders();

		send_odom[0] = odom->x;
		send_odom[1] = odom->y;
		send_odom[2] = odom->theta;

		left_motor.current_speed = odom->left_speed;
		right_motor.current_speed = odom->right_speed;

		Send_Command(ODOM_TRANSMIT, send_odom, sizeof(send_odom));
	}

	// Calculates speed loop PID and sends the output to the driver
	if( interrupt_counter % PID_TIME == 0 ){
		//Send_Byte('4');
		left_motor.Compute_PID(&left_motor);
		right_motor.Compute_PID(&right_motor);

		Set_Motor_Direction(left_motor.control_PWM, right_motor.control_PWM);
		Set_Motor_PWM(abs(left_motor.control_PWM), abs(right_motor.control_PWM));
	}

	// Clears interrupt flag so that other interrupts can work
	TIM10->SR &= ~TIM_SR_UIF;
}

// Sets target speed for speed loop
void Set_Motor_Speed(float left, float right)
{
	left_motor.target_speed = left;
	right_motor.target_speed = right;
}

// Updates PWM control for reaching and holding target speed
void Compute_PID(sMotor_t *self){
	self->errors[2] = self->errors[1];
	self->errors[1] = self->errors[0];
	self->errors[0] = self->target_speed - self->current_speed;

	q0 = self->Kp + self->Kd/PID_TIME;
	q1 = self->Ki*PID_TIME - 2*self->Kd/PID_TIME - self->Kp;
	q2 = - self->Kd/PID_TIME;

	self->control_PWM += q0 * self->errors[0] + q1 * self->errors[1] + q2 * self->errors[2];
}

// Changes motor direction based on sign of received speeds
void Set_Motor_Direction(int left, int right)
{
	if (left > 0){ // counter clockwise
		Set_Pin(AI1, 0);
		Set_Pin(AI2, 1);
	}
	else{ // clockwise
		Set_Pin(AI1, 1);
		Set_Pin(AI2, 0);
	}

	if (right > 0){ // counter clockwise
		Set_Pin(BI1, 0);
		Set_Pin(BI2, 1);
	}
	else{ // clockwise
		Set_Pin(BI1, 1);
		Set_Pin(BI2, 0);
	}
}

// Sets motor PWM value caclulated in speed loop
void Set_Motor_PWM(unsigned int left, unsigned int right)
{
	// CCR1 value determines the duty cycle of the
	// PWM signal, min - 0, max - PWM_ARR
	if (left > PWM_ARR) left = PWM_ARR;
	if (right > PWM_ARR) right = PWM_ARR;

	// TODO check which motor is which
	TIM2->CCR1 = left;
	TIM2->CCR2 = right;
}

// Sets or resets a pin
void Set_Pin(sOutput_t output, int value){
	if (value > 0) output.port->BSRR |= (0b1 << output.pin);
	else output.port->BSRR |= (0b1 << (output.pin + 16));
}
