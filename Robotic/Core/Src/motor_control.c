/*
 * motor_control.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "motor_control.h"

unsigned int left_PWM;
unsigned int right_PWM;
float target_left_RPM;
float target_right_RPM;

uint32_t interrupt_counter = 0;

void Motors_Init()
{
	// TIM2 CH1 and CH2 configuration as PWM output

	// Enabe GPIOA (PWM port) clock source
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN );

	// Enable TIM2 (PWM source) clock source - 84Mhz
	RCC->APB1ENR |= ( RCC_APB1ENR_TIM2EN );

	// Set TIM2 channel pins PA0 (CH1) and PA1 (CH2)
	// to alternate function mode AF1
	PWM_Port->MODER &= ~(0b11 << 2*PWMA_Pin | 0b11 << 2*PWMB_Pin);
	PWM_Port->MODER |=  (0b10 << 2*PWMA_Pin | 0b10 << 2*PWMB_Pin);

	PWM_Port->AFR[0] &= ~(0b1111 << 4*PWMA_Pin | 0b1111 << 4*PWMB_Pin);
	PWM_Port->AFR[0] |=  (0b0001 << 4*PWMA_Pin | 0b0001 << 4*PWMB_Pin);

	// Autoreload value determines the frequency of
	// the PWM signal - 84MHz -> 20kHz
	PWM_Tim->PSC = 1 - 1;
	PWM_Tim->ARR = PWM_ARR;

	// PWM mode 1 - channel is active as long as
	// TIM2->CNT < TIM2->CCR1 - during the timer period
	PWM_Tim->CCMR1 &= ~(0b111 << TIM_CCMR1_OC1M_Pos | 0b111 << TIM_CCMR1_OC2M_Pos);
	PWM_Tim->CCMR1 |=  (0b110 << TIM_CCMR1_OC1M_Pos | 0b110 << TIM_CCMR1_OC2M_Pos);

	// Enable preload for TIM2
	PWM_Tim->CCMR1 |= (0b1 << TIM_CCMR1_OC1PE_Pos | 0b1 << TIM_CCMR1_OC2PE_Pos);

	// Enable autoreload for TIM2
	PWM_Tim->CR1 |= (0b1 << TIM_CR1_ARPE_Pos);

	// Enable automatic update of registers
	PWM_Tim->EGR |= (0b1 << TIM_EGR_UG_Pos);

	// Enable capture/compare as output
	// (PWM output to driver) for CH1 and CH2
	// Default falling edge mode
	PWM_Tim->CCER |= (0b1 << TIM_CCER_CC1E_Pos | 0b1 << TIM_CCER_CC2E_Pos);

	// Enable PWM generation in default edge aligned mode
	PWM_Tim->CR1 |= (0b1 << TIM_CR1_CEN_Pos);


	// Driver direction pins configuration

	// Enable GPIOB and GPIOC clock source
	// (GPIOA is enabled above)
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOBEN );
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOCEN );

	// Configure pins for driver direction as
	// general purpose output pins
	BI2_Port->MODER &= ~(0b11 << 2*BI2_Pin);
	BI2_Port->MODER |=  (0b01 << 2*BI2_Pin);

	BI1_Port->MODER &= ~(0b11 << 2*BI1_Pin);
	BI1_Port->MODER |=  (0b01 << 2*BI1_Pin);

	AI1_Port->MODER &= ~(0b11 << 2*AI1_Pin);
	AI1_Port->MODER |=  (0b01 << 2*AI1_Pin);

	AI2_Port->MODER &= ~(0b11 << 2*AI2_Pin);
	AI2_Port->MODER |=  (0b01 << 2*AI2_Pin);

	// Configure pins for driver direction as
	// output push pull (reset state)
	BI2_Port->OTYPER &=  ~(0b1 << BI2_Pin);

	BI1_Port->OTYPER &=  ~(0b1 << BI1_Pin);

	AI1_Port->OTYPER &=  ~(0b1 << AI1_Pin);

	AI2_Port->OTYPER &=  ~(0b1 << AI2_Pin);
}

void PID_Odom_Interrupt_Init()
{
	// Lower priority than UART but higher than everything else in pre-emtion group 1
	uint32_t tim10_pri_encoding = NVIC_EncodePriority(0, 1, 2);

	// Clock source for TIM10
	RCC->APB2ENR |= ( RCC_APB2ENR_TIM10EN );

	// Setup the NVIC to enable interrupts.
	NVIC_SetPriorityGrouping( 0 );
	// UART receive interrupts should be high priority.
	NVIC_SetPriority( TIM1_UP_TIM10_IRQn, tim10_pri_encoding );
	NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn );

	// Clock setup for TIM10, 10ms
	PID_ODOM_Tim->PSC = 11;
	PID_ODOM_Tim->ARR = 60000 - 1;

	// Enable immediate update of register on counter
	PID_ODOM_Tim->EGR |= ( TIM_EGR_UG );

	// Enable interrupts for TIM10
	PID_ODOM_Tim->DIER |= ( TIM_DIER_UIE );

	// Enable counter for TIM10
	PID_ODOM_Tim->CR1 |= ( TIM_CR1_CEN );
}

void TIM1_UP_TIM10_IRQHandler(void)
{
//	// Disable counter for TIM10
//	TIM10->CR1 &= ~( TIM_CR1_CEN );

	interrupt_counter++;

	// Calculates new position and orientation based on encoder output
	// and sends odometry data via UART
	if( interrupt_counter % ODOM_TIME == 0){
		Send_Byte('2');
//		Read_Encoders();
//
//		odom[0] = x;
//		odom[1] = y;
//		odom[2] = theta;
//		Send_Command(ODOM_TRANSMIT, odom, sizeof(odom));
	}

	// Calculates speed loop PID and sends the output to the driver
	if( interrupt_counter % PID_TIME == 0){
		Send_Byte('4');
		// Speed_Loop();
	}

//	// Enable counter for TIM10
//	TIM10->CR1 |= ( TIM_CR1_CEN );

}

void Set_Motor_Speed(float left, float right)
{
	target_left_RPM = left;
	target_right_RPM = right;
}

void Speed_Loop()
{
	// TODO PID

	Set_Motor_Direction(left_PWM, right_PWM);
	Set_Motor_PWM(abs(left_PWM), abs(right_PWM));
}

void Set_Motor_Direction(int left, int right)
{
	if (left < 0){ // counter clockwise
		Set_Pin(AI1_Port, AI1_Pin, 0);
		Set_Pin(AI2_Port, AI2_Pin, 1);
	}
	else{ // clockwise
		Set_Pin(AI1_Port, AI1_Pin, 1);
		Set_Pin(AI2_Port, AI2_Pin, 0);
	}

	if (right < 0){ // counter clockwise
		Set_Pin(BI1_Port, BI1_Pin, 0);
		Set_Pin(BI2_Port, BI2_Pin, 1);
	}
	else{ // clockwise
		Set_Pin(BI1_Port, BI1_Pin, 1);
		Set_Pin(BI2_Port, BI2_Pin, 0);
	}
}

void Set_Motor_PWM(unsigned int left, unsigned int right)
{
	// CCR1 value determines the duty cycle of the
	// PWM signal, min - 0, max - PWM_ARR
	if (left > PWM_ARR) left = PWM_ARR;
	if (right > PWM_ARR) right = PWM_ARR;

	// TODO check which motor is which
	PWM_Tim->CCR1 = left;
	PWM_Tim->CCR2 = right;
}

void Set_Pin(GPIO_TypeDef* port, int pin, int value){
	if (value > 0) port->BSRR |= (0b1 << pin);
	else port->BSRR |= (0b1 << (pin + 16));
}
