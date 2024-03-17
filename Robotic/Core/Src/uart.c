/*
 * uart.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "uart.h"

volatile uint32_t uart_interrupt_counter = 0;

uint8_t in_buffer[BUFFER_SIZE + 1];
uint8_t out_buffer[BUFFER_SIZE + 1];

circular_buff in_buf = {
		len    : BUFFER_SIZE,
		buffer : in_buffer,
		head   : 0,
		tail   : 0
};

circular_buff out_buf = {
		len    : BUFFER_SIZE,
		buffer : out_buffer,
		head   : 0,
		tail   : 0
};

uint8_t first = 0;
uint8_t len   = 0;
uint8_t last  = 0;

uint8_t size  = 0;

uint8_t send_data = 0;
uint8_t recv_data[13];

uint8_t c = 0;
uint8_t null = '\0';

union U_F{
	float f;
	uint8_t u[4];
}convert_float;

// Initializes USART2 over pins PA2 and PA3 with interrupt handler
void USART2_Init()
{
	uint32_t uartdiv = SystemCoreClock/2 / BAUDRATE;
    uint32_t uart_pri_encoding = NVIC_EncodePriority( 0, 1, 0 );

	// Enable clock for port A
	RCC->AHB1ENR |= ( RCC_AHB1ENR_GPIOAEN );

	// Clock source for USART2
	RCC->APB1ENR |= ( RCC_APB1ENR_USART2EN );

	// PA2-3 AF7
	UART_Port->MODER &= ~(0b11 << 2*TX_Pin | 0b11 << 2*RX_PIN);
	UART_Port->MODER |=  (0b10 << 2*TX_Pin | 0b10 << 2*RX_PIN);

	UART_Port->AFR[0] &= ~(0b1111 << 4*TX_Pin | 0b1111 << 4*RX_PIN);
	UART_Port->AFR[0] |=  (0b0111 << 4*TX_Pin | 0b0111 << 4*RX_PIN);

    // Setup the NVIC to enable interrupts.
    NVIC_SetPriorityGrouping( 0 );
    // UART receive interrupts should be high priority.
    NVIC_SetPriority( USART2_IRQn, uart_pri_encoding );
    NVIC_EnableIRQ( USART2_IRQn );

    // Set BaudRate to 115200
    USART2->BRR = ((uartdiv/16) << USART_BRR_DIV_Mantissa_Pos |
    			   (uartdiv%16) << USART_BRR_DIV_Fraction_Pos);

    USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE);
}

// Interrupt handler for receiving data over UART
void USART2_IRQHandler(void)
{
	if (USART2->SR & USART_SR_RXNE) {
	  c = USART2->DR;
	  buffer_write(&in_buf, c);
	}
}

// Configures timer TIM10 with interrupt every 1ms which check input buffer
void UART_Interrupt_Init()
{
	// Lower priority than UART IRQHandler, higher priority than PID
	uint32_t tim11_pri_encoding = NVIC_EncodePriority(0, 1, 1);

	// Clock source for TIM11
	RCC->APB2ENR |= ( RCC_APB2ENR_TIM11EN );

	// Clock setup for TIM11, 1ms
	UART_Tim->PSC = 1;
	UART_Tim->ARR = 42000 - 1;

	// Enable immediate update of register on counter
	UART_Tim->EGR |= ( TIM_EGR_UG );

	// Enable interrupts for TIM11
	UART_Tim->DIER |= ( TIM_DIER_UIE );

	// Enable counter for TIM11
	UART_Tim->CR1 |= ( TIM_CR1_CEN );

	// Enable interrupt
	NVIC_SetPriorityGrouping( 0 );
	NVIC_SetPriority( TIM1_TRG_COM_TIM11_IRQn, tim11_pri_encoding );
	NVIC_EnableIRQ( TIM1_TRG_COM_TIM11_IRQn );
}

// Checks input buffer for messages from connected device
void TIM1_TRG_COM_TIM11_IRQHandler(void){

	uart_interrupt_counter++;

	if (uart_interrupt_counter % UART_TIME == 0){
		//Send_Byte('1');
		// TODO read buffer and discard message if it isn't valid
		// TODO send acknowledge to the connected device
	}

	// Clears interrupt flag so that other interrupts can work
	UART_Tim->SR &= ~TIM_SR_UIF;
}

// Sends a character over UART to the connected device
void Send_Byte(uint8_t data)
{
	while(!(USART2->SR & USART_SR_TXE)){};
	USART2->DR = data;
}

// Sends contents of output buffer to the connected device
void Send_Buffer()
{
	send_data = buffer_read(&out_buf);

	while(send_data != '\0'){
		Send_Byte(send_data);
		send_data = buffer_read(&out_buf);
	}
}

// Returns a complete message from input buffer if it si valid
uint8_t* Read_Buffer()
{
	first = buffer_check(&in_buf, in_buf.head);
	len   = buffer_check(&in_buf, in_buf.head + 2);
	last  = buffer_check(&in_buf, in_buf.head + len);
	if(first == START && last == STOP && len > 0){
		for(uint8_t i=0; i<len; i++){
			recv_data[i] = buffer_read(&in_buf);
		}

		return recv_data;
	}

	return &null;
}

// Constructs a message and sends it through the output buffer to the connected device
void Send_Command(uint8_t code, float value[], uint8_t len)
{
	buffer_write(&out_buf, START);
	buffer_write(&out_buf, code);
	buffer_write(&out_buf, len + 3);

	size = sizeof(value[0]);

	for(uint8_t i=0; i<len/size; i++){
		convert_float.f = value[i];

		buffer_write(&out_buf, convert_float.u[0]);
		buffer_write(&out_buf, convert_float.u[1]);
		buffer_write(&out_buf, convert_float.u[2]);
		buffer_write(&out_buf, convert_float.u[3]);
	}

	buffer_write(&out_buf, STOP);

	Send_Buffer();
}





