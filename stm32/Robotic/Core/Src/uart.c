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
uint8_t recv[20];
uint8_t bad_msg_counter = 0;


uint8_t c = 0;
uint8_t null = '\0';
float ack = 1;

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
	  if (c != '\r')
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
	UART_Tim->PSC = 2 - 1;
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
		// Reads buffer and discards message if it isn't valid
		Read_Buffer(recv);
		if (*recv != null){
			switch(recv[1]){
			case SPEED_RECEIVE:
				float left_speed = Read_Float(recv, 3);
				float right_speed = Read_Float(recv, 7);
				Set_Motor_Speed(left_speed, right_speed);
				bad_msg_counter = 0;
				memset(recv, 0, sizeof recv);

				break;
			case INIT_RECEIVE:
				sOdom_t odom = {
						.x = Read_Float(recv, 3),
						.y = Read_Float(recv, 7),
						.theta = Read_Float(recv, 11),
						.left_speed = 0,
						.right_speed = 0
				};
				Reset_Encoders(&odom);
				bad_msg_counter = 0;
				memset(recv, 0, sizeof recv);


				break;
			default:
				// TODO handle message that doesn't exist
				break;
			}

			// Sends an acknowledge for the received message to the connected device
			Send_Command(ACK_TRANSMIT, &ack, 1);
		}
		// If the whole message was not received in more than 30ms, discard the buffer
		else if (*recv == null && bad_msg_counter > 3){
			while(in_buf.head != in_buf.tail){
				buffer_read(&in_buf);
			}
			bad_msg_counter = 0;
		}
	}

	// Clears interrupt flag so that other interrupts can work
	UART_Tim->SR &= ~TIM_SR_UIF;
}


float Read_Float(uint8_t msg[], uint8_t start)
{
	convert_float.u[0] = msg[start];
	convert_float.u[1] = msg[start+1];
	convert_float.u[2] = msg[start+2];
	convert_float.u[3] = msg[start+3];

	return convert_float.f;
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

// Returns a complete message from input buffer if it is valid
void Read_Buffer(uint8_t* recv_data)
{
	first = buffer_check(&in_buf, in_buf.head);
	len   = buffer_check(&in_buf, in_buf.head+2);
	last  = buffer_check(&in_buf, in_buf.head+len-1);

	if(first == START && last == STOP && len > 0){
		for(uint8_t i=0; i<len+2; i++)
			recv_data[i] = buffer_read(&in_buf);
	}
	else if(first == START){
		bad_msg_counter++;
		recv_data[0] = null;
	}
}

// Constructs a message and sends it through the output buffer to the connected device
void Send_Command(uint8_t code, float value[], uint8_t len)
{
	buffer_write(&out_buf, START);
	buffer_write(&out_buf, code);
	buffer_write(&out_buf, len + 4);

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





