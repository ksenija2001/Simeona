/*
 * uart.c
 *
 *  Created on: Mar 14, 2024
 *      Author: xenia
 */

#include "uart.h"

volatile uint32_t uart_interrupt_counter = 0;

uint8_t in_buffer[BUFFER_SIZE];
uint8_t out_buffer[BUFFER_SIZE];

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
uint8_t code  = 0;

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

float send_odom_enc[7];

uint32_t test =0;

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

    USART2->CR1 |= (USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TCIE);

	memset(recv, 0, sizeof recv);

}

// Interrupt handler for receiving data over UART
void USART2_IRQHandler(void)
{
	// test = USART2->SR;

	// Handles interrupt if a byte was received
	if (USART2->SR & USART_SR_RXNE) {
	  c = USART2->DR;
	  buffer_write(&in_buf, c);
	}
	// Hanldes interrupt if a byte was transmitted successfully
	else if (USART2->SR & USART_SR_TC){
		USART2->SR &= ~(0b1 << USART_SR_TC_Pos);
		if (out_buf.head != out_buf.tail){
			send_data = buffer_read(&out_buf);
			USART2->DR = send_data;
		}
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

	if (uart_interrupt_counter % UART_TIME == 0 &&
		in_buf.head != in_buf.tail)
	{
		Read_Buffer(recv);
		if (*recv != null){
			switch(recv[1]){
			case ODOM_RECEIVE:
				Send_Command(ACK_TRANSMIT, &ack, 4);

				sOdom_t* odom_enc = Read_Encoders();

				send_odom_enc[0] = odom_enc->x;
				send_odom_enc[1] = odom_enc->y;
				send_odom_enc[2] = odom_enc->theta;
				send_odom_enc[3] = odom_enc->left_speed;
				send_odom_enc[4] = odom_enc->right_speed;
				send_odom_enc[5] = odom_enc->left_inc;
				send_odom_enc[6] = odom_enc->right_inc;

				Send_Command(ODOM_TRANSMIT, send_odom_enc, sizeof(send_odom_enc));
				break;
			case SPEED_RECEIVE:
				Set_Motor_Speed(Read_Float(recv, 3),  // left speed
								Read_Float(recv, 7)); // right_speed

				// Sends an acknowledge for the received message
				// to the connected device
				Send_Command(ACK_TRANSMIT, &ack, 4);
				break;
			case INIT_RECEIVE:
				sOdom_t odom = {
						.x = Read_Float(recv, 3),
						.y = Read_Float(recv, 7),
						.theta = Read_Float(recv, 11),
						.left_speed = 0,
						.right_speed = 0,
						.left_inc = 0,
						.right_inc = 0
				};
				Reset_Encoders(&odom);

				// Sends an acknowledge for the received message
				// to the connected device
				Send_Command(ACK_TRANSMIT, &ack, 4);
				break;
			case CONFIG_RECEIVE:
				Config(Read_Float(recv, 3),   // diameter
						Read_Float(recv, 7)); // track distance

				// Sends an acknowledge for the received message
				// to the connected device
				Send_Command(ACK_TRANSMIT, &ack, 4);
				break;
			}

			// If a valid message was received the counter
			// for reseting the buffer is reset
			bad_msg_counter = 0;
		}
		// If the whole message was not received in more than 5ms, discard the buffer
		else if (bad_msg_counter >= 4){
			while(in_buf.head != in_buf.tail)
				buffer_read(&in_buf);
			bad_msg_counter = 0;
		}

		memset(recv, 0, sizeof recv);
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
	while(out_buf.head != out_buf.tail){
		send_data = buffer_read(&out_buf);
		Send_Byte(send_data);
	}
}

// Returns a complete message from input buffer if it is valid
void Read_Buffer(uint8_t* recv_data)
{
	first = buffer_check(&in_buf, in_buf.head);
	code   = buffer_check(&in_buf, in_buf.head+1);
	len   = buffer_check(&in_buf, in_buf.head+2);
	last  = buffer_check(&in_buf, in_buf.head+len);

	if(first == START &&
	   last == STOP &&
	   len > 0){
		for(uint8_t i=0; i<len+2; i++)
			recv_data[i] = buffer_read(&in_buf);
		return;
	}

	// If the message is not valid null is returned
	// and the counter for reseting the buffer increases
	bad_msg_counter++;
	recv_data[0] = null;
}

// Constructs a message and sends it through the output buffer to the connected device
void Send_Command(uint8_t code, float value[], uint8_t len)
{
	buffer_write(&out_buf, START);
	buffer_write(&out_buf, code);
	buffer_write(&out_buf, len+3);

	size = sizeof(value[0]);

	for(uint8_t i=0; i<len/size; i++){
		convert_float.f = value[i];

		buffer_write(&out_buf, convert_float.u[0]);
		buffer_write(&out_buf, convert_float.u[1]);
		buffer_write(&out_buf, convert_float.u[2]);
		buffer_write(&out_buf, convert_float.u[3]);
	}

	buffer_write(&out_buf, STOP);

	// First byte needs to be sent manually to start the USART interrupt transmission
	send_data = buffer_read(&out_buf);
	while(!(USART2->SR & USART_SR_TXE));
	USART2->DR = send_data;
}





