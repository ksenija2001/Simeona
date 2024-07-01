/*
 * circular_buffer.h
 *
 *  Created on: Jan 19, 2024
 *      Author: xenia
 */

#ifndef INC_CIRCULAR_BUFFER_H_
#define INC_CIRCULAR_BUFFER_H_

#include <stdint.h>

typedef struct{
	uint8_t len;
	volatile uint8_t* buffer;
	volatile uint8_t head;
	volatile uint8_t tail;
}circular_buff;

static inline void buffer_write(circular_buff* buf, uint8_t x)
{
	buf->buffer[buf->tail] = x;

	if ((buf->tail + 1) >= buf->len)
		buf->tail = 0;
	else
		buf->tail++;
}

static inline uint8_t buffer_read(circular_buff* buf)
{
	if (buf->head == buf->tail)
		return '\0';

	uint8_t read = buf->buffer[buf->head];
	buf->buffer[buf->head] = '\0';

	if ((buf->head + 1) >= buf->len)
		buf->head = 0;
	else
		buf->head++;

	return read;
}

static inline uint8_t buffer_check(circular_buff* buf, uint8_t pos)
{
	if (pos > buf->len)
		pos = pos - buf->len;

	if ((buf->head == buf->tail) ||
		(pos > buf->tail))
		return '\0';

	uint8_t read = buf->buffer[pos];

	return read;
}

#endif /* INC_CIRCULAR_BUFFER_H_ */
