#ifndef UART_H
#define UART_H

#ifndef MAX_STRLEN
	#warning MAX_STRLEN not defined for lib/uart.h
	/* provide a default to prevent compiler error */
	#define MAX_STRLEN 12
#endif /* MAX_STRLEN */

/* the received string (accessible by all source files inluding UART_H) */
extern unsigned char string[MAX_STRLEN+1];

void init_uart(uint16_t baudrate);
unsigned char uart_getc(void);
void uart_putc(unsigned char c);
void uart_puts(char *s);

ISR(USART0_RX_vect);

#endif
