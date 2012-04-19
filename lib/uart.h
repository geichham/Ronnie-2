#ifndef UART_H
#define UART_H
 
void init_uart(uint16_t baudrate);
unsigned char uart_getc(void);
void uart_putc(unsigned char c);
void uart_puts(char *s);

ISR(USART0_RX_vect);

#endif
