#ifndef F_CPU 
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

volatile unsigned char RX_buf[5];
volatile uint8_t RX_pointer = 0;

void init_uart(uint16_t baudrate){
	
	uint16_t UBRR_val = (F_CPU / (baudrate * 16)) - 1;
	
	UBRR0H = UBRR_val << 8;				
	UBRR0L = UBRR_val & 0xFF;
	
	UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0); // UART TX (Transmit - senden) einschalten
	UCSR0C |= (1<<USBS0) | (3<<UCSZ00);	//Modus Asynchron 8N1 (8 Datenbits, No Parity, 1 Stopbit)	
}

unsigned char uart_getc(void){
	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	return UDR0; // return character from UDR0
}

void uart_putc(unsigned char c){
	
	while(!(UCSR0A & (1<<UDRE0))); // wait until sending is possible
	UDR0 = c; // output character saved in c
}

void uart_puts(char *s){
	
	while(*s){
		uart_putc(*s);
		s++;
	}
}

ISR(USART0_RX_vect){
	
	if(RX_pointer < 10){
		RX_buf[RX_pointer] = UDR0;
		RX_pointer++;	
	}
	else{
		RX_pointer = 0;
	}
}
