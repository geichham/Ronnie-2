#ifndef F_CPU 
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

/* define maximum string length in characters for uart.h */
#define MAX_STRLEN 12

#include "uart.h"

void init_uart(uint16_t baudrate){
	
	uint16_t UBRR_val = (F_CPU/16)/(baudrate-1);
	
	UBRR0H = UBRR_val >> 8;				
	UBRR0L = UBRR_val;
	
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
	
	static uint8_t character = 0;
	unsigned char t;

	t = UDR0;
	
	if( (character < MAX_STRLEN) && (t != '\n') ){
		string[character] = t;
		character++;
	}
	else{
		string[MAX_STRLEN] = '\0'; // generate a "valid" 0 terminated C-String
		character = 0;
	}
}
