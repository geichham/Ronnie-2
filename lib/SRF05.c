#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "SRF05.h"

#define SRFout PD6
#define SRFin PD2

#define Servo OCR2A

volatile unsigned int microseconds = 0; // holds pulselength
volatile uint8_t INT0_interrupt = 0; // used to determine weather it's rising or falling edge of the pulse 
volatile uint8_t measurement_complete = 0; // is being polled to determine end of reading

void init_SRF05(void){
	// initialize Timer 
	TCCR0A = (1<<WGM01); // CTC Mode of Timer 0
	// ((16000000 / 8) / 100000) -1 = 19  
	OCR0A = 19; // 19 steps = interrupt every ~10µs
	
	TIMSK0 |= (1<<OCIE0A); // Enable compare interrupt
	//~~~~~~~~~~~~~~~~~
	
	EIMSK |= (1<<INT0); // enable INT0 in external interrupt mask register  
	
	DDRD |= (1<<SRFout); // set trigger pin to output 
	DDRD &= ~(1<<SRFin); // set INT0 pin to input
	
}

uint16_t SRF05_getDistance(uint8_t direction){		
		
		Servo = direction; // rotate sensor in the passed in direction
		
		_delay_ms(600);
		
		uint16_t distance = 0;
		distance = 0; // reset distance, otherwise readings are added up
		
		// read the SRF05 four times and then calculate the average to get more stable values
		for(uint8_t sample = 0; sample < 4; sample++){
			microseconds = 0; // reset microseconds aswell
			TCNT0 = 0; // set Timer to 0
			
			// Trigger the reading:
			PORTD |= (1<<SRFout);
			_delay_us(12); // 12µs Trigger-Signal
			PORTD &= ~(1<<SRFout);
			 
			// trigger external interrupt 0 on rising edge:
			EICRA = (1<<ISC01) | (1<<ISC00);
			
			while(measurement_complete != 1){
				//wait until end of reading
			}
			
			measurement_complete = 0;
			distance += microseconds;
			_delay_ms(10);
		}
		
		return (distance / 58) / 4;
}

ISR(INT0_vect){  // external interrupt 0 
	
	if(INT0_interrupt == 0){ // only if it hasn't yet been triggered on rising edge 
		TCCR0B |= (1<<CS01); // start Timer with prescaler 8
		
		// set INT0 to falling edge:
		EICRA = (1<<ISC01); // EICRA = External Interrupt Control Register A
		
		INT0_interrupt = 1; // because it has been triggered on rising edge already 
	}
	else{ // of INT0 has already been triggered prevoiusly
		TCCR0B &= ~(1<<CS01); // stop Timer
		
		INT0_interrupt = 0; //  reset variable
		measurement_complete = 1; // set to 1 to indicate end of reading	
	}	
}

// interrupt every ~10µs
ISR(TIMER0_COMPA_vect){
	microseconds += 10;
}
