#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include "lib/standards.h"
#include "lib/i2cmaster.h"
#include "lib/motors.h"
#include "lib/compass.h"

#define SRFout PD6
#define SRFin PD2

#define Servo OCR2A
#define left 35
#define middle 21
#define right 8

#define threshold 60
#define maxspeed 179

float headingDegrees;
uint8_t max_course_deviation = 5;
uint8_t heading_previous = 0;

char buffer[10];

// SRF05
uint16_t distR = 0;
uint16_t distM = 0;
uint16_t distL = 0;

volatile unsigned int microseconds = 0; // holds pulselength
volatile uint8_t INT0_interrupt = 0; // used to determine weather it's rising or falling edge of the pulse 
volatile uint8_t measurement_complete = 0; // is being polled to determine end of reading

// RX input via interrupt
/*
 *	0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9 
 *	   |	 |     |     |	   |     |     |     |     |
 *	M  |  D  | S1  | S2  | S3  |
 *
 *	M = motor / actuator
 *  D = direction
 *	Sn = speed / data
 */
volatile unsigned char RX_buf[5];
volatile uint8_t RX_pointer = 0;

void init_SRF05(void){
	// Timer einstellen
	TCCR0A = (1<<WGM01); // CTC Mode of Timer 0
	// ((16000000 / 8) / 100000) -1 = 19  
	OCR0A = 19; // 19 steps = interrupt every ~10�s
	
	TIMSK0 |= (1<<OCIE0A); // Enable compare interrupt
	//~~~~~~~~~~~~~~~~~
	
	EIMSK |= (1<<INT0); // enable INT0 in external interrupt mask register  
	
	DDRB |= (1<<SRFout); // set trigger pin to output 
	DDRD &= ~(1<<SRFin); // set INT0 pin to input
}

uint16_t SRF05_getDistance(uint8_t direction){		
		
		Servo = direction; // rotate sensor in the passed in direction
		
		_delay_ms(600);
		
		uint16_t distance = 0;
		
		distance = 0; // reset distance, otherwise readings are added up
		microseconds = 0; // reset microseconds aswell
		TCNT0 = 0; // set Timer to 0
		
		// Trigger the reading:
		PORTD |= (1<<SRFout);
		_delay_us(12); // 12�s Trigger-Signal
		PORTD &= ~(1<<SRFout);
		
		// trigger external interrupt 0 on rising edge:
		EICRA = (1<<ISC01) | (1<<ISC00);
		
		while(measurement_complete != 1){
			//wait until end of reading
		}
		
		EICRA &= ~(1<<ISC01);
		
		measurement_complete = 0;
		
		distance = microseconds / 58;
		
		heading_previous = direction;
		
		return distance;
}

void init_Servo(void){
	
	TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); // Fast PWM, non-inverting
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20); // Prescaler 1024 = 61,035Hz = 16ms 
	
	DDRD |= (1<<PD7); // OC2A as output
	
	OCR2A = 21; // set to 90 degrees neutral position	
}

int main(void)
{
	init_SRF05();
	init_USART();
	init_Servo();
	init_Timer1();
	
	
	i2c_init();
	init_HMC5883L();
	
	sei();
	
	getHeading();
	heading_previous = headingDegrees;
	
	distL = SRF05_getDistance(left);
	distM = SRF05_getDistance(middle);
	distR = SRF05_getDistance(right);
	
	
	Mleftfwd();
	Mrightfwd();
	accelerate(1);
	
    while(1)
    {
	
		// Main driving loop
		
		while(distM > threshold){
			distL = SRF05_getDistance(left);
			distM = SRF05_getDistance(middle);
			distR = SRF05_getDistance(right);   
		}
		
		if(distM < threshold && distR > threshold){
			
			while(distM < threshold){
				setPWMleft(motor_values[maxspeed]);
				setPWMright(motor_values[0]);
				distM = SRF05_getDistance(middle);
			}			
			
			setPWMleft(motor_values[maxspeed]);
			setPWMright(motor_values[maxspeed]);
		}
		else if(distM < threshold && distL > threshold){
	
			while(distM < threshold){
				setPWMleft(motor_values[0]);
				setPWMright(motor_values[maxspeed]);
				distM = SRF05_getDistance(middle);
			}	
			
			setPWMleft(motor_values[maxspeed]);
			setPWMright(motor_values[maxspeed]);
		}
		
		// another test
		/*
		while( (headingDegrees < (heading_previous + max_course_deviation)) && (headingDegrees > heading_previous - max_course_deviation ) ){
			getHeading();
			_delay_ms(66);
		}
		
		if(headingDegrees > (heading_previous + max_course_deviation) ){
		
			while(headingDegrees > (heading_previous + max_course_deviation) ){
				setPWMlinks(maxspeed + 50);
				setPWMrechts(maxspeed - 50);
				getHeading();
				_delay_ms(66);
			}
			
			setPWMlinks(maxspeed);
			setPWMrechts(maxspeed);	
		}
		else if(headingDegrees < (heading_previous - max_course_deviation) ){
			
			while(headingDegrees < (heading_previous - max_course_deviation)){
				setPWMlinks(maxspeed - 50);
				setPWMrechts(maxspeed + 50);
				getHeading();
				_delay_ms(66);	
			}
			
			setPWMlinks(maxspeed);
			setPWMrechts(maxspeed);	
		}	
		*/		
    }
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

// interrupt every ~10�s
ISR(TIMER0_COMPA_vect){
	microseconds += 10;
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