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
#include "lib/SRF05.h"

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
	
	headingDegrees = getHeading();
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

ISR(USART0_RX_vect){
	
	if(RX_pointer < 10){
		RX_buf[RX_pointer] = UDR0;
		RX_pointer++;	
	}
	else{
		RX_pointer = 0;
	}
}
