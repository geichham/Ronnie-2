#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

#include "lib/I2C_master.h"
#include "lib/motors.h"
#include "lib/compass.h"
#include "lib/SRF05.h"

/* define maximum string length in characters for uart.h (must correspond with define in lib/uart.c) */
#define MAX_STRLEN 12

#include "lib/uart.h"

#define Servo OCR2A
#define left 35
#define middle 21
#define right 8

#define threshold 60
#define maxspeed 120

float headingDegrees;
uint8_t max_course_deviation = 5;
uint8_t heading_previous = 0;
uint8_t course_deviation = 0;


char buffer[10];

// SRF05
uint16_t distR = 0;
uint16_t distM = 0;
uint16_t distL = 0;


void init_Servo(void){
	
	TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); // Fast PWM, non-inverting
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20); // Prescaler 1024 = 61,035Hz = 16ms 
	
	DDRD |= (1<<PD7); // OC2A as output
	
	OCR2A = 21; // set to 90 degrees neutral position	
}

void getCourse(void){
	headingDegrees = getHeading();
	
	if( headingDegrees > (heading_previous + max_course_deviation) ){
		while(headingDegrees > (heading_previous + max_course_deviation) ){
			setPWMleft(maxspeed + 50);
			setPWMright(0);
			headingDegrees = getHeading();
			_delay_ms(66);
		}
				
		setPWMleft(maxspeed);
		setPWMright(maxspeed);	
	}
	else if(headingDegrees < (heading_previous - max_course_deviation) ){
		while(headingDegrees < (heading_previous - max_course_deviation)){
			setPWMleft(0);
			setPWMright(maxspeed + 50);
			headingDegrees = getHeading();
			_delay_ms(66);	
		}
				
		setPWMleft(maxspeed);
		setPWMright(maxspeed);	
	}
}

int main(void)
{
	init_SRF05();
	init_uart(9600);
	init_Servo();
	init_Timer1();
	
	
	I2C_init();
	init_HMC5883L();
	
	
	sei();
	
	
	distL = SRF05_getDistance(left);
	distM = SRF05_getDistance(middle);
	distR = SRF05_getDistance(right);
	
	
	Mleftfwd();
	Mrightfwd();
	accelerate(2);
	
	
	headingDegrees = getHeading();
	heading_previous = headingDegrees;
	
    while(1)
    {
		
		// Driving loop with object detection
		
		while(distM > threshold){
			distL = SRF05_getDistance(left);
			distM = SRF05_getDistance(middle);			
			distR = SRF05_getDistance(right); 
			//getCourse();
		}
		
		if( (distM < threshold) && (distL > threshold )){
			
			Mleftfwd();
			Mrightbwd();
			
			while(distM < threshold){
				distM = SRF05_getDistance(middle);
			}			
			
			Mleftfwd();
			Mrightfwd();
			setPWMleft(motor_values[maxspeed]);
			setPWMright(motor_values[maxspeed]);
		}
		else if( (distM < threshold) && (distR > threshold) ){
	
			Mleftbwd();
			Mrightfwd();
	
			while(distM < threshold){
				distM = SRF05_getDistance(middle);
			}	
			
			Mleftfwd();
			Mrightfwd();
			setPWMleft(motor_values[maxspeed]);
			setPWMright(motor_values[maxspeed]);
		}
		else{ // if nothing else applies --> left and right are not greater than threshold
			
			OCR1A = 0;
			OCR1B = 0;
			
			Mleftbwd();
			Mrightbwd();
			
			accelerate(2);
			_delay_ms(300);
			
			OCR1A = 0;
			OCR1B = 0;
			
			
			Mleftfwd();
			Mrightbwd();
			
			accelerate(2);
			_delay_ms(300);
			
			OCR1A = 0;
			OCR1B = 0;
			
			distL = SRF05_getDistance(left);
			distM = SRF05_getDistance(middle);			
			distR = SRF05_getDistance(right); 
			
			Mleftfwd();
			Mrightfwd();
			accelerate(2);
		}
		
		heading_previous = getHeading();
    }
}

