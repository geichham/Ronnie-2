#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include "motors.h"

#define maxspeed 179

// f(x) = 128-128*cos(x)
// allows for smooth acceleration and breaking
uint8_t motor_values[180] = {
	0, 0, 0, 0, 0, 0, 0, 0, 1, 
	1, 1, 2, 2, 3, 3, 4, 4, 5, 6,
	6, 7, 8, 9, 10, 11, 11, 12, 13, 14, 16,
	17, 18, 19, 20, 21, 23, 24, 25, 27, 28,
	29, 31, 32, 34, 35, 37, 39, 40, 42, 44,
	45, 47, 49, 50, 52, 54, 56, 58, 60, 62,
	63, 65, 67, 69, 71, 73, 75, 77, 80, 82, 
	84, 86, 88, 90, 92, 94, 97, 99, 101, 103,
	105, 107, 110, 112, 114, 116, 119, 121, 123, 125,
	127, 130, 132, 134, 136, 139, 141, 143, 145, 148,
	150, 152, 154, 156, 158, 161, 163, 165, 167, 169,
	171, 173, 175, 178, 180, 182, 184, 186, 188, 190,
	191, 193, 195, 197, 199, 201, 203, 205, 206, 208,
	210, 211, 213, 215, 216, 218, 220, 221, 223, 224, 
	226, 227, 228, 230, 231, 232, 234, 235, 236, 237,
	238, 239, 241, 242, 243, 244, 244, 245, 246, 247, 
	248, 249, 249, 250, 251, 251, 252, 252, 253, 253,
	254, 254, 254, 255, 255, 255, 255, 255, 255, 255
};

void init_Timer1(void){
	
	TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM10); // non-Inverting 8-bit Fast PWM
	TCCR1B |= (1<<WGM12) | (1<<CS10); // no prescaler
	
	TIMSK1 = 0x00; // disable interrputs for Timer 1 
	
	// set ouputs for motors
	DDRB |= 0x03;
	DDRC |= 0xC0;
	DDRD |= 0x30;
	
	OCR1A = 0; //  set motor speed to 0
	OCR1B = 0; 
}

void setPWMleft(uint8_t speed) // speed left motor
{OCR1BL = speed;}

void setPWMright(uint8_t speed) // speed right motor
{OCR1AL = speed;}


void Mleftfwd(void)	// left forward 
{PORTC &= ~(1<<PC6); PORTC |= (1<<PC7);}

void Mleftbwd(void)	// left backward
{PORTC |= (1<<PC6); PORTC &= ~(1<<PC7);}

void Mleftstop(void)	// stop
{ PORTC &= ~(1<<PC6); PORTC &= ~(1<<PC7);}


void Mrightfwd(void)	// right forward
{PORTB |= (1<<PB0); PORTB &= ~(1<<PB1);}

void Mrightbwd(void)	// right backward
{PORTB &= ~(1<<PB0); PORTB |= (1<<PB1);}

void Mrightstop(void)	// stop
{PORTB &= ~(1<<PB0); PORTB &= ~(1<<PB1);}
	
	
void accelerate(uint8_t step){
	
	for(uint8_t i = 0; i <= maxspeed; i += step){
		setPWMleft(motor_values[i]);
		setPWMright(motor_values[i]);
		_delay_ms(20);
	}
}

void slowdown(uint8_t step){
	
	for(uint8_t i = maxspeed; i > 0; i -= step){
		setPWMleft(motor_values[i]);
		setPWMright(motor_values[i]);
		_delay_ms(20);
	}	
}