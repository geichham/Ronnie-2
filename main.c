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

#define SRFout PD6
#define SRFin PD2

#define Servo OCR2A
#define left 35
#define middle 21
#define right 8

#define threshold 60
#define maxspeed 179

#define HMC5883L_WRITE 0x3C // write address
#define HMC5883L_READ 0x3D // read address

uint8_t max_course_deviation = 5;
uint8_t heading_previous = 0;

char buffer[10];
uint16_t dist = 0;

// SRF05
uint16_t distR = 0;
uint16_t distM = 0;
uint16_t distL = 0;

volatile unsigned int microseconds = 0; // holds pulselength
volatile uint8_t INT0_interrupt = 0; // used to determine weather it's rising or falling edge of the pulse 
volatile uint8_t measurement_complete = 0; // is being polled to determine end of reading

// compass 
int16_t raw_x = 0;	
int16_t raw_y = 0;
int16_t raw_z = 0;
float headingDegrees = 0;

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

void init_SRF05(void){
	// Timer einstellen
	TCCR0A = (1<<WGM01); // CTC Mode of Timer 0
	// ((16000000 / 8) / 100000) -1 = 19  
	OCR0A = 19; // 19 steps = interrupt every ~10µs
	
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
		_delay_us(12); // 12µs Trigger-Signal
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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Motor functions ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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


void Mleftbwd(void)	// left backward
{PORTC |= (1<<PC6); PORTC &= ~(1<<PC7);}

void Mleftfwd(void)	// left forward 
{PORTC &= ~(1<<PC6); PORTC |= (1<<PC7);}

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
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void init_Servo(void){
	
	TCCR2A |= (1<<COM2A1) | (1<<WGM21) | (1<<WGM20); // Fast PWM, non-inverting
	TCCR2B |= (1<<CS22) | (1<<CS21) | (1<<CS20); // Prescaler 1024 = 61,035Hz = 16ms 
	
	DDRD |= (1<<PD7); // OC2A as output
	
	OCR2A = 21; // set to 90 degrees neutral position	
}

void init_HMC5883L(void){
	
	sendUSART("Initializing HMC5883L...");
	
	
	i2c_start(HMC5883L_WRITE); 
	i2c_write(0x00); // set pointer to CRA
	i2c_write(0x70); // write 0x70 to CRA
	i2c_stop();
		
	i2c_start(HMC5883L_WRITE); 
	i2c_write(0x01); // set pointer to CRB
	i2c_write(0xA0); 
	i2c_stop();
		
	i2c_start(HMC5883L_WRITE); 
	i2c_write(0x02); // set pointer to measurement mode
	i2c_write(0x00); // continous measurement
	i2c_stop();
	
	sendUSART("Done!");
}

void getHeading(void){
	
	sendUSART("Start reading...");
	
	i2c_start_wait(HMC5883L_WRITE);
	i2c_write(0x03); // set pointer to X axis MSB 
	i2c_stop();
	
	i2c_rep_start(HMC5883L_READ); 

	raw_x = ((uint8_t)i2c_readAck())<<8;
	raw_x |= i2c_readAck();
	
	raw_z = ((uint8_t)i2c_readAck())<<8;
	raw_z |= i2c_readAck();
	
	raw_y = ((uint8_t)i2c_readAck())<<8;
	raw_y |= i2c_readNak();
	
	i2c_stop();
	
	headingDegrees = atan2((double)raw_y,(double)raw_x) * 180 / 3.141592654 + 180; 
	
	sendUSART("complete!");
	sendUSART("\r\n");
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
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Just some tests ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		/*accelerate(1);
		_delay_ms(1000);
		slowdown(1);
		_delay_ms(1000);´*/
		
		/* SRF05 TEST - Funktioniert
        dist = SRF05_getDistance(middle);
		itoa(dist, buffer, 10);
		
		sendUSART(buffer);
		sendchar('\n');
		_delay_ms(1000);
		*/
		
		/* Servo TEST  - Funktioniert
		Servo = left;
		_delay_ms(1000);
		Servo = middle; 
		_delay_ms(1000);
		Servo = right;
		_delay_ms(1000);
		*/
		
		/* 
		dist = SRF05_getDistance(left);
		itoa(dist, buffer, 10);
		
		sendUSART("Left: ");
		sendUSART(buffer);
		sendchar('\n');
		_delay_ms(1000);
		_delay_ms(100);
		
		dist = SRF05_getDistance(middle);
		itoa(dist, buffer, 10);
		
		sendUSART("Middle: ");
		sendUSART(buffer);
		sendchar('\n');
		_delay_ms(1000);
		
		dist = SRF05_getDistance(right);
		itoa(dist, buffer, 10);
		
		sendUSART("Right: ");
		sendUSART(buffer);
		sendchar('\n');
		_delay_ms(1000);*/
		
		/* HMC5883L TEST - Funktioniert
		getHeading();
		dtostrf(headingDegrees, 8, 2, buffer);
		sendUSART(buffer);
		_delay_ms(66);
		*/
		
		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
		
		// Main driving loop
		
		while(distM > threshold){
			distL = SRF05_getDistance(left);
			distM = SRF05_getDistance(middle);
			distR = SRF05_getDistance(right);   
		}
		
		if(distM < threshold && distR > threshold){
			
			while(distM < threshold){
				setPWMleft(maxspeed + 100);
				setPWMright(maxspeed - 100);
				distM = SRF05_getDistance(middle);
			}			
			
			setPWMleft(maxspeed);
			setPWMright(maxspeed);
		}
		else if(distM < threshold && distL > threshold){
	
			while(distM < threshold){
				setPWMleft(maxspeed - 100);
				setPWMright(maxspeed + 100);
				distM = SRF05_getDistance(middle);
			}	
			
			setPWMleft(maxspeed);
			setPWMright(maxspeed);
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

// interrupt every ~10µs
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