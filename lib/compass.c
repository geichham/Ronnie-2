#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "compass.h"
#include "I2C_master.h"

#define HMC5883L_WRITE 0x3C // write address
#define HMC5883L_READ 0x3D // read address

// axis 
int16_t raw_x = 0;	
int16_t raw_y = 0;
int16_t raw_z = 0;
float headingDegrees = 0;

void init_HMC5883L(void){

	I2C_start(HMC5883L_WRITE); 
	I2C_write(0x00); // set pointer to CRA
	I2C_write(0x70); // write 0x70 to CRA
	I2C_stop();
		
	I2C_start(HMC5883L_WRITE); 
	I2C_write(0x01); // set pointer to CRB
	I2C_write(0xA0); 
	I2C_stop();
		
	I2C_start(HMC5883L_WRITE); 
	I2C_write(0x02); // set pointer to measurement mode
	I2C_write(0x00); // continous measurement
	I2C_stop();
}

float getHeading(void){
	
	I2C_start(HMC5883L_WRITE);
	I2C_write(0x03); // set pointer to X axis MSB 
	I2C_stop();
	
	I2C_start(HMC5883L_READ); 

	raw_x = ((uint8_t)I2C_read_ack())<<8;
	raw_x |= I2C_read_ack();
	
	raw_z = ((uint8_t)I2C_read_ack())<<8;
	raw_z |= I2C_read_ack();
	
	raw_y = ((uint8_t)I2C_read_ack())<<8;
	raw_y |= I2C_read_nack();
	
	I2C_stop();
	
	headingDegrees = atan2((double)raw_y,(double)raw_x) * 180 / 3.141592654 + 180; 

	return headingDegrees;
}