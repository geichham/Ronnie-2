#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "compass.h"
#include "i2cmaster.h"

#define HMC5883L_WRITE 0x3C // write address
#define HMC5883L_READ 0x3D // read address

// axis 
int16_t raw_x = 0;	
int16_t raw_y = 0;
int16_t raw_z = 0;
float headingDegrees = 0;

void init_HMC5883L(void){

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
}

float getHeading(void){
	
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

	return headingDegrees;
}