
#define F_CPU 16E6
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <math.h>
#include "i2cmaster.h"	// Enable communication to sensor (i2c_init())							(twimaster.c)
// #include "lcd.h"		// Enable the LCD (lcd_init(), lcd_clear(), lcd_gotoxy())				(lcd.c)
#include "usart.h"
#include "MMA8451.h"

#define SENSITIVITY_2G    1024

void MMA8451_init();
void i2c_write_register8(unsigned char reg, unsigned char value);
unsigned char i2c_read_register8(unsigned char reg);

unsigned char x, y,z,Xoffset,Yoffset,Zoffset;
int x_val, y_val, z_val;
float x_val_float, y_val_float, z_val_float;
float x_angle, y_angle,z_angle;
float x_g, y_g,z_g;
unsigned char data_array[10];


int main(void)
{
	uart_init(); // open the communication to the microcontroller
	i2c_init(); // initialize i2c
	io_redirect(); // redirect i/o
	MMA8451_init();
	
	while (1)
	{
		//Reading accelerometer's registers
		data_array[0] = i2c_read_register8(MMA8451_REG_OUT_X_MSB);
		data_array[1] = i2c_read_register8(MMA8451_REG_OUT_X_LSB);
		data_array[2] = i2c_read_register8(MMA8451_REG_OUT_Y_MSB);
		data_array[3] = i2c_read_register8(MMA8451_REG_OUT_Y_LSB);
		data_array[4] = i2c_read_register8(MMA8451_REG_OUT_Z_MSB);
		data_array[5] = i2c_read_register8(MMA8451_REG_OUT_Z_LSB);

		//ADD EXPLANATION
		//X-axis acceleration normalization
		x_val = ((short) (data_array[0]<<8 | data_array[1])) >> 2; // Compute 14-bit X-axis output value
		x_val_float = x_val/16384.0;    // +/-2g range -> 1g = 16384/4 = 4096 counts

		//Y-axis acceleration normalization
		y_val = ((short) (data_array[2]<<8 | data_array[3])) >> 2; // Compute 14-bit Y-axis output value
		y_val_float = y_val/16384.0;   // +/-2g range -> 1g = 16384/4 = 4096 counts
		
		//Z-axis acceleration normalization
		z_val = ((short) (data_array[4]<<8 | data_array[5])) >> 2; // Compute 14-bit Z-axis output value
		z_val_float = z_val/16384.0;   // +/-2g range -> 1g = 16384/4 = 4096 counts
		

		//ADD EXPLANATION
		// Compute X-axis offset correction value
		Xoffset = x_val / 8 * (-1);

		// Compute Y-axis offset correction value
		Yoffset = y_val / 8 * (-1);
		
		// Compute Z-axis offset correction value
		Zoffset = (x_val - SENSITIVITY_2G) / 8 * (-1);
		
		//ADD EXPLANATION
		x_val+=Xoffset;
		y_val+=Yoffset;
		z_val+=Zoffset;
		
		//ADD EXPLANATION
		//Calculating Angles
		x_angle = atan(x_val_float/z_val_float);
		x_angle = x_angle*(180.0/3.141592);

		
		y_angle = atan(y_val_float/z_val_float);
		y_angle = y_angle*(180.0/3.141592);

		
		z_angle = atan(x_val_float/y_val_float);
		z_angle = z_angle*(180.0/3.141592);
		
		//orientation = getOrientation(); //reading the detected orientation
		
		//Calculating Acceleration
		x_g = ((float) x_val) / SENSITIVITY_2G;
		y_g = ((float) y_val) / SENSITIVITY_2G;
		z_g = ((float) z_val) / SENSITIVITY_2G;
		

		// ***Printing the x-axis angle/acceleration***
		//If using LCD before printf -> LCD_set_cursor(0,0);
		
		printf("X:%6.2f ", x_angle);  // X ANGLE
		printf("X:%7.2f [m/s^2]", x_g);  // X ACC
		// LCD_set_cursor(0,0); printf("%7d", x_val);
		// LCD_set_cursor(0,0); printf("%f", x_val_float);
		
		// ***Printing the y-axis angle/acceleration***
		//If using LCD before printf -> LCD_set_cursor(0,1);

		printf("Y:%6.2f", y_angle); // Y ANGLE
		printf("Y:%7.2f [m/s^2]", y_g);  // Y ACC
		//LCD_set_cursor(0,1); printf("%7d", y_val);
		//LCD_set_cursor(0,1); printf("%f", y_val_float);
		
		// ***Printing the z-axis angle/acceleration***
		//If using LCD before printf -> LCD_set_cursor(9,1);;
		printf("Z:%6.2f \n", z_angle); // Z ANGLE
		//LCD_set_cursor(9,1); printf("Z:%7.2f [m/s^2]", z_g);  // Z ACC
		

		// ***Cycle counting***
		//LCD_set_cursor(0,0); printf("%7d", iterations);
		
		// ***Printing the detected orientation***
		//LCD_set_cursor(0,0); printf("%d",orientation);
		
		// ***Printing Registers***
		//LCD_set_cursor(0,0); printf("%d", data_array[0]);
		//LCD_set_cursor(2,0); printf("%d", data_array[1]);
		//LCD_set_cursor(3,0); printf("%d", data_array[4]);
		//LCD_set_cursor(4,0); printf("%d", data_array[5]);
		
	}
}

void MMA8451_init()
{

	//enabled all of these
	i2c_start(0x3A+I2C_WRITE);
	i2c_write(MMA8451_REG_WHOAMI);
	i2c_rep_start(0x3A+I2C_READ);
	x = i2c_readNak();		//if x not 0x3A then false  //WHY?!
	i2c_stop();
	
	//enabled	
	i2c_write_register8(MMA8451_REG_CTRL_REG2, 0x40);
	
	x = i2c_read_register8(MMA8451_REG_WHOAMI);
	
	i2c_write_register8(MMA8451_REG_CTRL_REG2, 0x40);		// reset
	_delay_ms(10);											// very important
	// test returværdi fra i2c_start om device er busy //PREVEDI!
	
	y = i2c_read_register8(MMA8451_REG_CTRL_REG2);
	
	// Enable 2G range
	i2c_write_register8(MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_2_G);

	// High resolution
	i2c_write_register8(MMA8451_REG_CTRL_REG2, 0x02);

	// DRDY on INT1
	i2c_write_register8(MMA8451_REG_CTRL_REG4, 0x01);
	i2c_write_register8(MMA8451_REG_CTRL_REG5, 0x01);

	// Turn on orientation config
	i2c_write_register8(MMA8451_REG_PL_CFG, 0x40);

	// Activate at max rate, low noise mode,
	i2c_write_register8(MMA8451_REG_CTRL_REG1, 0x20 | 0x10 | 0x08 | 0x04 | 0x01);
	
	
}

void i2c_write_register8(unsigned char reg, unsigned char value)
{
	i2c_start(MMA8451_DEFAULT_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
}

unsigned char i2c_read_register8(unsigned char reg)
{
	unsigned char ret_val=0;
	i2c_start(MMA8451_DEFAULT_ADDRESS+I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(MMA8451_DEFAULT_ADDRESS+I2C_READ);
	ret_val = i2c_readNak();
	i2c_stop();
	return(ret_val);
}

unsigned char getOrientation(void) {
	return i2c_read_register8(MMA8451_REG_PL_STATUS) & 0x07;
}