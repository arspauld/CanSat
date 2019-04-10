/*
 * ms5607.c
 *
 * Created: 9/3/2018 12:51:59 PM
 *  Author: cellis
 */ 

#include <asf.h>
#include "ms5607.h"
#include "spi_controller.h"

// this is a # define, it's a way to define a constant like a command that will never change
#define CMD_MS5607_READ_ADC		0x00
#define CMD_MS5607_CONVERT_D1	0x46
#define CMD_MS5607_CONVERT_D2	0x56
#define MS5607_PORT				0x10

uint16_t ms5607_read(uint16_t comm)
{
	uint16_t rx_data = 0; // temporary 16-bit value
	spi_select(MS5607_PORT); // select our spi device
	spi_write(comm); // write a specified command to ask for data
	
	/*typecast this expression from an 8-bit to a 16-bit and shift it 8 bits to the left
	  meaning the returned value is now in the upper 8 bits rx_data*/
	rx_data = (uint16_t)spi_read()<<8; 
	
	// OR the second byte with the 16-bit variable, the returned value is now in the lower 8 bits of 'rx_data'
	rx_data |= spi_read(); 
	
	spi_select(MS5607_PORT); // end spi exchange
	
	return rx_data; // return the 16-bit value
}

uint32_t ms5607_convert_d1(void)
{
	uint32_t rx_data = 0; // temporary 16-bit value
	// CONVERT D1
	spi_select(MS5607_PORT); // select our spi device
	spi_write(CMD_MS5607_CONVERT_D1); // write a specified command to ask for data
	delay_ms(5);
	spi_select(MS5607_PORT);
	
	spi_select(MS5607_PORT);
	spi_write(CMD_MS5607_READ_ADC);
	rx_data  = (uint32_t) spi_read()<<16;
	rx_data |= (uint32_t) spi_read()<<8;
	rx_data |= spi_read();
	spi_select(MS5607_PORT);
	
	return rx_data;
}

uint32_t ms5607_convert_d2(void)
{
	uint32_t rx_data = 0; // temporary 16-bit value
	// CONVERT D2
	spi_select(MS5607_PORT); // select our spi device
	spi_write(CMD_MS5607_CONVERT_D2); // write a specified command to ask for data
	delay_ms(5);
	spi_select(MS5607_PORT);
	
	spi_select(MS5607_PORT);
	spi_write(CMD_MS5607_READ_ADC);
	rx_data  = (uint32_t) spi_read()<<16;
	rx_data |= (uint32_t) spi_read()<<8;
	rx_data |= spi_read();
	spi_select(MS5607_PORT);
	
	return rx_data;
}