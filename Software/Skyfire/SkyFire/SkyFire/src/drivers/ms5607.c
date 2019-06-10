/*
 * ms5607.c
 *
 * Created: 9/3/2018 12:51:59 PM
 *  Author: cellis
 */ 

#include <asf.h>
#include "ms5607.h"
#include "spi_controller.h"

void ms5607_init(void){
	PORTC.OUT |= 0x10; // makes the 4th pin on Port C be set on high (0b00010000)
	
	// Resets the ms5607
	flip_ms5607();
	spi_write(CMD_MS5607_RESET);
	delay_ms(3);
	flip_ms5607();
	delay_ms(2);
}

uint16_t ms5607_read(uint16_t comm)
{
	uint16_t rx_data = 0; // temporary 16-bit value
	flip_ms5607(); // select our spi device
	spi_write(comm); // write a specified command to ask for data
	
	/*typecast this expression from an 8-bit to a 16-bit and shift it 8 bits to the left
	  meaning the returned value is now in the upper 8 bits rx_data*/
	rx_data = (uint16_t)spi_read()<<8; 
	
	// OR the second byte with the 16-bit variable, the returned value is now in the lower 8 bits of 'rx_data'
	rx_data |= spi_read(); 
	
	flip_ms5607(); // end spi exchange
	
	return rx_data; // return the 16-bit value
}

uint32_t ms5607_convert_d1(void)
{
	volatile uint32_t rx_data = 0; // temporary 16-bit value
	// CONVERT D1
	flip_ms5607(); // select our spi device
	spi_write(CMD_MS5607_D1_4096); // write a specified command to ask for data
	delay_ms(10);
	flip_ms5607();
	
	flip_ms5607();
	spi_write(CMD_MS5607_READ_ADC);
	rx_data  = (uint32_t) spi_read()<<16;
	rx_data |= (uint32_t) spi_read()<<8;
	rx_data |= spi_read();
	flip_ms5607();
	
	return rx_data;
}

uint32_t ms5607_convert_d2(void)
{
	volatile uint32_t rx_data = 0; // temporary 16-bit value
	// CONVERT D2
	flip_ms5607(); // select our spi device
	spi_write(CMD_MS5607_D2_4096); // write a specified command to ask for data
	delay_ms(10);
	flip_ms5607();
	
	flip_ms5607();
	spi_write(CMD_MS5607_READ_ADC);
	rx_data  = (uint32_t) spi_read()<<16;
	rx_data |= (uint32_t) spi_read()<<8;
	rx_data |= spi_read();
	flip_ms5607();
	
	return rx_data;
}

void ms5607_write_unprotected(uint8_t comm){
	flip_ms5607(); // select our spi device
	spi_write(comm); // write a specified command to ask for data
}

uint32_t ms5607_read_unprotected(void){
	flip_ms5607(); // deselects from the write
	uint32_t rx_data = 0; // temporary 16-bit value
	flip_ms5607(); // select our spi device
	spi_write(CMD_MS5607_READ_ADC); // write a specified command to ask for data
	rx_data  = (uint32_t) spi_read()<<16;
	rx_data |= (uint32_t) spi_read()<<8;
	rx_data |= spi_read();
	flip_ms5607();
	
	return rx_data;
}

void flip_ms5607(void){
	MS5607_PORT.OUT ^= MS5607_PIN;
}