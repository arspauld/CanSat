/*
 * spi_controller.c
 *
 * Created: 9/3/2018 12:50:35 PM
 *  Author: cellis
 */ 

#include <asf.h>
#include "spi_controller.h"

void spi_init(void)
{
	PORTC.DIR |= 0b10110000; 
	sysclk_enable_peripheral_clock(&SPIC);
	SPIC.CTRL = 0x50; //enables SPI and puts a prescaler of 16 idling high and transmitting MSB first on rising signal
}

void spi_select(uint8_t port)
{
	PORTC.OUT ^= port; //switches SS
}

volatile uint8_t spi_read(void)
{
	uint8_t oldInterruptState = SREG;
	cli();
	SPIC.DATA = 0xFF; // make the DATA register something we know
	while(!(SPIC.STATUS>>7));
	sei();
	SREG=oldInterruptState;
	return SPIC.DATA; // return the data from this function
}

void spi_write(uint8_t data)
{
	uint8_t oldInterruptState = SREG;
	cli();
	SPIC.DATA = data; // write the data we want to send to the data register
	while(!(SPIC.STATUS>>7)); // wait to ensure the data is sent before we do anything else
	sei();
	SREG=oldInterruptState;
}