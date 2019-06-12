/*
 * voltage.c
 *
 * Created: 4/14/2019 12:42:36 AM
 *  Author: Pat Smith
 */ 

#include "voltage.h"
#include <asf.h>


void voltage_init(void)
{
	sysclk_enable_peripheral_clock(&VOLTAGE_SERIAL);
	VOLTAGE_SERIAL.CTRLA = 0x01; // enable the ADC
	VOLTAGE_SERIAL.CTRLB = 0x00; // unsigned 12 bit mode
	VOLTAGE_SERIAL.REFCTRL = 0x10; // voltage Reference of Vcc/1.6V
	VOLTAGE_SERIAL.PRESCALER = 0x05; // prescaler of DIV128 on the clock
	VOLTAGE_SERIAL.CAL = adc_get_calibration_data(ADC_CAL_ADCA); // retrieve stored calibration data about the ADC
	
	VOLTAGE_CHANNEL.CTRL = 0x01; // single ended input
	VOLTAGE_CHANNEL.MUXCTRL = VOLTAGE_PIN; // reading ADCA pin 0
}

uint16_t voltage_read(void)
{
	VOLTAGE_CHANNEL.CTRL |= 0x80; // start the conversion
	while(!(VOLTAGE_CHANNEL.INTFLAGS)); // wait until the conversion is done
	return VOLTAGE_CHANNEL.RES; // return the 12-bit result as a uint16_t
}