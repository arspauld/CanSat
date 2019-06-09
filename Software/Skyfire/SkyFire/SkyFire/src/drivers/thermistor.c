/*
 * adc.c
 *
 * Created: 9/3/2018 2:11:20 PM
 *  Author: swidmier
 */ 

#include <asf.h>
#include "thermistor.h"

void thermistor_init(void)
{
	sysclk_enable_peripheral_clock(&THERMISTOR_SERIAL);
	THERMISTOR_SERIAL.CTRLA = 0x01; // enable the ADC
	THERMISTOR_SERIAL.CTRLB = 0x00; // unsigned 12 bit mode
	THERMISTOR_SERIAL.REFCTRL = 0x10; // voltage Reference of Vcc/1.6V
	THERMISTOR_SERIAL.PRESCALER = 0x05; // prescaler of DIV128 on the clock
	THERMISTOR_SERIAL.CAL = adc_get_calibration_data(ADC_CAL_ADCA); // retrieve stored calibration data about the ADC
	
	THERMISTOR_SERIAL.THERMISTOR_CHANNEL.CTRL = 0x01; // single ended input
	THERMISTOR_SERIAL.THERMISTOR_CHANNEL.MUXCTRL = THERMISTOR_PIN; // reading ADCA pin 0	
}

uint16_t thermistor_read(void)
{
	THERMISTOR_SERIAL.THERMISTOR_CHANNEL.CTRL |= 0x80; // start the conversion
	while(!(THERMISTOR_SERIAL.THERMISTOR_CHANNEL.INTFLAGS)); // wait until the conversion is done
	return THERMISTOR_SERIAL.THERMISTOR_CHANNEL.RES; // return the 12-bit result as a uint16_t
}