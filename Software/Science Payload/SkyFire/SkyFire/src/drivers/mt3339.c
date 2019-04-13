/*
 * CFile1.c
 *
 * Created: 4/11/2019 9:59:04 AM
 *  Author: Pat Smith
 */ 

#include <asf.h>
#include "mt3339.h"

void gps_uart_init(void){
	sysclk_enable_peripheral_clock(GPS_TERMINAL_SERIAL);	// enable the USART's clock
	// initialize a configuration struct with USART settings
	static usart_serial_options_t gps_config = {
		.baudrate	=	GPS_TERMINAL_SERIAL_BAUDRATE,
		.charlength =	GPS_TERMINAL_SERIAL_CHAR_LEN,
		.paritytype =	GPS_TERMINAL_SERIAL_PARITY,
		.stopbits	=	GPS_TERMINAL_SERIAL_STOP_BIT
	};
	
	GPS_TERMINAL_PORT.DIR |= GPS_TERMINAL_TX_PIN; // Puts pin to output
	
	usart_serial_init(GPS_TERMINAL_SERIAL,&gps_config);
}

void gps_command(uint8_t* data){
	usart_serial_write_packet(GPS_TERMINAL_SERIAL, data, sizeof(data));
}

void gps_read_GPGGA(uint8_t* data){
	usart_serial_read_packet(GPS_TERMINAL_SERIAL, data, 70);
}