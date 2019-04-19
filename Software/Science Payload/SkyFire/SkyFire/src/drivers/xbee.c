/*
 * xbee.c
 *
 * Created: 4/12/2019 10:13:22 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>
#include <string.h>
#include "xbee.h"

void XBEE_uart_init(void){
	sysclk_enable_peripheral_clock(XBEE_TERMINAL_SERIAL);	// enable the USART's clock
	// initialize a configuration struct with USART settings
	static usart_serial_options_t XBEE_config = {
		.baudrate	=	XBEE_TERMINAL_SERIAL_BAUDRATE,
		.charlength =	XBEE_TERMINAL_SERIAL_CHAR_LEN,
		.paritytype =	XBEE_TERMINAL_SERIAL_PARITY,
		.stopbits	=	XBEE_TERMINAL_SERIAL_STOP_BIT
	};
	
	XBEE_TERMINAL_PORT.DIR |= XBEE_TERMINAL_TX_PIN; // Puts pin to output
	
	usart_serial_init(XBEE_TERMINAL_SERIAL,&XBEE_config);
}

void XBEE_write(char* data){
	for(uint8_t i = 0; i < strlen(data); i++){
		usart_putchar(XBEE_TERMINAL_SERIAL, (uint8_t) data[i]);
	}
	//usart_putchar(XBEE_TERMINAL_SERIAL, 10);
}

uint8_t XBEE_read(void){
	uint8_t c = usart_getchar(XBEE_TERMINAL_SERIAL);
	return c;
}