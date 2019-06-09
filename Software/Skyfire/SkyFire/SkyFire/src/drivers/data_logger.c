/*
 * uart.c
 *
 * Created: 9/3/2018 12:22:41 PM
 *  Author: swidmier
 */ 

#include <asf.h>
#include <string.h>
#include "data_logger.h"

void data_terminal_init()
{
	sysclk_enable_peripheral_clock(UART_TERMINAL_SERIAL);	// enable the USART's clock
	// initialize a configuration struct with USART settings
	static usart_serial_options_t usart_config = {
		.baudrate	=	UART_TERMINAL_SERIAL_BAUDRATE,
		.charlength =	UART_TERMINAL_SERIAL_CHAR_LEN,
		.paritytype =	UART_TERMINAL_SERIAL_PARITY,
		.stopbits	=	UART_TERMINAL_SERIAL_STOP_BIT
	};
	
	UART_TERMINAL_PORT.DIR |= UART_TERMINAL_TX_PIN;	// set the USART transmit pin to output
	
	//usart_serial_init(UART_TERMINAL_SERIAL, &usart_config);
	stdio_serial_init(UART_TERMINAL_SERIAL, &usart_config); // function maps the serial output to printf, not necessary to know how it works
}

void uart_write(char* data){
	for(uint8_t i = 0; i < strlen(data); i++){
		usart_putchar(UART_TERMINAL_SERIAL, (uint8_t) data[i]);
	}
	//usart_putchar(XBEE_TERMINAL_SERIAL, 10);
}