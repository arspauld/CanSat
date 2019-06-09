/*
 * xbee.c
 *
 * Created: 4/12/2019 10:13:22 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>
#include <string.h>
#include "spi_controller.h"
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

void XBEE_spi_init(void){
	XBEE_SS_PORT.DIR |= XBEE_SS_PIN;
	XBEE_SS_PORT.OUT |= XBEE_SS_PIN;
	
	
}

void XBEE_uart_write(char* data){
	for(uint8_t i = 0; i < strlen(data); i++){
		usart_putchar(XBEE_TERMINAL_SERIAL, (uint8_t) data[i]);
	}
	//usart_putchar(XBEE_TERMINAL_SERIAL, 10);
}

uint8_t XBEE_uart_read(void){
	uint8_t c = usart_getchar(XBEE_TERMINAL_SERIAL);
	return c;
}

void XBEE_spi_write(char* data){
	flip_XBEE();
	for(uint8_t i = 0; i < strlen(data); i++){
		spi_write((uint8_t) data[i]);
	}
	//delay_ms(5);
	flip_XBEE();
	//usart_putchar(XBEE_TERMINAL_SERIAL, 10);
}

uint8_t XBEE_spi_read(void){
	flip_XBEE();
	uint8_t data = spi_read();
	flip_XBEE();
	return data;
}

void flip_XBEE(void){
	XBEE_SS_PORT.OUT ^= XBEE_SS_PIN;
}