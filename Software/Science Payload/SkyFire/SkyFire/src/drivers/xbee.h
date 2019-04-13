/*
 * xbee.h
 *
 * Created: 4/12/2019 10:13:38 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>

#ifndef XBEE_H_
#define XBEE_H_


#define XBEE_TERMINAL_SERIAL			&USARTC0
#define XBEE_TERMINAL_SERIAL_BAUDRATE	9600
#define XBEE_TERMINAL_SERIAL_CHAR_LEN	USART_CHSIZE_8BIT_gc
#define XBEE_TERMINAL_SERIAL_PARITY		USART_PMODE_DISABLED_gc
#define XBEE_TERMINAL_SERIAL_STOP_BIT	true

#define XBEE_TERMINAL_TX_PIN			PIN3_bm
#define XBEE_TERMINAL_PORT				PORTC

void XBEE_uart_init(void);
void XBEE_write(char* data);
uint8_t XBEE_read(void);


#endif /* XBEE_H_ */