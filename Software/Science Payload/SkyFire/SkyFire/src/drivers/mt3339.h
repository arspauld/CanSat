/*
 * mt3339.h
 *
 * Created: 4/11/2019 9:59:41 AM
 *  Author: Pat Smith
 */ 

#include <asf.h>

#ifndef MT3339_H_
#define MT3339_H_

#define GPS_TERMINAL_SERIAL				&USARTD1 //&USARTC0
#define GPS_TERMINAL_SERIAL_BAUDRATE	9600
#define GPS_TERMINAL_SERIAL_CHAR_LEN	USART_CHSIZE_8BIT_gc
#define GPS_TERMINAL_SERIAL_PARITY		USART_PMODE_DISABLED_gc
#define GPS_TERMINAL_SERIAL_STOP_BIT	true

#define GPS_TERMINAL_TX_PIN				PIN3_bm
#define GPS_TERMINAL_PORT				PORTC  //PORTC for data logging

#define GPS_UPDATE_RATE					"$PMTK220,1000*1F<CR><LF>\0"
#define GPS_NMEA_SENTENCE				"$PMTK314,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2D<CR><LF>\0"
#define GPS_NMEA_SENTENCE_2				"$PMTK514,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2B<CR><LF>\0"

void gps_uart_init(void);
void gps_command(char* data);
uint8_t gps_read(void);

#endif /* MT3339_H_ */