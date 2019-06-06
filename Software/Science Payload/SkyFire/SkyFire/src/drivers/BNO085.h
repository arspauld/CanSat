/*
 * BNO085.h
 *
 * Created: 6/4/2019 12:46:32 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>

#ifndef BNO085_H_
#define BNO085_H_

#define BNO085_SPI_PORT			SPIC
#define BNO085_SS_TERMINAL		PORTC
#define BNO085_SS_PIN			PIN3_bm

#define ROT_VECT_CHANNEL_NUM	0x02

#define ROT_VECT_FEAT_ID		0x05				// Rotational Vector
#define ROT_VECT_FEAT_FLAGS		0x0B				// Wake Up and always on
#define ROT_VECT_CHANGE_SENS	0x0000				// No change to the sensitivity
#define ROT_VECT_TIMER_INT		0x0000C350			// 50 ms intervals
#define ROT_VECT_BATCH_INT		0x000186A0			// 100 ms time out
#define ROT_VECT_SENS_SPEC		0x00000000			// Nada

#define QSCALE					1.0/(0x01<<14)		// Q point of the data

void bno085_init(void);
void bno085_read(uint8_t* data);

#endif /* BNO085_H_ */