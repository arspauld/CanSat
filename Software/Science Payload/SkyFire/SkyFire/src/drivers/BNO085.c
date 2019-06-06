/*
 * BNO085.c
 *
 * Created: 6/4/2019 12:46:15 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>
#include "spi_controller.h"
#include "BNO085.h"

uint8_t SeqNum = 0;

void bno085_init(void){
	BNO085_SS_TERMINAL.OUT |= BNO085_SS_PIN;
		
	uint16_t length	= 0x0014;	// 20
	uint8_t data[] = {
		length & 0xFF,
		length >> 8,
		ROT_VECT_CHANNEL_NUM,
		SeqNum,
		ROT_VECT_FEAT_ID,
		ROT_VECT_FEAT_FLAGS,
		ROT_VECT_CHANGE_SENS & 0xFF,
		ROT_VECT_CHANGE_SENS >> 8,
		ROT_VECT_TIMER_INT & 0xFF,
		(ROT_VECT_TIMER_INT >> 8) & 0xFF,
		(ROT_VECT_TIMER_INT >> 16) & 0xFF,
		ROT_VECT_TIMER_INT >> 24,
		ROT_VECT_BATCH_INT & 0xFF,
		(ROT_VECT_BATCH_INT >> 8) & 0xFF,
		(ROT_VECT_BATCH_INT >> 16) & 0xFF,
		ROT_VECT_BATCH_INT >> 24,
		ROT_VECT_SENS_SPEC & 0xFF,
		(ROT_VECT_SENS_SPEC >> 8) & 0xFF,
		(ROT_VECT_SENS_SPEC >> 16) & 0xFF,
		ROT_VECT_SENS_SPEC >> 24,
	};
	
	spi_select(BNO085_SS_PIN);
	for(uint8_t i = 0; i < length; i++){
		spi_write(data[i]);
	}
	spi_select(BNO085_SS_PIN);
	
	
	sysclk_enable_peripheral_clock(&TCC0); // starts peripheral clock

	TCC0.CTRLA = 0x05; // divisor set to 64
	TCC0.PER = 25000; // 20 Hz
	TCC0.INTCTRLA = TC_OVFINTLVL_LO_gc; // CCA int flag Lo level
}

void bno085_read(uint8_t* data){
	spi_select(BNO085_SS_PIN);
	for(uint8_t i = 0; i < 18; i++){
		data[i] = spi_read();
	}
	spi_select(BNO085_SS_PIN);
}