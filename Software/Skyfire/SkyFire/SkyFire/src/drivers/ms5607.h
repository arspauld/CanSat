/*
 * ms5607.h
 *
 * Created: 9/3/2018 12:51:48 PM
 *  Author: cellis
 */ 


#ifndef MS5607_H_
#define MS5607_H_

#define CMD_MS5607_READ_ADC		0x00
#define CMD_MS5607_D1_4096		0x48		// 4096 oversampling
#define CMD_MS5607_D1_2048		0x46		// 2048 oversampling
#define CMD_MS5607_D1_1024		0x44		// 1024 oversampling
#define CMD_MS5607_D2_4096		0x58
#define CMD_MS5607_D2_2048		0x56
#define CMD_MS5607_D2_1024		0x54

#define CMD_MS5607_RESET		0x1E
#define CMD_MS5607_READ_C1		0xA2
#define CMD_MS5607_READ_C2		0xA4
#define CMD_MS5607_READ_C3		0xA6
#define CMD_MS5607_READ_C4		0xA8
#define CMD_MS5607_READ_C5		0xAA
#define CMD_MS5607_READ_C6		0xAC

#define MS5607_PORT				PORTC
#define MS5607_PIN				0x10

void ms5607_init(void);

uint16_t ms5607_read(uint16_t port);
uint32_t ms5607_convert_d1(void);
uint32_t ms5607_convert_d2(void);

void ms5607_write_unprotected(uint8_t comm);
uint32_t ms5607_read_unprotected(void);
void flip_ms5607(void);

#endif /* MS5607_H_ */