/*
 * voltage.h
 *
 * Created: 4/14/2019 12:42:50 AM
 *  Author: Pat Smith
 */ 

#include <asf.h>

#ifndef VOLTAGE_H_
#define VOLTAGE_H_

#define VOLTAGE_SERIAL		ADCA
#define VOLTAGE_CHANNEL		CH0
#define VOLTAGE_PIN			0x80 // Pin 7


void voltage_init(void);
uint16_t voltage_read(void);

#endif /* VOLTAGE_H_ */