/*
 * adc.h
 *
 * Created: 9/3/2018 2:11:29 PM
 *  Author: Sean
 */ 


#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include <asf.h>

#define THERMISTOR_SERIAL		ADCA
#define THERMISTOR_CHANNEL		CH0
#define THERMISTOR_PIN			0x00


void thermistor_init(void);
uint16_t thermistor_read(void);

#endif /* THERMISTOR_H_ */