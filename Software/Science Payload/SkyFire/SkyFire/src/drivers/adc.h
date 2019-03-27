/*
 * adc.h
 *
 * Created: 9/3/2018 2:11:29 PM
 *  Author: Sean
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <asf.h>

void adc_init(void);
uint16_t adc_read(void);

#endif /* ADC_H_ */