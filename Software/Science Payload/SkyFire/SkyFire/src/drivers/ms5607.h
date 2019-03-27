/*
 * ms5607.h
 *
 * Created: 9/3/2018 12:51:48 PM
 *  Author: cellis
 */ 


#ifndef MS5607_H_
#define MS5607_H_


uint16_t ms5607_read(uint16_t port);
uint32_t ms5607_convert_d1(void);
uint32_t ms5607_convert_d2(void);


#endif /* MS5607_H_ */