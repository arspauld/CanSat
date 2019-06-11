/*
 * hall.c
 *
 * Created: 5/13/2019 2:44:29 PM
 *  Author: Pat Smith
 */ 

#include <asf.h>
#include "hall.h"
#include <string.h>

#define VOLTAGE_SCALE_FACTOR 62

void hall_sensor_init(void){
	struct ac_config aca_config;
	
	memset(&aca_config, 0, sizeof(struct ac_config));
	
	ac_set_mode(&aca_config, AC_MODE_SINGLE);
	ac_set_hysteresis(&aca_config, AC_HYSMODE_LARGE_gc);
	ac_set_voltage_scaler(&aca_config, VOLTAGE_SCALE_FACTOR);
	ac_set_interrupt_mode(&aca_config, AC_INT_MODE_RISING_EDGE);
	ac_set_positive_reference(&aca_config, AC_MUXNEG_SCALER_gc);
	ac_set_negative_reference(&aca_config, AC_MUXPOS_PIN5_gc);
	
	ac_set_interrupt_callback(&aca_config, hall_sensor_measure);
	
	ac_write_config(&ACA, 0, &aca_config);
	
	ac_enable(&ACA, 0);
	
	cpu_irq_enable();
	
}