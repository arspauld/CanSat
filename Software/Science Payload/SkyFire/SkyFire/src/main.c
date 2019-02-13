#include <asf.h>
#include <math.h>
#include "uart.h"

void system_init(void);
double get_pressure(void);
double get_temperature(void);
double get_altitude(double press);

uint8_t state = 0;
double ground_t = 288.15;		// 288.15 K is hundredths of Kelvin
double ground_p = 101325;		// Pascals standard sea level pressure

double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.81;


int main (void)
{
	system_init();
	while(1){
		// Check Sensors
		get_pressure()
		
		//
	}

	/* Insert application code here, after the board has been initialized. */
}

void system_init(void){
	sysclk_init();
	uart_terminal_init();
}

double get_pressure(void){
	uint16_t val = 0;
	uint16_t call = 0x54;
	val = ms5607_read_adc(call); // 1/100th Pa pressure
	return double(val) / 100;	// returns pressure in Pa
}

double get_temperature(void){
	uint16_t val = 0;
	//////Call NTCLE/////
	return double(val);
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return val;		//returns altitude in meters
}