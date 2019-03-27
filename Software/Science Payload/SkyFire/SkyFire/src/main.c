#include <asf.h>
#include <math.h>
#include "uart.h"
#include "adc.h"
#include "ms5607.h"
#include "spi_controller.h"
#include "RingBuffer.h"

///////////////////////// Function Prototypes //////////////////////////
void system_init(void);
double get_pressure(void);
double get_temperature(void);
double get_altitude(double press);


/////////////////////////// Global Variables ///////////////////////////
uint8_t state = 0;
double ground_p = 101325;		// Pascals standard sea level pressure
double ground_t = 288.15;		// 288.15 K is Kelvin 15 C
double ground_a = 0;			// Assumes ground altitude at launch

// Altitude calculation variables
double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.81;

////////////////////////////// Functions ///////////////////////////////
int main (void)
{
	system_init();
	while(1){
		// Check Sensors
		double press = get_pressure();
		double temp = get_temperature();
		double alt = get_altitude(press);
		
		//
	}

	/* Insert application code here, after the board has been initialized. */
}


// Sensor functions
void system_init(void){
	// Initialization of systems
	sysclk_init();
	uart_terminal_init();
	adc_init();
	spi_init();
	
	ground_p = get_pressure();
	ground_t = get_temperature();
	ground_a = get_altitude(ground_p);
}

double get_pressure(void){
	uint16_t val = 0;
	uint16_t call = 0x54;
	val = ms5607_read_adc(call); // 1/100th Pa pressure
	return double(val) / 100;	// returns pressure in Pa
}

double get_temperature(void){
	double val = 0;
	uint16_t reading = adc_read();
	double voltage = (.000495 * reading + .5016); // m and b are collected from testing
	double resistance = 6720 * (3.3 - voltage) / voltage; // 6720 is the resistance of the steady resistor
	val = (uint16_t) (100 / (3.354016E-3 + 2.569850E-4 * log(resistance / 10000) + 2.620131E-6 * pow(log(resistance / 10000), 2) + 6.383091E-8 * pow(log(resistance / 10000), 3))); // returns the temperature in hundredths of kelvin
	return val;
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return val;		//returns altitude in meters
}