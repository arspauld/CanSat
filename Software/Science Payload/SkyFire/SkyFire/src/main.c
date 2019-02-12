#include <asf.h>
#include <math.h>

double get_pressure(void);
double get_temperature(void);
double get_altitude(double press);

uint8_t state = 0;
double ground_t = 288.15;		// 288.15 K is hundredths of Kelvin
double ground_p = 101325;		// Pascals standard sea level pressure


int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	board_init();
	while(1){
		get_pressure();
	}

	/* Insert application code here, after the board has been initialized. */
}

double get_pressure(void){
	uint16_t val = 0;
	uint16_t call = 0x54;
	val = ms5607_read_adc(call); // hundredths of mbar pressure
	return double(val) / 100;
}

double get_temperature(void){
	uint16_t val = 0;
	//////Call NTCLE/////
	return double(val);
}

double get_altitude(double press){
	double val = 0;
	double R = 287.0578987;
	double L = 0.0065;
	double g_0 = 9.81;
	val = ground_t * (1 - pow(press / ground_p, R * L / g_0)) / L;
	return val;
}