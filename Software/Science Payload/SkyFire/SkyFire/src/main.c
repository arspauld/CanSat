#include <asf.h>
#include <math.h>
#include "drivers\uart.h"
#include "drivers\adc.h"
#include "drivers\ms5607.h"
#include "drivers\spi_controller.h"
#include "tools\RingBuffer.h"

// GCS Commands
#define RESET				0xFF
#define CALIBRATE			0xEE
#define CALIBRATE_CAMERA	0xDD
#define CALIBRATE_ALTITUDE  0xCC
#define CALIBRATE_ANGLE		0xBB
#define GPS					0xAA


///////////////////////// Function Prototypes //////////////////////////
void system_init(void);				// Starts the system
void ms5607_init(void);				// Collects the sensors constants
double get_pressure(void);			// kilo Pascals
double get_temperature(void);		// Kelvin
double get_altitude(double press);	// meters
double get_velocity(RingBuffer16_t* altitudes, uint8_t frequency);		// Approximates velocity of CanSat
void report(double alt, double temp, double press);
void record(double alt, double temp, double press);
void transmit(double alt, double temp, double press);


/////////////////////////// Global Variables ///////////////////////////
uint8_t state = 0;
double ground_p = 101.325;		// Pascals standard sea level pressure
double ground_t = 288.15;		// 288.15 K is Kelvin 15 C
double ground_a = 0;			// Assumes ground altitude at launch

// Altitude calculation variables
double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.80665;

// Pressure Calculation variables
uint16_t c[] = {0,0,0,0,0,0};


////////////////////////////// Functions ///////////////////////////////
int main (void)
{
	system_init();
	
	int16_t alt_array[10];
	RingBuffer16_t altitudes;	// in centimeters
	rb16_init(&altitudes, alt_array, (uint16_t) 10);
	
	/*
	uint8_t mem_array[10];
	RingBufferu8_t gcs_comms;
	rbu8_init(&gcs_comms, mem_array, 10);
	*/
	
	printf("Initialized\n");
	
	while(1){
		//Gives each flight state their unique tasks
		switch(state){
			case 0:
				break;
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			default:
				state = 0;
				break;
		}
		
		
		// Check Sensors
		double press = get_pressure();
		double temp = 288.15; //get_temperature();
		double alt = get_altitude(press);
		int16_t a[] = {(int16_t) (alt*100)};
		rb16_write(&altitudes, &a, 1);
		printf("%i\n", a[0]);
		double velocity = get_velocity(&altitudes, 2);
		//printf("%li\n",(int32_t)press);
		printf("Pressure (Pa): %li, Altitude (cm): %i, Velocity (cm/s): %i\n", (int32_t) (press*10), (int16_t) (alt * 100), (int16_t) velocity);
		delay_ms(500);
	}
}


// Sensor functions
void system_init(void){
	// Initialization of systems
	sysclk_init(); // initializes the system clock
	delay_ms(2); // delays the rest of the processes to ensure a started clock
	
	// Initialization of pins
	PORTC.DIR = 0xBB; // makes Port C have pins, 7, 5, 4, 3, 1, 0 be output (0b10111011)
	PORTC.OUT = 0x10; // makes the 4th pin on Port C be set on high (0b00010000)
	PMIC.CTRL = PMIC_LOLVLEN_bm; // enables lo level interrupts
	
	// Driver Initialization
	uart_terminal_init();
	delay_ms(2);
	
//	adc_init();
	delay_ms(2);
	
	spi_init();
	delay_ms(2);
	
	ms5607_init();
	delay_ms(2);
	
	
	// Initialization of variables
	ground_p = get_pressure();
	//ground_t = get_temperature();
	ground_a = get_altitude(ground_p);
}

void ms5607_init(void){
	// Resets the ms5607
	spi_select(0x10);
	spi_write(0x1E);
	delay_ms(3);
	spi_select(0x10);
	delay_ms(2);
	
	// records the constants
	c[0] = ms5607_read(0xA2);
	c[1] = ms5607_read(0xA4);
	c[2] = ms5607_read(0xA6);
	c[3] = ms5607_read(0xA8);
	c[4] = ms5607_read(0xAA);
	c[5] = ms5607_read(0xAC);
	
	//printf("%u,%u,%u,%u,%u,%u\n",c[0],c[1],c[2],c[3],c[4],c[5]);
}

double get_pressure(void){
	double val = 0;
	uint32_t d1 = ms5607_convert_d1();
	uint32_t d2 = ms5607_convert_d2();
	//printf("%li,%li\n",d1,d2);
	double dt = d2 - (uint64_t)c[4] * 256;
	double off = (uint64_t) c[1] * 131072 + ((uint64_t) c[3] * dt) / 64;
	double sens = (uint64_t) c[0] * 65536 + ((uint64_t) c[2] * dt) / 128;
	val = (double) (((uint64_t) d1 * sens / 2097152 - off) / 32768);
	return (val);	// returns pressure in Pa
}

double get_temperature(void){
	double val = 0;
	uint16_t reading = adc_read();
	double voltage = (.000495 * reading + .5016); // m and b are collected from testing
	double resistance = 6720 * (3.3 - voltage) / voltage; // 6720 is the resistance of the steady resistor
	val = (uint16_t) (100 / (3.354016E-3 + 2.569850E-4 * log(resistance / 10000) + 2.620131E-6 * pow(log(resistance / 10000), 2) + 6.383091E-8 * pow(log(resistance / 10000), 3))); // returns the temperature in hundredths of kelvin
	return val / 100.0; //returns the temperature in kelvin
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return val;		//returns altitude in meters
}

// Approximates the Velocity from past five altitudes
double get_velocity(RingBuffer16_t* altitudes, uint8_t frequency){
	double vel = 0;
	for(uint16_t i = 0; i < 5; i++){
		int16_t new = rb16_get_nth(altitudes,i);
		int16_t old = rb16_get_nth(altitudes,i+1);
		vel += (double) ((new - old) * frequency);
		printf("Old: %5i, New: %5i, Vel: %5i\n", old, new, (int16_t) (vel/5.0));
	}
	vel /= 5.0;
	return vel;
}

void report(double alt, double temp, double press){
	record(alt,temp,press);
	transmit(alt,temp,press);
}

void record(double alt, double temp, double press){
	printf("5343, %8.4f, %8.4f, %8.4f\n", alt, temp, press);
}

void transmit(double alt, double temp, double press){
	 uint8_t a = 0;
	 a = 1;
}