#include <asf.h>
#include <math.h>
#include <string.h>
#include "drivers\uart.h"
#include "drivers\thermistor.h"
#include "drivers\ms5607.h"
#include "drivers\mt3339.h"
#include "drivers\xbee.h"
#include "drivers\spi_controller.h"
#include "drivers\spy_cam.h"
#include "drivers\voltage.h"
#include "tools\RingBuffer.h"

// GCS Commands
#define RESET				0xFF
#define CALIBRATE			0xEE
#define CALIBRATE_CAMERA	0xDD
#define CALIBRATE_ALTITUDE  0xCC
#define CALIBRATE_ANGLE		0xBB
#define SEND_GPS_LOCATION	0xAA

// Tolerances for Flight State
#define EPSILON_VELOCITY	5
#define EPSILON_ALTITUDE	10


///////////////////////// Function Prototypes //////////////////////////
void	system_init(void);												// Starts the system
void	pressure_init(void);											// Collects the sensors constants
void	gps_init(void);													// Starts the GPS
void	xbee_init(void);												// Starts the Xbee
double	get_pressure(void);												// Pascals
double	get_temperature(void);											// Celsius
double	get_altitude(double press);										// meters
double	get_velocity(RingBuffer16_t* altitudes, uint8_t frequency);		// Approximates velocity of CanSat
void	data_collect(RingBuffer16_t* alts, RingBuffer32_t* presses);	// Handles data collection
double	data_check(RingBuffer32_t* presses);							// Function that averages values and compares with stdev
void	state_check(void);												// Returns the current state
void	reset_ground(void);												// Resets the ground variables
void	servo_timer_init(void);											// Starts PWM wave for fin servos
void	servo_timer_alt(void);											// Function that alters the position of the fins
void	clock_init(void);												// Starts a timer to count the seconds


/////////////////////////// Global Variables ///////////////////////////
uint8_t state = 0;
uint8_t released = 0;
double ground_p = 101325;		// Pascals standard sea level pressure
double ground_t = 288.15;		// 15 C
double ground_a = 0;			// Assumes ground altitude at launch

// Altitude calculation variables
double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.80665;

// Pressure Calculation variables
uint16_t c[] = {0,0,0,0,0,0};

// Fin Servo
uint16_t servo_on_time_us = 1500;

// GPS Stuff
uint8_t gps[70];			// GPS sentences
uint8_t writing;
uint8_t pos;

// Output string
char str[100];					// Output String

// Time and Packets
uint16_t timer = 0;
uint16_t packets = 0;
uint16_t rate = 10;

// Initializes variables
double press = 0;			// Pressure (Pa)
double temp = 0;			// Temperature (C)
double alt = 0;				// Altitude (m)
double volt = 0;			// Battery Terminal Voltage (V)
double velocity = 0;		// Velocity (cm/s)
double gps_t = 0;			// GPS Time
double gps_lat = 0;			// GPS Latitude (+:N,-:S)
double gps_long = 0;		// GPS Longitude (+:E,-:W)
double gps_alt = 0;			// GPS Altitude
uint8_t gps_sats = 0;		// GPS Satellites
double pitch = 0;			// Pitch Angle
double roll = 0;			// Roll Angle
double rpm = 0;				// Calculate RPM of Blades
double angle = 0;			// Angle of Bonus Direction	


////////////////////////////// Functions ///////////////////////////////
int main (void)
{
	system_init();
	
	//printf("Initialized\n");
	cam_switch();
	
	int16_t alt_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer16_t altitudes;	// in centimeters
	rb16_init(&altitudes, alt_array, (uint16_t) 10);
	
	int32_t press_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer32_t pressures;	// in Pascals / 10
	rb32_init(&pressures, press_array, (uint16_t) 10);
	
	char* format = "5343,%i,%i,%i,%li,%i,%i,%li,%li,%li,%i,%i,%i,%i,%i,%i,%i\n\0";
	
	while(1){
		// Check Sensors
		data_collect(&altitudes,&pressures);
		
		// Checks State
		state_check();
		
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
		
		packets++;
		if(timer != 0){
			rate = packets / timer;
		}
		// Prints information
		//printf("5343,%i,%i,%i,%li,%i,%i,%li,%li,%li,%i,%i,%i,%i,%i,%i,%i",time,packets,(int16_t)alt*10,(int32_t) press,(int16_t) temp*10,volt,gps_t,gps_lat,gps_long,gps_alt,gps_sats,pitch,roll,rpm,state,angle)
		sprintf(str,format,timer,packets,(int16_t)(alt),(int32_t) press,(int16_t)(temp-273.15),(int16_t)volt,(int32_t)gps_t,(int32_t)gps_lat,(int32_t)gps_long,(int16_t)gps_alt,(int16_t)gps_sats,(int16_t)pitch,(int16_t)roll,(int16_t)rpm,state,(int16_t)angle); // Data Logging Test
		//printf(str);
	}
}


// Sensor functions
void system_init(void){
	// Initialization of systems
	sysclk_init(); // initializes the system clock
	delay_ms(2); // delays the rest of the processes to ensure a started clock
	sei();
	
	// Initialization of pins
	PORTC.DIR = 0xBB; // makes Port C have pins, 7, 5, 4, 3, 1, 0 be output (0b10111011)
	PORTC.OUT = 0x10; // makes the 4th pin on Port C be set on high (0b00010000)
	PMIC.CTRL = PMIC_LOLVLEN_bm; // enables lo level interrupts
	
	// Driver Initialization
	data_terminal_init();
	delay_ms(2);
	
//	thermistor_init();
	delay_ms(2);
	
	spi_init();
	delay_ms(2);
	
	pressure_init();
	delay_ms(2);
	
	//xbee_init();
	//gps_init();
	
	clock_init();
    //servo_timer_init();
	cam_init();
	
	delay_ms(10);
	
	// Initialization of variables
	ground_p = get_pressure();
	ground_t = get_temperature();
	ground_a = get_altitude(ground_p);
}

void pressure_init(void){
	ms5607_init();
	
	// records the constants
	c[0] = ms5607_read(CMD_MS5607_READ_C1);
	c[1] = ms5607_read(CMD_MS5607_READ_C2);
	c[2] = ms5607_read(CMD_MS5607_READ_C3);
	c[3] = ms5607_read(CMD_MS5607_READ_C4);
	c[4] = ms5607_read(CMD_MS5607_READ_C5);
	c[5] = ms5607_read(CMD_MS5607_READ_C6);
	//printf("%u,%u,%u,%u,%u,%u\n",c[0],c[1],c[2],c[3],c[4],c[5]);
}

void gps_init(void){
	gps_uart_init();				// Starts the GPS
	delay_ms(2);

	gps_command(GPS_UPDATE_RATE);	// Changes the update rate of the GPS
	delay_ms(2);
	gps_command(GPS_NMEA_SENTENCE);	// Changes the output NMEA sentences of the GPS
	delay_ms(2);
	gps_command(GPS_NMEA_SENTENCE_2);
	
	(*GPS_TERMINAL_SERIAL).CTRLA = USART_RXCINTLVL_LO_gc;
}

void xbee_init(void){
	XBEE_uart_init();				// Starts the GPS
	delay_ms(2);
	
	(*XBEE_TERMINAL_SERIAL).CTRLA = USART_RXCINTLVL_LO_gc;
}

double get_pressure(void){
	double val = 101325;
	/*
	uint32_t d1 = ms5607_convert_d1();
	uint32_t d2 = ms5607_convert_d2();
	//printf("%li,%li\n",d1,d2);
	double dt = d2 - (uint64_t)c[4] * 256.0;
	//double temp = 20.0 + ((uint64_t) dt * c[5]) / 8388608.0;
	double off = (uint64_t) c[1] * 131072.0 + ((uint64_t) c[3] * dt) / 64.0;
	double sens = (uint64_t) c[0] * 65536.0 + ((uint64_t) c[2] * dt) / 128.0;
	
	if(temp < 20){
		double t2 = ((uint64_t) dt * dt) / 2147483648.0;
		double off2 = 61 * pow((temp - 2000),2.0) / 16.0;
		double sens2 = 2 * pow(temp - 2000, 2.0);
		
		temp -= t2;
		off -= off2;
		sens -= sens2;
	}
	
	
	val = (double) (((uint64_t) d1 * sens / 2097152 - off) / 32768);
	*/
	return val;	// returns pressure in Pa
}

double get_temperature(void){
	double val = 288.15;
	/*
	uint16_t reading = adc_read();
	double voltage = (.000495 * reading + .5016); // m and b are collected from testing
	double resistance = 6720 * (3.3 - voltage) / voltage; // 6720 is the resistance of the steady resistor
	val = (uint16_t) (100 / (3.354016E-3 + 2.569850E-4 * log(resistance / 10000) + 2.620131E-6 * pow(log(resistance / 10000), 2) + 6.383091E-8 * pow(log(resistance / 10000), 3))); // returns the temperature in hundredths of kelvin
	*/
	return val; //returns the temperature in kelvin
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return (val-ground_a);		//returns altitude in meters
}

// Approximates the Velocity from past five altitudes
uint8_t data_samples = 3;
double get_velocity(RingBuffer16_t* altitudes, uint8_t frequency){
	double vel = 0;
	for(uint16_t i = 0; i < data_samples; i++){
		int32_t new = rb16_get_nth(altitudes,i);
		int32_t old = rb16_get_nth(altitudes,i+1);
		int32_t oldest = rb16_get_nth(altitudes,i+2);
		vel += ((3*new - 4*old + oldest) * frequency / 2.0); // O(h^2) approximation of backwards derivative (Thanks MAE284, you're good for something)
	}
	vel /= data_samples;
	return (vel/100);
}

void data_collect(RingBuffer16_t* alts, RingBuffer32_t* presses){
	double p = get_pressure();
	int32_t p_i = (int32_t) (p * 10);
	rb32_write(presses,&p_i,1);
	double p_s = data_check(presses)/10.0;
	
	if(p_s != -1){
		press = p_s;					// Pulls middle value of pressures
		alt	= get_altitude(p_s);	// Uses the corrected pressure to calculate the altitude
		
		// Stores Altitude
		int16_t a = (int16_t) (alt*100);
		rb16_write(alts, &a, 1); // Writes altitude in buffer
		
		// Calculates Velocity
		velocity = get_velocity(alts, rate);
	}
	temp = get_temperature();	// Grabs the temperature once
}

double data_check(RingBuffer32_t* presses){
	// Calculates average of data
	uint8_t length = 5;
	double mean = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 100000 && p < 10000000){ // Checks to make certain value makes sense
			mean += ((double) p)/length;
		}
	}
	
	// Calculates standard deviation of data
	double stdev = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 100000 && p < 10000000){
			stdev += pow(p-mean, 2);
		}
	}
	stdev /= (length - 1);
	
	// Throws out data that is farther than standard deviation from average
	double result = 0;
	uint8_t nums = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 100000 && p < 10000000){
			if(abs(p - mean) <= stdev){
				result += p;
				nums++;
			}
		}
	}
	
	if(nums == 0){
		result = -1;
	}
	else{
		result /= (double) nums; // Averages the new values
	}
	return result;
}

void state_check(void){
	if(abs(velocity)>EPSILON_VELOCITY){
		state = 1;
		if(velocity < 0){
			state = 2;
		}
	}
	else{
		state = 0;
		if(alt > 50){
			state = 1;
		}
		if(released){	// only change this to true in flight state 2
			state = 3;
		}
	}
}

void reset_ground(void){
	ground_p = get_pressure();
	ground_a = get_altitude(ground_p);
	ground_t = get_temperature();
}

void servo_timer_init(void){
	sysclk_enable_peripheral_clock(&TCD0); //enables peripheral clock for TC E0
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES); //necessary jumbo

	TCD1.CTRLA = 0x05; // sets the clock's divisor to 64
	TCD1.CTRLB = 0x13; // enables CCA and Single Waveform
	TCD1.PER = 10000; // sets the period (or the TOP value) to the period
	TCD1.CCA = (uint16_t) ((TCD1.PER) * (servo_on_time_us / 1000.0)); // makes the waveform be created for a duty cycle
}

void servo_timer_alt(void){
	TCD1.CCA = (uint16_t) ((TCD0.PER) * (servo_on_time_us / 1000.0)); // makes the waveform be created for a duty cycle
}

void clock_init(void){
	sysclk_enable_peripheral_clock(&TCE0); // starts peripheral clock

	TCE0.CTRLA = 0x07; // divisor set to 1024
	TCE0.PER = 31249; // 1 Hz
	TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc; // CCA int flag Lo level
}

ISR(TCE0_OVF_vect){
	timer++;
	printf(str);
	XBEE_write(str);
}

ISR(USARTC0_RXC_vect){
	uint8_t c = usart_getchar(XBEE_TERMINAL_SERIAL);
	printf("%c", c);
	/*
	switch(c){
		case RESET:
			printf("RESET\n");
			break;
		case CALIBRATE:
			printf("CALIBRATE\n");
			break;
		case CALIBRATE_CAMERA:
			printf("CALIBRATE_CAMERA\n");
			break;
		case CALIBRATE_ALTITUDE:
			printf("CALIBRATE_ALTITUDE\n");
			break;
		case CALIBRATE_ANGLE:
			printf("CALIBRATE_ANGLE\n");
			break;
		case SEND_GPS_LOCATION:
			printf("SEND_GPS_LOCATION\n");
			break;
	}
	*/
}


ISR(USARTD1_RXC_vect){
	uint8_t c = usart_getchar(GPS_TERMINAL_SERIAL);
	printf("%c", c);
	
	if(c == (uint8_t) '$'){
		writing = 1;
		pos = 0;
	}
}
