#include <asf.h>
#include <math.h>
#include <string.h>
#include "drivers\data_logger.h"
#include "drivers\thermistor.h"
#include "drivers\ms5607.h"
#include "drivers\mt3339.h"
#include "drivers\spi_controller.h"
#include "drivers\spy_cam.h"
#include "drivers\voltage.h"
#include "drivers\hall.h"
#include "drivers\IMU.h"
#include "tools\RingBuffer.h"

// GCS Commands
#define RESET				'f'
#define CALIBRATE			'e'
#define CALIBRATE_ALTITUDE	'c'
#define CALIBRATE_ANGLE		'b'
#define SEND_GPS_LOCATION	'a'
#define PACKET				'd'

// Tolerances for Flight State
#define EPSILON_VELOCITY	5
#define EPSILON_ALTITUDE	10

// EEPROM stuff
// uint16_t addr = PAGE | BYTE;
#define EEPROM_PAGE			0x1000	// Page 0
#define ALT_ADDR_BYTE0		0x00	// Byte 0
#define ALT_ADDR_BYTE1		0x01	// Byte 1
#define PACKET_ADDR_BYTE0	0x03	// Byte 3
#define PACKET_ADDR_BYTE1	0x04	// Byte 4
#define TIME_ADDR_BYTE0		0x06	// Byte 6
#define TIME_ADDR_BYTE1		0x07	// Byte 7

// Ground Constants byte addresses
#define GROUND_PRESS_ADDR0  0x09	// Byte 9
#define GROUND_PRESS_ADDR1  0x0A	// Byte 10
#define GROUND_PRESS_ADDR2  0x0B	// Byte 11
#define GROUND_PRESS_ADDR3  0x0C	// Byte 12
#define GROUND_PRESS_ADDR4  0x0D	// Byte 13
#define GROUND_PRESS_ADDR5  0x0E	// Byte 14
#define GROUND_PRESS_ADDR6  0x0F	// Byte 15
#define GROUND_PRESS_ADDR7  0x10	// Byte 16
#define GROUND_TEMP_ADDR0	0x12	// Byte 18
#define GROUND_TEMP_ADDR1	0x13	// Byte 19
#define GROUND_TEMP_ADDR2	0x14	// Byte 20
#define GROUND_TEMP_ADDR3	0x15	// Byte 21
#define GROUND_TEMP_ADDR4	0x16	// Byte 22
#define GROUND_TEMP_ADDR5	0x17	// Byte 23
#define GROUND_TEMP_ADDR6	0x18	// Byte 24
#define GROUND_TEMP_ADDR7	0x19	// Byte 25

#define READ_EEPROM			0x06	// Load CMD, Load ADDR, Load CMDEX
#define ERASE_EEPROM		0x30	// Load CMD, Load CMDEX, Wait for BUSY flag to drop
#define	ERASE_PAGE_CMD		0x32	// Load CMD, Load ADDR, Load CMDEX, Wait for BUSY flag to drop
#define LOAD_BUFFER_CMD		0x33	// Load CMD, Load ADDR0, Load DATA0, Repeat 2-3
#define	WRITE_PAGE_CMD		0x34	// Load CMD, Load ADDR, Load CMDEX, Wait for BUSY flag to drop
#define	ATOMIC_WRITE_CMD	0x35	// Load CMD, Load ADDR, Load CMDEX, Wait for BUSY flag to drop
#define ERASE_BUFFER_CMD	0x36	// Load CMD, Load CMDEX, Wait for BUSY flag to drop
// BUSY flag is NVM.STATUS>>7
 
#define CTRLA_CMDEX_BYTE	0x01	// Tells NVM to execute the command
#define CCP_IOREG_MODE		0xD8	// Set CCP to this to allow CMDEX to be changed
 
///////////////////////// Function Prototypes //////////////////////////
void	system_init(void);												// Starts the system
void	pressure_init(void);											// Collects the sensors constants
void	gps_init(void);													// Starts the GPS
void	xbee_init(void);												// Starts the XBEE
void	bno_init(void);													// Starts the BNO055
void	release(void);													// Releases the Science Payload
double	get_pressure(void);												// Pascals
double	get_temperature(void);											// Celsius
double	get_altitude(double press);										// meters
double	get_voltage(void);												// Volts
double	diff(RingBuffer16_t* data, uint8_t frequency);					// Approximates velocity of CanSat
void	data_collect(RingBuffer16_t* alts, RingBuffer32_t* presses);	// Handles data collection
double	data_check(RingBuffer32_t* presses);							// Function that averages values and compares with stdev
void	imu_read(void);													// Reads and calculates position of board
void	state_check(void);												// Returns the current state
void	release_servo_init(void);										// Starts Release Servo Rate
void	servo_timer_init(void);											// Starts PWM wave for fin servos
void	servo_pid(RingBuffer16_t* direct);								// Function that alters the position of the fins
void	clock_init(void);												// Starts a timer to count the seconds
void	time_update(void);												// Function to perform timer based actions
void	buzzer_init(void);

//XBEE controls
void	command(uint8_t c);
void	reset(void);													// Re-initializes the CanSat
void	calibrate(void);
void	cali_alt(void);
void	cali_ang(void);
void	send_gps(void);
void	packet(void);

// EEPROM commands
void	eeprom_write_const(void);
void	eeprom_write(void);
uint8_t	eeprom_read(uint16_t address);
void	eeprom_erase(void);	

/////////////////////////// Global Variables ///////////////////////////0
uint8_t state = 0;
uint8_t released = 0;
uint8_t reset_received = 0;
double ground_p = 101325;		// Pascals standard sea level pressure
double ground_t = 288.15;		// 15 C
double ground_a = 0;			// Assumes ground altitude at launch

// Interrupt Flags
uint8_t time_flag = 0;			// New data to write to EEPROM
uint8_t quat_data = 0;			// New quaternion information
uint8_t xbee_flag = 0;

// Altitude calculation variables
double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.80665;

// Pressure Calculation variables
uint16_t c[] = {0,0,0,0,0,0};

// Release
uint16_t release_pulse = 1500; // microseconds (0 position)

// Fin Servo
uint16_t servo_pulse = 1500;

// XBEE
uint8_t xbee_comm = 0;

// IMU
double ref_yaw = 0;
double ref_roll = 0;
double ref_pitch = 0;

// GPS Stuff
char gps[15];			// GPS sentences
char dec[5];
uint8_t writing = 0;
uint8_t pos = 0;
uint8_t word_pos = 0;
uint8_t commas = 0;
uint8_t idx = 0;

// Output string
char str[100];					// Output String

// Time and Packets
uint16_t timer = 0;
uint16_t data_packets = 0;
uint16_t packets = 0;
uint16_t rate = 10;

// Initializes variables
double press = 0;			// Pressure (Pa)
double temp = 0;			// Temperature (C)
double alt = 0;				// Altitude (m)
double volt = 0;			// Battery Terminal Voltage (V)
double velocity = 0;		// Velocity (m/s)
double gps_t = 0;			// GPS Time
double gps_lat = 0;			// GPS Latitude (+:N,-:S)
double gps_long = 0;		// GPS Longitude (+:E,-:W)
double gps_alt = 0;			// GPS Altitude
int16_t gps_sats = 0;		// GPS Satellites
double pitch = 0;			// Pitch Angle
double roll = 0;			// Roll Angle
double rpm = 0;				// Calculate RPM of Blades
double angle = 0;			// Angle of Bonus Direction	

char* format = "5343,%i,%i,%i,%li,%i,%i,%02i:%02i:%02i,%i.%li,%i.%li,%i.%i,%i,%i,%i,%i,%i,%i\n\0";


////////////////////////////// Functions ///////////////////////////////
int main (void)
{
	system_init();
	//delay_ms(100);
	
	PORTD.DIR |= PIN3_bm;
	PORTD.OUT |= PIN3_bm;
	
	//printf("Initialized\n");
	//buzzer_init();
	
	int16_t alt_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer16_t altitudes;	// in centimeters
	rb16_init(&altitudes, alt_array, (uint16_t) 10);
	
	int32_t press_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer32_t pressures;	// in Pascals / 10
	rb32_init(&pressures, press_array, (uint16_t) 10);
	
	int16_t direct_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer16_t directions;	// in hundredths degrees
	rb16_init(&directions, direct_array, (uint16_t) 10);
	
	uint8_t cam_initialized = 0;
	uint8_t buzzer_initialized = 0;
	//printf("Before Loop\n");

	
	while(1){
		// Check Sensors
		data_collect(&altitudes,&pressures);
		
		state_check();
		
		// IMU Check
		//imu_read();
		 
		//Gives each flight state their unique tasks
		switch(state){
			case 0:
				break;
			case 1:
				if(alt > 600 && !cam_initialized){
					cam_initialized = 1;
					//cam_switch();
				}
				break;
			case 2:
				if(!cam_initialized){
					cam_initialized = 1;
					//cam_switch();
				}
				if(abs(alt-450)<EPSILON_ALTITUDE){
					release();				// Releases the payload
					hall_sensor_init();		// Starts hall effect sensor to read rpm
				}
				else if(released){
					servo_pid(&directions);	// Updates the PID
				}
				break;
			case 3:
				if(!buzzer_initialized){
					//buzzer_init();
					buzzer_initialized = 1;
				}
				break;
			default:
				state = 0;
				break;
		}
		
		if(time_flag){
			time_update();
			time_flag = 0;
		}
		if(xbee_flag){
			command(xbee_comm);
			xbee_comm = 0;
			xbee_flag = 0;
		}
		
		data_packets++;
		if(timer != 0){
			rate = data_packets / timer;
		}
		
		delay_ms(20);
	}
}


// Sensor functions
void system_init(void){
	// Initialization of systems
	sysclk_init(); // initializes the system clock
	delay_ms(2); // delays the rest of the processes to ensure a started clock
	sei();
	
	// Initialization of pins
	PORTC.DIR = 0xBC; // makes Port C have pins, 7, 5, 4, 3, and 2 be output (0b10111100)
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm; // enables lo level interrupts
	
	cam_init();
	// Driver Initialization
	data_terminal_init();
	delay_ms(500);
	xbee_init();
	gps_init();
	//delay_ms(100);
	
	//thermistor_init();
	//voltage_init();
	spi_init();
	pressure_init();
	//bno_init();
	cam_switch();
	clock_init();

	//buzzer_init();
	//release_servo_init();

	delay_ms(10);
	
	state_check();
	
	// Check EEPROM
	/*
	if(eeprom_read(EEPROM_PAGE|TIME_ADDR_BYTE1)^0xFF){
		ground_p = (double) ((uint64_t) (eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR7)<<56 | eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR6)<<48 | 
										 eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR5)<<40 | eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR4)<<32 | 
										 eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR3)<<24 | eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR2)<<16 | 
										 eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR1)<<8  | eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR0)));
		ground_t = (double) ((uint64_t) (eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR7)<<56 | eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR6)<<48 |
										 eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR5)<<40 | eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR4)<<32 |
										 eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR3)<<24 | eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR2)<<16 |
										 eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR1)<<8  | eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR0)));
		alt = (double) ((int16_t) (eeprom_read(EEPROM_PAGE|ALT_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|ALT_ADDR_BYTE0)));
		timer = (uint16_t) (eeprom_read(EEPROM_PAGE|TIME_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|TIME_ADDR_BYTE0));
		packets = (uint16_t) (eeprom_read(EEPROM_PAGE|PACKET_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|PACKET_ADDR_BYTE0));
	}
	else{
	*/
	// Initialization of variables
	ground_p = get_pressure();
	//printf("%li\n", (int32_t) ground_p);
	ground_t = get_temperature();
	
	eeprom_write_const();
	//}
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
	printf("\n%u,%u,%u,%u,%u,%u\n",c[0],c[1],c[2],c[3],c[4],c[5]);
}

void gps_init(void){
	gps_uart_init();				// Starts the GPS
	delay_ms(2);
	
	(*GPS_TERMINAL_SERIAL).CTRLA = USART_RXCINTLVL_HI_gc;
}

void xbee_init(void){
	USARTE0.CTRLA = USART_RXCINTLVL_MED_gc;
}

void bno_init(void){
	sysclk_enable_module(SYSCLK_PORT_E, PR_TWI_bm);
	sysclk_enable_peripheral_clock(&TWIE);
	
	imu_init();
}

void release(void){
	TCD0.CCA = 1200;
	
	released = 1;
}

double get_pressure(void){
	//printf("Time: %i\n", timer/10);
	double val = 101325;
	
	delay_ms(8);
	uint32_t d1 = ms5607_convert_d1();
	delay_ms(8);
	uint32_t d2 = ms5607_convert_d2();	
	//while(d1 == 0xffffffff | d2 == 0xffffffff);
	//printf("%lu,%lu\n",d1,d2);
	double dt = d2 - (uint64_t)c[4] * 256.0;
	//double temp = 20.0 + ((uint64_t) dt * c[5]) / 8388608.0;
	long double off = (uint64_t) c[1] * 131072.0 + ((uint64_t) c[3] * dt) / 64.0;
	long double sens = (uint64_t) c[0] * 65536.0 + ((uint64_t) c[2] * dt) / 128.0;
	
	val = (double) (((double) d1 * sens / 2097152.0 - off) / 32768.0);
	//printf("Pressure: %li\n", (int32_t) val);
	//printf("%li\n",(int32_t) val);
	return val;	// returns pressure in Pa
}

double get_temperature(void){
	double val = 303.15; // Change to 15 degrees C
	//uint16_t reading = thermistor_read();
	//printf("%u\n", reading);
	//double voltage = (.000496735 * reading - 0.095430804); // m and b are collected from testing
	//double resistance = 9990 * (3.27 - voltage) / voltage; // 6720 is the resistance of the steady resistor
	//val = (100.0 / (3.354016E-3 + 2.569850E-4 * log(resistance / 10000) + 2.620131E-6 * pow(log(resistance / 10000), 2) + 6.383091E-8 * pow(log(resistance / 10000), 3))); // returns the temperature in hundredths of kelvin
	return val; //returns the temperature in kelvin
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return (val-ground_a);		//returns altitude in meters
}

double get_voltage(void){
	uint16_t reading = voltage_read();
	//printf("%u\n", reading);
	double voltage = (.0004966 * reading - .096364766); // m and b are collected from testing
	voltage = voltage * (29800.0 / 13000.0) + voltage; // 6720 is the resistance of the steady resistor
	return voltage;
}

// Approximates the Velocity from past five altitudes
uint8_t data_samples = 3;
double diff(RingBuffer16_t* data, uint8_t frequency){
	double val = 0;
	for(uint16_t i = 0; i < data_samples; i++){
		int32_t new = rb16_get_nth(data,i);
		int32_t old = rb16_get_nth(data,i+1);
		int32_t oldest = rb16_get_nth(data,i+2);
		val += ((3*new - 4*old + oldest) * frequency / 2.0); // O(h^2) approximation of backwards derivative (Thanks MAE284, you're good for something)
	}
	val /= data_samples;
	return (val/100);
}

void data_collect(RingBuffer16_t* alts, RingBuffer32_t* presses){
	//printf("In Data Collect\n");
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
		velocity = diff(alts, rate);
	}
	temp = get_temperature();	// Grabs the temperature once
	//volt = get_voltage();
}

double data_check(RingBuffer32_t* presses){
	// Calculates average of data
	uint8_t length = 5;
	double mean = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 700000 && p < 1100000){ // Checks to make certain value makes sense
			mean += ((double) p)/length;
		}
	}
	
	// Calculates standard deviation of data
	double stdev = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 700000 && p < 1100000){
			stdev += pow(p-mean, 2);
		}
	}
	stdev /= (length - 1);
	
	// Throws out data that is farther than standard deviation from average
	double result = 0;
	uint8_t nums = 0;
	for(uint8_t i = 0; i < length; i++){
		int32_t p = rb32_get_nth(presses, i);
		if(p > 700000 && p < 1100000){
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

void imu_read(void){
	imu_update();
	
	double r = imu_roll() - ref_roll;
	double p = imu_pitch() - ref_pitch;
	double y = imu_heading() - ref_yaw;
	
	if(abs(r) > 180){
		r = r - 360 * r / abs(r);
	}
	if(abs(p) > 90){
		p = p - 180 * p / abs(p);
	}
	if(abs(y) > 180){
		y = y - 360 * y / abs(y);
	}
	
	roll = r;
	pitch = p;
	angle = y;
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
		if(alt > 50 || reset_received){
			state = 1;
		}
		if(released){	// only change this to true in flight state 2
			state = 3;
		}
	}
}

void release_servo_init(void){
	sysclk_enable_peripheral_clock(&TCD0); //enables peripheral clock for TCD0
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES); //necessary jumbo

	PORTA.DIR |= 0x02;
	PORTA.OUT |= 0x02;
	PORTD.DIR |= 0x07;
	TCD0.CTRLA = 0x05; // sets the clock's divisor to 64
	TCD0.CTRLB = 0x13; // enables CCA and Single Waveform
	TCD0.PER = 10000; // sets the period (or the TOP value) to the period
	TCD0.CCA = 800; // makes the waveform be created for a duty cycle // 500 ticks per millisecond
}

void servo_timer_init(void){
	PORTD.DIR |= 0x02;
}

void servo_pid(RingBuffer16_t* direct){
	double k_p = 0;
	double k_i = 0;
	double k_d = 0;
	
	double p = 0;
	double i = 0;
	double d = 0;
	
	int16_t sum = 0;
	for(uint8_t i = 0; i < 10; i++){
		sum += rb16_get_nth(direct,i);
	}
	
	p = k_p * rb16_get_nth(direct, 0) / 100.0;
	i = k_i * sum/100;
	d = k_d * diff(direct,rate);
	
	double val = p + i + d;
	servo_pulse = 1500 + val;  // 1500 is the zero position in this model
	
	TCD0.CCA = (uint16_t) ((500) * (servo_pulse / 1000.0)); // makes the waveform be created for a duty cycle
}

void clock_init(void){
	sysclk_enable_peripheral_clock(&TCE0); // starts peripheral clock

	TCE0.CTRLA = 0x07; // divisor set to 1024 0x07
	TCE0.PER = 31249; // 1 Hz
	TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc; // CCA int flag Lo level
}

void time_update(void){
	timer+=1;
	packets++;
	
	sprintf(str,format,timer,packets,
	(int16_t) (alt),						(int32_t) press,							(int16_t) (temp-273.15),				(int16_t)volt,
	(int16_t) (((int32_t)gps_t)/10000),		(int16_t) ((((int32_t)gps_t)%10000)/100),	(int16_t) (((int32_t)gps_t)%100),
	(int16_t) gps_lat,						((int32_t) (gps_lat*1000000))%1000000,		(int16_t) gps_long,						(int32_t)(abs(((int32_t)(gps_long*1000000))%1000000)),
	(int16_t) gps_alt,						((int16_t) (gps_alt)*10)%10,				gps_sats,
	(int16_t) pitch,						(int16_t) roll,								(int16_t) rpm,
	state,									(int16_t) angle); // Data Logging Test
	printf(str);
	
	eeprom_write();
	
	time_flag = 0;
}

void buzzer_init(void){
	sysclk_enable_peripheral_clock(&TCD1); //enables peripheral clock for TCD0
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES); //necessary jumbo

	PORTD.DIR |= 0x10;
	TCD1.CTRLA = 0x05; // sets the clock's divisor to 64
	TCD1.CTRLB = 0x13; // enables CCA and Single Waveform
	TCD1.PER = 184; // 2700hz
	TCD1.CCA = 92;
}

void command(uint8_t c){
	switch(c){
		case RESET:
			//printf("RESET\n");
			reset();
			break;
		case CALIBRATE:
			calibrate();
			//printf("CALIBRATE\n");
			break;
		case CALIBRATE_ALTITUDE:
			cali_alt();
			//printf("CALIBRATE_ALTITUDE\n");
			break;
		case CALIBRATE_ANGLE:
			cali_ang();
			//printf("CALIBRATE_ANGLE\n");
			break;
		case SEND_GPS_LOCATION:
			send_gps();
			//printf("SEND_GPS_LOCATION\n");
			break;
		case PACKET:
			packet();
			//printf("PACKET\n");
			break;
	}
}

void reset(void){
	eeprom_erase();
	
	uint8_t oldInterruptState = SREG;	// no real need to store the interrupt context as the reset will pre-empt its restoration
	cli();		                        // Disable interrupts

	CCP = 0xD8;							// Configuration change protection: allow protected IO regiser write
	RST.CTRL = RST_SWRST_bm;			// Request software reset by writing to protected IO register

	SREG=oldInterruptState;
}

void calibrate(void){
	cali_alt();
	cali_ang();
}

void cali_alt(void){
	ground_p = press;
	//ground_a = alt;
	ground_t = get_temperature();
}

void cali_ang(void){
	//////Calibrate angle and stuff///////
}

char* gps_msg = "%i.%li,%i.%li\n\0";
void send_gps(void){
	char msg[70];
	sprintf(msg,gps_msg,(int16_t)gps_lat,((int32_t)(gps_lat*1000000))%1000000,(int16_t)gps_long,(int32_t)(abs(gps_long)*1000000)%1000000);
	//XBEE_spi_write(msg);
	printf(msg);
}

void packet(void){
	//XBEE_spi_write(str);
	packets++;
	sprintf(str,format,timer/10,timer%10,packets,
	(int16_t) (alt),						(int32_t) press,							(int16_t) (temp-273.15),				(int16_t)volt,
	(int16_t) (((int32_t)gps_t)/10000),		(int16_t) ((((int32_t)gps_t)%10000)/100),	(int16_t) (((int32_t)gps_t)%100),
	(int16_t) gps_lat,						((int32_t) (gps_lat*1000000))%1000000,		(int16_t) gps_long,						(int32_t)(abs(((int32_t)(gps_long*1000000))%1000000)),
	(int16_t) gps_alt,						((int16_t) (gps_alt)*10)%10,				gps_sats,
	(int16_t) pitch,						(int16_t) roll,								(int16_t) rpm,
	state,									(int16_t) angle); // Data Logging Test
	printf(str);
}

void eeprom_write_const(void){
	uint64_t g_p = (uint64_t) ground_p;
	uint64_t g_t = (uint64_t) ground_t;
	
	uint8_t data[] = {	g_p&0xFF, (g_p>>8)&0xFF, (g_p>>16)&0xFF, (g_p>>24)&0xFF, (g_p>>32)&0xFF, (g_p>>40)&0xFF, (g_p>>48)&0xFF, (g_p>>56), 
						g_t&0xFF, (g_t>>8)&0xFF, (g_t>>16)&0xFF, (g_t>>24)&0xFF, (g_t>>32)&0xFF, (g_t>>40)&0xFF, (g_t>>48)&0xFF, (g_t>>56)};
	uint8_t addresses[] = {	GROUND_PRESS_ADDR0, GROUND_PRESS_ADDR1, GROUND_PRESS_ADDR2, GROUND_PRESS_ADDR3, GROUND_PRESS_ADDR4, GROUND_PRESS_ADDR5, GROUND_PRESS_ADDR6, GROUND_PRESS_ADDR7,
							GROUND_TEMP_ADDR0,  GROUND_TEMP_ADDR1,  GROUND_TEMP_ADDR2,  GROUND_TEMP_ADDR3,  GROUND_TEMP_ADDR4,  GROUND_TEMP_ADDR5,  GROUND_TEMP_ADDR6,  GROUND_TEMP_ADDR7};
	
	NVM.CMD = LOAD_BUFFER_CMD;
	for(uint8_t i = 0; i < 16; i++){
		NVM.ADDR0 = addresses[i];
		NVM.DATA0 = data[i];
	}
	
	// Erases and writes the page buffer
	NVM.CMD = ATOMIC_WRITE_CMD;
	NVM.ADDR0 = EEPROM_PAGE & 0xFF;
	NVM.ADDR1 = EEPROM_PAGE >> 8;
	CCP = CCP_IOREG_MODE;
	NVM.CTRLA = CTRLA_CMDEX_BYTE;
	while(NVM.STATUS>>7);
}

void eeprom_write(void){
	uint16_t a = (uint16_t) ((int16_t) alt); // creates an unsigned int of the altitude
	
	// saves data and addresses in array
	uint8_t data[] = {a >> 8, a & 0xFF, packets >> 8, packets & 0xFF, timer >> 8, timer & 0xFF};
	uint8_t addresses[] = {ALT_ADDR_BYTE1, ALT_ADDR_BYTE0, PACKET_ADDR_BYTE1, PACKET_ADDR_BYTE0, TIME_ADDR_BYTE1, TIME_ADDR_BYTE0};
	
	// Writes the NVM Registers to write the buffer
	NVM.CMD = LOAD_BUFFER_CMD;
	for(uint8_t i = 0; i < 6; i++){
		NVM.ADDR0 = addresses[i];
		NVM.DATA0 = data[i];
	}
	
	// Erases and writes the page buffer
	NVM.CMD = ATOMIC_WRITE_CMD;
	NVM.ADDR0 = EEPROM_PAGE & 0xFF;
	NVM.ADDR1 = EEPROM_PAGE >> 8;
	CCP = CCP_IOREG_MODE;
	NVM.CTRLA = CTRLA_CMDEX_BYTE;
	while(NVM.STATUS>>7);
}

uint8_t	eeprom_read(uint16_t address){
	NVM.CMD = READ_EEPROM;
	NVM.ADDR0 = address & 0xFF;
	NVM.ADDR1 = address >> 8;
	CCP = CCP_IOREG_MODE;
	NVM.CTRLA = CTRLA_CMDEX_BYTE;
	delay_ms(1);
	uint8_t byte = NVM.DATA0;
	return byte;
}

void eeprom_erase(void){
	NVM.CMD = LOAD_BUFFER_CMD;
	for(uint8_t i = 0; i < 32; i++){
		NVM.ADDR0 = i;
		NVM.DATA0 = 0xFF;
	}
	
	NVM.CMD = ERASE_EEPROM;
	CCP = CCP_IOREG_MODE;
	NVM.CTRLA = CTRLA_CMDEX_BYTE;
	while(NVM.STATUS>>7);
}

ISR(TCE0_OVF_vect){	
	time_flag = 1;
}

ISR(USARTE0_RXC_vect){
	xbee_comm = usart_getchar(UART_TERMINAL_SERIAL);
	xbee_flag = 1;
}


// GPS recording
ISR(USARTD1_RXC_vect){
	uint8_t c = usart_getchar(GPS_TERMINAL_SERIAL);
	//printf("%c",c);
	
	if(c == (uint8_t) '$'){
		writing = 1;
		pos = 0;
		word_pos = 0;
		commas = 0;
	}
	else if(c == (uint8_t) '*'){
		commas = 0;
		writing = 0;
	}
	else if(c == (uint8_t) ','){
		gps[0] = 32;
		if(gps[1] == 32){
			gps[0] = '0';
		}
		int32_t val = 0;
		int16_t val2 = 0;
		switch(commas){
			case 1:			//Time
				for(uint8_t i = 0; i < 15; i++){
					if(gps[i] == (uint8_t) '.'){
						idx = i;
						break;
					}
				}
				gps[idx] = 32;
				for(uint8_t i = idx+1; i < 15; i++){
					if(gps[i] == 32){
						break;
					}
					gps[i] = 32;
				}
				sscanf(gps,"%ld",&val);
				gps_t = (double) val;
				break;
			case 2:			//Latitude
				for(uint8_t i = 0; i < 15; i++){
					if(gps[i] == (uint8_t) '.'){
						idx = i;
						break;
					}
				}
				gps[idx] = 32;
				for(uint8_t i = idx+1; i < 15; i++){
					if(gps[i] == 32){
						break;
					}
					dec[i-idx-1] = gps[i];
					gps[i] = 32;
				}
				if(dec[0] == 32){
					dec[0] = '0';
				}
				sscanf(gps,"%ld",&val);
				sscanf(dec,"%d",&val2);
				gps_lat = val/100 + ((double)(val%100) + ((double)(val2))/10000) / 60.0;
				break;
			case 4:			//Longitude
				for(uint8_t i = 0; i < 15; i++){
					if(gps[i] == (uint8_t) '.'){
						idx = i;
						break;
					}
				}
				gps[idx] = 32;
				for(uint8_t i = idx+1; i < 15; i++){
					if(gps[i] == 32){
						break;
					}
					dec[i-idx-1] = gps[i];
					gps[i] = 32;
				}
				if(dec[0] == 32){
					dec[0] = '0';
				}
				sscanf(gps,"%ld",&val);
				sscanf(dec,"%d",&val2);
				gps_long = -(val/100 + ((double)(val%100) + ((double)(val2))/10000) / 60.0);
				break;
			case 7:			//Sats
				sscanf(gps,"%d",&gps_sats);
				break;
			case 9:			//Altitude
				for(uint8_t i = 0; i < 15; i++){
					if(gps[i] == (uint8_t) '.'){
						idx = i;
						break;
					}
				}
				gps[idx] = 32;
				for(uint8_t i = idx+1; i < 15; i++){
					if(gps[i] == 32){
						break;
					}
					dec[i-idx-1] = gps[i];
					gps[i] = 32;
				}
				if(dec[0] == 32){
					dec[0] = '0';
				}
				sscanf(gps,"%ld",&val);
				sscanf(dec,"%d",&val2);
				gps_alt = (double) val + ((double) val2/10.0);
				break;
		}
	
		if(writing){
			idx = 0;
			commas++;
			word_pos = 0;
			for(uint8_t i = 0; i < 15; i++){
				gps[i] = 32;
			}
			for(uint8_t i = 0; i < 5; i++){
				dec[i] = 32;
			}
		}
	}
		
	if(writing){
		switch(pos){
			case 3:
				if(c != 'G'){
					writing = 0;
				}
				break;
				
			case 4:
				if(c != 'G'){
					writing = 0;
				}
				break;
		}
		gps[word_pos] = c;
		word_pos++;
		pos++;
	}
}