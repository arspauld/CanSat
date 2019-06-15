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
#include "drivers\IMU.h"
#include "tools\RingBuffer.h"

// GCS Commands
#define RESET				'f'
#define CALIBRATE			'e'
#define CALIBRATE_ALTITUDE	'c'
#define CALIBRATE_ANGLE		'b'
#define SERVO_RELEASE		'a'
#define SERVO_CLOSE			's'
#define PACKET				'd'

// Tolerances for Flight State
#define EPSILON_VELOCITY	5
#define EPSILON_ALTITUDE	10

// Servo
#define SERVO_NEUTRAL_PULSE	750
#define SERVO_WAVELENGTH	10000
#define RELEASE_SERVO_OPEN	1000
#define RELEASE_SERVO_CLOSE 600
#define SERVO_TRIM		45

// EEPROM stuff
// uint16_t addr = PAGE | BYTE;
#define EEPROM_PAGE			0x1000	// Page 0
#define ALT_ADDR_BYTE0		0x00	// Byte 0
#define ALT_ADDR_BYTE1		0x01	// Byte 1
#define PACKET_ADDR_BYTE0	0x03	// Byte 3
#define PACKET_ADDR_BYTE1	0x04	// Byte 4
#define TIME_ADDR_BYTE0		0x06	// Byte 6
#define TIME_ADDR_BYTE1		0x07	// Byte 7
#define VEL_ADDR_BYTE0		0x1B	// Byte 27
#define VEL_ADDR_BYTE1		0x1C	// Byte 28
#define CHECK_WRITE_BYTE0	0x02	// Byte 2
#define CHECK_WRITE_BYTE1	0x1F	// Byte 31
#define STATE_BYTE			0x05	// Byte 5
#define RELEASED_BYTE		0x08	// Byte 8

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
void	hall_sensor_init(void);											// Starts the Hall Effect Sensor
void	change_hall_sensor_scaler(void);								// Updates the Hall Effect scale factor
void	release(void);													// Releases the Science Payload
double	get_pressure(void);												// Pascals
double	get_temperature(void);											// Celsius
double	get_altitude(double press);										// meters
double	get_voltage(void);												// Volts
double	diff(RingBuffer16_t* data, uint8_t frequency);					// Approximates velocity of CanSat
void	data_collect(RingBuffer16_t* alts, RingBuffer32_t* presses);	// Handles data collection
double	data_check(RingBuffer32_t* presses);							// Function that averages values and compares with stdev
void	imu_read(void);
void	pid_val(RingBuffer16_t* direct);								// Reads and calculates position of board
void	state_check(void);												// Returns the current state
void	release_servo_init(void);										// Starts Release Servo Rate
void	fin_servo_init(void);											// Starts PWM wave for fin servos
void	servo_pid(RingBuffer16_t* direct);								// Function that alters the position of the fins
void	clock_init(void);												// Starts a timer to count the seconds
void	time_update(void);												// Function to perform timer based actions
void	buzzer_init(void);												// Starts the buzzer
void	buzzer_stop(void);												// Stops the buzzer
void	calc_rpm(void);													// Function to calculate rpm for packets
void	update_scale_factor(void);										// Recalculates the scale factor of the hall effect based on input voltage
static void hall_sensor_measure(AC_t *ac, uint8_t channel, enum ac_status_t status); // Interrupt function for Hall Effect sensor

// XBEE controls
void	command(uint8_t c);												// Switch case for commands
void	reset(void);													// Re-initializes the CanSat
void	calibrate(void);												// Calibrates the payload (angle and altitude)
void	cali_alt(void);													// Calibrates the altitude
void	cali_ang(void);													// Calibrates the angle
void	servo_release(void);											// Opens the release servo (unlocks)
void	servo_close(void);												// Closes the release servo (locks)
void	packet(void);													// Transmits last packet stored

// EEPROM commands
void	eeprom_write_const(void);										// Writes the ground pressure and temp to EEPROM
void	eeprom_write(void);												// Writes the last packet to EEPROM
uint8_t	eeprom_read(uint16_t address);									// Reads the byte at specified address in EEPROM
void	eeprom_erase(void);												// Erases all bytes on all pages of EEPROM

/////////////////////////// Global Variables ///////////////////////////
volatile uint8_t state = 0;
volatile uint8_t released = 0;

double ground_p = 101325;		// Pascals standard sea level pressure
double ground_t = 288.15;		// 15 C

// Interrupt Flags
volatile uint8_t time_flag = 0;			// New data to write to EEPROM
volatile uint8_t xbee_flag = 0;			// New command received from ground station

// Hall Effect Constants
volatile uint8_t scale_factor = 0;		// updated with function

// Altitude calculation variables
double R = 287.0578987;
double L = -0.0065;
double g_0 = 9.80665;

// Pressure Calculation variables
uint16_t c[] = {0,0,0,0,0,0};

// XBEE
volatile uint8_t xbee_comm = 0; // Variable stores XBee commands

// IMU
volatile double ref_yaw = 180;				// Should be collected
volatile double ref_roll = 0;				// Ideal
volatile double ref_pitch = 90;				// Ideal

// RPM
volatile uint16_t ticks_per_sec = 0; // Hall effect interrupts per seconds (what a unit)

// GPS Stuff
char gps[15];						// GPS sentences
char dec[5];
volatile uint8_t writing = 0;
volatile uint8_t pos = 0;
volatile uint8_t word_pos = 0;
volatile uint8_t commas = 0;
volatile uint8_t idx = 0;

// EEPROM
volatile uint8_t check_write = 0; // Number used to verify successful write to EEPROM

// Output String
char str[100];

// Time and Packets
volatile uint16_t timer = 0;			// Time since startup
volatile uint16_t data_packets = 0;		// Number of times looped through data collection function
volatile uint16_t packets = 0;			// Number of packets since startup (equal to time at 1Hz transmission rate)
volatile uint16_t rate = 10;			// Frequency at which data is collected

// Initializes variables
volatile double press = 0;				// Pressure (Pa)
volatile double temp = 0;				// Temperature (C)
volatile double alt = 0;				// Altitude (m)
volatile double volt = 0;				// Battery Terminal Voltage (V)
volatile double velocity = 0;			// Velocity (m/s)
volatile double gps_t = 0;				// GPS Time
volatile double gps_lat = 0;			// GPS Latitude (+:N,-:S)
volatile double gps_long = 0;			// GPS Longitude (+:E,-:W)
volatile double gps_alt = 0;			// GPS Altitude
volatile int16_t gps_sats = 0;			// GPS Satellites
volatile double pitch = 0;				// Pitch Angle
volatile double roll = 0;				// Roll Angle
volatile double rpm = 0;				// Calculate RPM of Blades
volatile double angle = 0;				// Angle of Bonus Direction

// TEAM ID, TIME, PACKETS, ALTITUDE, PRESSURE, TEMPERATURE, VOLTAGE, GPS TIME, LATITUDE, LONGITUDE, GPS ALT, GPS SATS, PITCH, ROLL, BLADE SPIN RATE, FLIGHT STATE, BONUS DIRECTION
char* format = "5343,%u,%u,%i.%i,%li,%i.%i,%i.%i,%02i:%02i:%02i,%i.%li,%i.%li,%i.%i,%i,%i,%i,%i,%u,%i\n\0"; // Format for output string


////////////////////////////// Functions ///////////////////////////////
int main(void){
	system_init();
	//delay_ms(100);

	// Turns on status LED
	PORTD.DIR |= PIN3_bm;
	PORTD.OUT |= PIN3_bm;
	//buzzer_init();
	delay_ms(125);
	//buzzer_stop();

	// Integer ring buffer for storing multiple older values
	int16_t alt_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer16_t altitudes;	// in centimeters
	rb16_init(&altitudes, alt_array, (uint16_t) 10);

	int32_t press_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer32_t pressures;	// in Pascals / 10
	rb32_init(&pressures, press_array, (uint16_t) 10);

	int16_t direct_array[] = {0,0,0,0,0,0,0,0,0,0};
	RingBuffer16_t directions;	// in hundredths degrees
	rb16_init(&directions, direct_array, (uint16_t) 10);

	// Boolean values of the state of the camera and buzzer
	uint8_t cam_initialized = 0;
	uint8_t buzzer_initialized = 0;


	while(1){
		// Check Sensors
		data_collect(&altitudes,&pressures);

		// Checks the Flight State
		state_check();

		// IMU Check
		imu_read();

		//Gives each flight state their unique tasks
		switch(state){
			case 0:
				break;
			case 1:
				if(!cam_initialized){
					cam_initialized = 1;
					cam_switch();	//	Turns on Camera
					delay_ms(100);
					cam_switch();
				}
				break;
			case 2:
				if(abs(alt-450)<EPSILON_ALTITUDE){
					release();				// Releases the payload
					pid_val(&directions);
					ref_yaw = angle;
				}
				else if(released){
					pid_val(&directions);	// Writes angle values to the ring buffer
					servo_pid(&directions); // Updates PID
				}
				if(!cam_initialized){
					cam_initialized = 1;
					cam_switch();
				}
				break;
			case 3:
				if(!buzzer_initialized){
					buzzer_init();			//UNCOMMENT BEFORE FLIGHT
					buzzer_initialized = 1;
					
					cam_switch();
				}
				break;
			default:
				state_check();
				break;
		}

		// 1 Hz timer interrupt function
		if(time_flag){
			temp = get_temperature();	// Grabs the temperature once
			volt = get_voltage();		// Updates voltage
			update_scale_factor();		// Updates Hall Effect scale factor
			calc_rpm();					// Resets the tick counter and averages it with the new values
			time_update();				// Transmits the data packet and writes the EEPROM
			time_flag = 0;
		}
		// XBEE command received
		if(xbee_flag){
			command(xbee_comm);
			xbee_comm = 0;
			xbee_flag = 0;
		}

		// Recalculates the data writing rate
		data_packets++;
		if(timer != 0){
			rate = data_packets / timer;
		}
	}
}


// Sensor functions
void system_init(void){
	// Initialization of systems
	sysclk_init(); // initializes the system clock
	delay_ms(2); // delays the rest of the processes to ensure a started clock
	sei();	// Enables global interrupts

	// Initialization of pins
	PORTC.DIR = 0xBC; // makes Port C have pins, 7, 5, 4, 3, and 2 be output (0b10111100)
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm; // enables all level interrupts

	// Driver Initialization
	cam_init();				// Initializes the Camera
	data_terminal_init();	// Initializes the OpenLog
	delay_ms(500);			// Delay to ensure clean writing
	xbee_init();			// Sets up XBEE command interrupt
	gps_init();				// Starts the GPS interrupt

	hall_sensor_init();		// Initializes the hall effect sensor (used here for debugging)
	thermistor_init();		// Initializes the thermistor
	voltage_init();			// Initializes the voltage reader
	spi_init();				// Initializes the SPI communication
	pressure_init();		// Initializes the pressure sensor
	bno_init();				// Initializes the IMU
	clock_init();			// Starts the clock for data transmission

	release_servo_init();	// Initializes the release servo
	fin_servo_init();		// Initializes the timer for the servo

	// Check EEPROM
	uint8_t b1 = eeprom_read(EEPROM_PAGE|CHECK_WRITE_BYTE0);
	uint8_t b2 = eeprom_read(EEPROM_PAGE|CHECK_WRITE_BYTE1);

	if((b1 == b2) && (b1 != 0xFF)){
		//printf("Reading EEPROM\n");
		uint64_t p =  ((uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR7)<<56 | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR6)<<48 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR5)<<40 | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR4)<<32 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR3)<<24 | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR2)<<16 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR1)<<8  | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_PRESS_ADDR0));
		uint64_t t =  ((uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR7)<<56  | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR6)<<48 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR5)<<40  | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR4)<<32 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR3)<<24  | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR2)<<16 |
					   (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR1)<<8   | (uint64_t) eeprom_read(EEPROM_PAGE|GROUND_TEMP_ADDR0));
		memcpy(&ground_p, &p, 8);
		memcpy(&ground_t, &t, 8);

		alt = (double) ((int16_t) (eeprom_read(EEPROM_PAGE|ALT_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|ALT_ADDR_BYTE0)));
		timer = (uint16_t) (eeprom_read(EEPROM_PAGE|TIME_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|TIME_ADDR_BYTE0));
		packets = (uint16_t) (eeprom_read(EEPROM_PAGE|PACKET_ADDR_BYTE1)<<8 | eeprom_read(EEPROM_PAGE|PACKET_ADDR_BYTE0));
		state = eeprom_read(EEPROM_PAGE|STATE_BYTE);
		released = eeprom_read(EEPROM_PAGE|RELEASED_BYTE);
		//printf("Ground Pressure: %li\nGround Temperature: %i\n", (int32_t) ground_p, (int16_t) ground_t);
	}
	else{
		// Initialization of variables
		ground_p = get_pressure();
		ground_t = get_temperature();
		state = 0;
		eeprom_write_const();
	}
	
	temp = get_temperature();	// Grabs the temperature once
	volt = get_voltage();		// Updates voltage
	update_scale_factor();

	state_check();
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
	//printf("\n%u,%u,%u,%u,%u,%u\n",c[0],c[1],c[2],c[3],c[4],c[5]);
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
	sysclk_enable_module(SYSCLK_PORT_C, PR_TWI_bm);
	sysclk_enable_peripheral_clock(&TWIC);

	imu_init();

	imu_update();

	ref_roll = imu_roll();
	ref_pitch = imu_pitch();
	//ref_yaw = imu_heading();
}

struct ac_config aca_config;
void hall_sensor_init(void){
	memset(&aca_config, 0, sizeof(struct ac_config));

	ac_set_mode(&aca_config, AC_MODE_SINGLE);
	ac_set_hysteresis(&aca_config, AC_HYSMODE_LARGE_gc);
	ac_set_voltage_scaler(&aca_config, scale_factor);
	ac_set_negative_reference(&aca_config, AC_MUXNEG_SCALER_gc);
	ac_set_positive_reference(&aca_config, AC_MUXPOS_PIN5_gc);
	
	ACA.AC0CTRL |= AC_HSMODE_bm;

	ac_set_interrupt_callback(&ACA, hall_sensor_measure);
	ac_set_interrupt_mode(&aca_config, AC_INT_MODE_RISING_EDGE);
	ac_set_interrupt_level(&aca_config, AC_INT_LVL_MED);

	ac_write_config(&ACA, 0, &aca_config);

	ac_enable(&ACA, 0);

}

void change_hall_sensor_scaler(void){
	ACA.CTRLB = scale_factor;
}

void release(void){
	servo_release();
}

double get_pressure(void){
	double val = 101325;
	
	uint32_t d1 = ms5607_convert_d1();
	uint32_t d2 = ms5607_convert_d2();

	//printf("%lx, %lx\n", d1, d2);

	double dt = d2 - (uint64_t)c[4] * 256.0;
	//double temp = 20.0 + ((uint64_t) dt * c[5]) / 8388608.0;
	long double off = (uint64_t) c[1] * 131072.0 + ((uint64_t) c[3] * dt) / 64.0;
	long double sens = (uint64_t) c[0] * 65536.0 + ((uint64_t) c[2] * dt) / 128.0;

	val = (double) (((double) d1 * sens / 2097152.0 - off) / 32768.0);
	
	return val;	// returns pressure in Pa
}

// ADC linear fit constants
double m =  0.000494760310;
double b = -0.093185924650;

double get_temperature(void){
	double val = 298.15; // Change to 25 degrees C
	volatile uint16_t reading = thermistor_read();
	double voltage = (m * reading + b); // m and b are collected from testing
	double resistance = 3180.0 * (3.30 - voltage) / voltage; // 6720 is the resistance of the steady resistor
	val = (1.0 / (3.354016E-3 + 2.569850E-4 * log(resistance / 10000.0) + 2.620131E-6 * pow(log(resistance / 10000.0), 2) + 6.383091E-8 * pow(log(resistance / 10000.0), 3))); // returns the temperature in hundredths of kelvin
	return val; //returns the temperature in kelvin
}

double get_altitude(double press){
	double val = 0;
	val = ground_t * (pow(ground_p / press, R * L / g_0) - 1) / L;
	return (val);		//returns altitude in meters
}

double get_voltage(void){
	double reading = (double) voltage_read();
	double voltage = (m * reading + b); // m and b are collected from testing
	voltage = voltage * 29600.0 / 12540.0 + voltage; // 6720 is the resistance of the steady resistor
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
	double y = imu_heading();

	if(abs(r) > 180){
		r = r - 360 * r / abs(r);
	}
	if(abs(p) > 90){
		p = p - 180 * p / abs(p);
	}
	if(abs(r) < 180 && abs(p) < 90){
		roll = r;
		pitch = p;
		angle = y;
	}
}

void pid_val(RingBuffer16_t* direct){
	double yaw = angle - ref_yaw;
	if(abs(yaw) > 180){
		yaw = yaw - 360 * yaw / abs(yaw);
	}
	int16_t y = (int16_t) (yaw *100); 
	rb16_write(direct,&y,1);
}

void state_check(void){
	switch(state){
		case 0:
			if((velocity < EPSILON_VELOCITY) && (alt > 450)){
				state++;
			}
			break;
		case 1:
			if(((velocity < EPSILON_VELOCITY) && (alt < 450)) || (released)){
				state++;
			}
			break;
		case 2:
			if((abs(velocity) < EPSILON_VELOCITY) || (alt < EPSILON_ALTITUDE)){
				state++;
			}
			break;
		case 3:
			break;
		default:			
			if(released){							//This case is state = 2 and state = 3
				state = 2;
				if(alt < EPSILON_ALTITUDE){
					state = 3;
				}
			}
			else{									//This case is state = 0 and state = 1
				state = 0;
				if(velocity < EPSILON_VELOCITY){
					state = 1;
				}
			}
			
	}
}

void release_servo_init(void){
	sysclk_enable_peripheral_clock(&TCD0); //enables peripheral clock for TCD0
	sysclk_enable_module(SYSCLK_PORT_D, SYSCLK_HIRES); //necessary jumbo

	PORTD.DIR |= 0x07;
	TCD0.CTRLA = 0x05; // sets the clock's divisor to 64
	TCD0.CTRLB = 0x23; // enables CCA and Single Waveform
	TCD0.PER = SERVO_WAVELENGTH; // sets the period (or the TOP value) to the period
	TCD0.CCC = TCD0.PER - RELEASE_SERVO_CLOSE; // makes the waveform be created for a duty cycle // 514.5 ticks per millisecond
}

void fin_servo_init(void){
	PORTA.DIR |= PIN2_bm;
	PORTA.OUT |= PIN2_bm;
	
	TCD0.CTRLB |= 0x50;
	
	TCD0.CCA = TCD0.PER - (SERVO_NEUTRAL_PULSE + SERVO_TRIM);		// 400 is 90 degrees
	TCD0.CCB = TCD0.CCA;		// 750 is middle
}

void servo_pid(RingBuffer16_t* direct){
	double k_p = .75;
	double k_i = 0.1;
	double k_d = -0.25;

	double p = 0;
	double i = 0;
	double d = 0;

	int16_t sum = 0;
	for(uint8_t i = 0; i < 10; i++){
		sum += rb16_get_nth(direct,i);
	}

	p = k_p * rb16_get_nth(direct, 0) / 100.0;
	i = k_i * sum / 100.0;
	d = k_d * diff(direct,rate);

	double val = p + i + d;
	//printf("%i\n",  (int16_t) val);
	
	uint16_t per = SERVO_NEUTRAL_PULSE + val;
	TCD0.CCA = TCD0.PER - (per + SERVO_TRIM); // makes the waveform be created for a duty cycle
	TCD0.CCB = TCD0.CCA;
}

void clock_init(void){
	sysclk_enable_peripheral_clock(&TCE0); // starts peripheral clock

	TCE0.CTRLA = 0x07; // divisor set to 1024 0x07
	TCE0.PER = 31249; // 1 Hz
	TCE0.INTCTRLA = TC_OVFINTLVL_LO_gc; // CCA int flag Lo level
}

void time_update(void){
	//packets++;
	
	packets++;
	sprintf(str,								format,										timer,										packets,
	(int16_t) (alt),							((int16_t) abs(alt*10))%10,					(int32_t) press,							(int16_t) (temp-273.15),  ((int16_t) (temp*10-2731.5))%10,
	(int16_t) volt,								((int16_t) (volt *10)) %10, 				(int16_t) (((int32_t)gps_t)/10000),			(int16_t) ((((int32_t)gps_t)%10000)/100),						(int16_t) (((int32_t)gps_t)%100),
	(int16_t) gps_lat,							((int32_t) (gps_lat*1000000))%1000000,		(int16_t) gps_long,							(int32_t)(abs(((int32_t)(gps_long*1000000))%1000000)),
	(int16_t) gps_alt,							((int16_t) (gps_alt)*10)%10,				gps_sats,									(int16_t) pitch,
	(int16_t) roll,								(int16_t) rpm,								state,										(int16_t) angle); // Data Logging Test
	printf(str);
	
	//printf("%i.%i, %i, %li, %i\n", timer/10, timer%10, (int16_t) alt, (int32_t) press, (int16_t) velocity);
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
	//TCD1.PER = 1907; // 262Hz (middle C)
	TCD1.CCA = 92;
}

void buzzer_stop(void){
	TCD1.PER = 0;
	TCD1.CCA = 0;
}

void calc_rpm(void){
	rpm = (rpm + ticks_per_sec * 60) / 2.0;
	ticks_per_sec = 0;
}

void update_scale_factor(void){
	double trigger_voltage = 0.425514285714 * volt - 0.100033333333;	// Calculates the trigger voltage from the input voltage
	scale_factor = (uint8_t) ((trigger_voltage * 64) / 3.3 - 1);
	change_hall_sensor_scaler();
}

static void hall_sensor_measure(AC_t *ac, uint8_t channel, enum ac_status_t status){
	ticks_per_sec++;
}

void command(uint8_t c){
	switch(c){
		case RESET:
			reset();
			break;
		case CALIBRATE:
			calibrate();
			break;
		case CALIBRATE_ALTITUDE:
			cali_alt();
			break;
		case CALIBRATE_ANGLE:
			cali_ang();
			break;
		case SERVO_RELEASE:
			servo_release();
			break;
		case SERVO_CLOSE:
			servo_close();
			break;
		case PACKET:
			packet();
			break;
	}
}

void reset(void){
	eeprom_erase();

	uint8_t oldInterruptState = SREG;	// no real need to store the interrupt context as the reset will pre-empt its restoration
	cli();		                        // Disable interrupts

	CCP = 0xD8;							// Configuration change protection: allow protected IO register write
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
	ref_roll = imu_roll();
	ref_pitch = imu_pitch();
	ref_yaw = imu_heading();
}

void servo_release(void){
	TCD0.CCC = TCD0.PER - RELEASE_SERVO_OPEN;

	released = 1;
}

void servo_close(void){
	TCD0.CCC = TCD0.PER - RELEASE_SERVO_CLOSE;

	released = 0;
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
	uint64_t p = 0;
	uint64_t t = 0;

	memcpy(&p, &ground_p, 8);
	memcpy(&t, &ground_t, 8);

	uint8_t data[] = {p & 0xFF, (p >> 8) & 0xFF, (p >> 16) & 0xFF, (p >> 24) & 0xFF, (p >> 32) & 0xFF, (p >> 40) & 0xFF, (p >> 48) & 0xFF, p >> 56,
					  t & 0xFF, (t >> 8) & 0xFF, (t >> 16) & 0xFF, (t >> 24) & 0xFF, (t >> 32) & 0xFF, (t >> 40) & 0xFF, (t >> 48) & 0xFF, t >> 56,};
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
	uint16_t v = (uint16_t) ((int16_t) velocity);

	check_write = timer % 100;

	// saves data and addresses in array
	volatile uint8_t data[] = {a >> 8, a & 0xFF, packets >> 8, packets & 0xFF, timer >> 8, timer & 0xFF, v >> 8, v & 0xFF, check_write, check_write, state, released};
	volatile uint8_t addresses[] = {ALT_ADDR_BYTE1, ALT_ADDR_BYTE0, PACKET_ADDR_BYTE1, PACKET_ADDR_BYTE0, TIME_ADDR_BYTE1, TIME_ADDR_BYTE0, VEL_ADDR_BYTE1, VEL_ADDR_BYTE0, CHECK_WRITE_BYTE0, CHECK_WRITE_BYTE1, STATE_BYTE, RELEASED_BYTE};

	// Writes the NVM Registers to write the buffer
	NVM.CMD = LOAD_BUFFER_CMD;
	for(uint8_t i = 0; i < 12; i++){
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
	timer++;
	time_flag = 1;
}


ISR(USARTE0_RXC_vect){
	xbee_comm = usart_getchar(UART_TERMINAL_SERIAL);
	//printf("%c\n", xbee_comm);
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