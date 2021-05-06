/*_______________________________________________________________________________
Copyright 2015 Istrate Liviu

DHT11 Humidity and Temperature Sensor v1.0

HOW TO USE AND DESCRIPTION
--------------------------
*	uint8_t DHT11ReadData(void):
This function must be executed first to read temperature and humidity from sensor. This function will
put the data in "DHT11Data" array. DHT11Data[0] holds the humidity value, DHT11Data[2] temperature,
DHT11Data[4] checksum. You don't need to access them directly instead use the functions described bellow.

This function incorporates safety checks to prevent the MCU remaining stuck in a while loop while
waiting for sensor responses, in case the sensor breaks and remain in a HIGH or LOW state.
Also the function returns 1 if no errors occurs or 0 if an error occured. You can customise the error
codes and check them in your functions to take proper actions. For example if the error code means
a checksum error you can run the function again to get a correct reading.

*	uint8_t DHT11ReadDataAvg(void):
Takes x number of samples and puts the average in DTH11Data array. x is defined by DHT_NR_OF_SAMPLES.
The sampling rate is 1 per second, maximum value supported by DHT11 sensor. Averaging the values 
prevents the values to fluctuate over a short time. Beeing a cheap sensor DHT11 can fluctuate 2, 3
degrees on consecutive measurements.

DHT_TEMP_ERROR_OFFSET - Using this function you can use an offset value by defining DHT_TEMP_ERROR_OFFSET.
Using another thermometer find room temperature and substract from sensor reading to find offset error.
USE_MCU_ONCHIP_SENSOR - You can define this if your MCU has an on-chip temperature sensor, and the average
of both sensors will be used thus providing better results.
There are two functions "DHT11ReadDataAvg". One of them is commented. You can try each of them to see
which is better.

*	void DHT11DisplayTemperature(void):
After "DHT11ReadData" has been executed use this function to display temperature value on an LCD.
Before that move LCD cursor to a proper location where you want temperature to be displayed.
After the numeric value, this function will display °C

*	void DHT11DisplayHumidity(void):
After "DHT11ReadData" has been executed use this function to display humidity value on an LCD.
Before that move LCD cursor to a proper location where you want humidity to be displayed.
After the numeric value, this function will display %

Tips:
- defining ADD_MINIMUM_DELAY will add a delay of 1 second between sensor readings in case you forget
to add in your code.
- don't put the sensor near a voltage regulator or other heat sources.

NOTICE
--------------------------------------------------------------------
Free for private, non-commercial use

AUTHOR: Istrate Liviu
__________________________________________________________________________________*/

#ifndef DHT11
#define DHT11

/*************************************************************
	INCLUDES
**************************************************************/
#include <avr/io.h>
#include <avr/eeprom.h> 
#include <util/delay.h>
#include "OnLCDLib.h"

/*************************************************************
	DEFINE SETUP
**************************************************************/
#define SENSOR_DDR				DDRB
#define SENSOR_PORT				PORTB
#define SENSOR_PIN				PINB
#define SENSOR_PIN_BIT			PB0
#define SAMPLE_DELAY			1000 // DHT11 has a maximum sampling rate of 1 per second
#define DHT_NR_OF_SAMPLES		5	 // Number of samples used for averaging the values
#define DHT_TEMP_ERROR_OFFSET	0    // In degrees. If positive, will be added to final result, if negative, will be subtracted
#define ADD_MINIMUM_DELAY		 	 // Comment out if you want to add the delay in your code

/*************************************************************
	FUNCTION PROTOTYPES
**************************************************************/
void DHT11Setup(void);
void DHT11DisplayTemperatureC(void);
void DHT11DisplayTemperatureF(void);
void DHT11WriteTemperatureEEPROM(void);
void DHT11WriteHumidityEEPROM(void);
void DHT11DisplayHumidity(void);
void DHT11ReadDataAvg(void);
int8_t DHT11ReadData(void);


/*************************************************************
	GLOBAL VARIABLES
**************************************************************/
uint8_t DHT11Data[5] = {0};
static uint8_t DHT11Init = 0;


/*************************************************************
	FUNCTIONS
**************************************************************/
void DHT11Setup(){
	// Wait for the sensor to stabilise on power on
	_delay_ms(2000);
	
	// Set DDR to output
	SENSOR_DDR |= 1 << SENSOR_PIN_BIT;
	
	// Set setup flag
	DHT11Init = 1;
}

void DHT11DisplayTemperatureF(){
	LCDWriteString("T:");
	LCDWriteInt(((DHT11Data[2]*1.8)+32) + DHT_TEMP_ERROR_OFFSET, 2);
	LCDData(0b11011111);
	LCDData('F');
}

void DHT11DisplayTemperatureC(){
	LCDWriteString("T:");
	LCDWriteInt(DHT11Data[2] + DHT_TEMP_ERROR_OFFSET, 2);
	LCDData(0b11011111);
	LCDData('C');
}

void DHT11WriteTemperatureEEPROM(void) {
	while (!eeprom_is_ready());
	cli();
	eeprom_write_word((uint16_t*)2, DHT11Data[2]);
	sei();
}

void DHT11DisplayHumidity(){
	LCDWriteString("H:");
	LCDWriteInt(DHT11Data[0], 2);
	LCDData('%');
}

void DHT11WriteHumidityEEPROM(void) {
	while (!eeprom_is_ready());
	cli();
	eeprom_write_word((uint16_t*)3, DHT11Data[0]);
	sei();
}

void DHT11ReadDataAvg(){
	uint8_t i;
	uint16_t bufferTemp=0;
	uint16_t bufferRH=0;
	
	for(i=DHT_NR_OF_SAMPLES; i>0; i--){
		// Read data from sensor. If the function is not returning 1
		// it means there was an error so it skips a reading
		if(DHT11ReadData() != 1) continue;
		
		// Sum all values in the array
		bufferRH += DHT11Data[0];
		bufferTemp += DHT11Data[2];
	}
	
	// Humidity average
	DHT11Data[0] = bufferRH / DHT_NR_OF_SAMPLES;
	
	// Temperature average
	DHT11Data[2] = (bufferTemp / DHT_NR_OF_SAMPLES) + DHT_TEMP_ERROR_OFFSET;
}

int8_t DHT11ReadData(){
	uint8_t sensor_bytes, bits, buffer=0, timeout=0, checksum;
	
	/* Initialise sensor if flag is 0 then set to 1 to run only once */
	if(DHT11Init == 0) DHT11Setup();
	
	/* Send START signal to sensor */
	SENSOR_DDR |= (1 << SENSOR_PIN_BIT); // set pin to output
	SENSOR_PORT &= ~(1 << SENSOR_PIN_BIT); // set pin LOW
	_delay_ms(20); // keep pin LOW for at least 18 ms

	/* Set DDR to input LOW (high Z) to read data from sensor. 
	The external pull-up resistor will pull the data line HIGH */
	SENSOR_DDR &= ~(1 << SENSOR_PIN_BIT);
	_delay_us(32); // wait for 20-40 us
	
	/* Listen for sensor response - 80us LOW and 80us HIGH signal */
	if(SENSOR_PIN & (1 << SENSOR_PIN_BIT)){ // If HIGH, sensor didn't respond
		return 0; // error code
	}
	
	/* Sensor sent LOW signal, wait for HIGH */
	_delay_us(82);
	
	/* If HIGH, sensor is ready to send data */
	if(SENSOR_PIN & (1 << SENSOR_PIN_BIT)){
		_delay_us(82); // wait for HIGH signal to end
		if(SENSOR_PIN & (1 << SENSOR_PIN_BIT)) return 0; // still HIGH - something is wrong
	}else{
		return 0; // error code
	}
	
	/* Ready to read data from sensor */
	for(sensor_bytes=0; sensor_bytes<5; sensor_bytes++){
		/* Reset the buffer */
		buffer = 0;
			
		for(bits=0; bits<8; bits++){
			/* Wait 50 us between each bits while signal is LOW */
			while(~SENSOR_PIN & (1 << SENSOR_PIN_BIT)){
				/* Wait no more than 80 us. If the sensor breaks and remains LOW
				the MCU will not remain stuck in a while loop */
				timeout++;
				if(timeout > 8) break;
				_delay_us(10);
			}
			timeout = 0;
			
			/* Signal is HIGH - read the bit */
			if(SENSOR_PIN & (1 << SENSOR_PIN_BIT)){
				_delay_us(40); // 26-28 us HIGH means a 0 bit, 70 us means a 1 bit
				/* If signal is still HIGH means a 1 bit */
				if(SENSOR_PIN & (1 << SENSOR_PIN_BIT)){
					/* Put a 1 to buffer. Sensor sends MSB first */
					buffer |= 1 << (7-bits);
				}
				
				/* Wait for HIGH signal to end */
				while(SENSOR_PIN & (1 << SENSOR_PIN_BIT)){
					/* Wait no more than 80 us. If the sensor breaks and remains HIGH
					the MCU will not remain stuck in a while loop */
					timeout++;
					if(timeout > 8) break;
					_delay_us(10);
				}
				timeout = 0;
			}else{
				return 0; // signal still LOW. Return error response
			}
		}
		
		/* Dump the buffer to global array */
		DHT11Data[sensor_bytes] = buffer;
	}
	
	/* Wait for data transmision to end. Sensor will output LOW for 50 us and then goes into
	low-power consumption mode until the next START command from the MCU */
	_delay_us(60);
	
	/* Set DDR to output */
	//SENSOR_DDR |= 1 << SENSOR_PIN_BIT;
	
	/* Set pin HIGH. When idle, sensor DATA line must be kept HIGH */
	//SENSOR_PORT |= 1 << SENSOR_PIN_BIT;
	
	/* Check for data transmission errors */
	checksum = DHT11Data[0] + DHT11Data[1] + DHT11Data[2] + DHT11Data[3];
	if(checksum != DHT11Data[4]){
		return -1; // checksum error code
	}
	
	#ifdef ADD_MINIMUM_DELAY
		_delay_ms(SAMPLE_DELAY);
	#endif
	
	/* OK return code */
	return 1;
}
#endif