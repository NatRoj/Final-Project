/*
 * FinalProject.c
 *
 * Created: 27-Apr-21 9:43:16 PM
 * Author : rojva
 */ 

#include <avr/io.h>
#include  <avr/interrupt.h>
#include <avr/eeprom.h> 
#include <string.h>
#define F_CPU 16000000UL
#define USART_BAUDRATE 9600
#define BAUD_PRESCALER (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
#define __DELAY_BACKWARD_COMPATIBLE__						//allow us to use variable in the delay function
#include <util/delay.h>
#include "DHT11sensor v1.0.h"					//Libraries for LCD and DHT11
#include "OnLCDLib.h"

void adc_init(void);							//Function to initialize/configure the ADC
void USART_init(void);
void USART_send( unsigned char data);			//Function that sends a char over the serial port
void USART_putstring(char* StringPtr);			//Function that sends a string over the serial port
char* itoa(int, char* , int);					//the itoa function

uint16_t read_adc(uint8_t channel);				//Function to read an arbitrary analog pin
uint16_t adc_value;								//Variable used to store the value read from the ADC
int8_t DHTreturnCode;
int dt;  //Length of time between measurements in s
int temp;    //Boolean whether to display temperature
int bright;  //Boolean whether to display brightness
int humid;   //Boolean whether to display humidity
int all;
int fahren;  //Boolean to determine whether temp is C or F
int Baddr = 1;
int Taddr = 2;
int Haddr = 3; 
int sensorValue = 0;
int readValue = 2;
char buffer[5];	

int main(void)
{
	all = 1;
	dt = 1;
	fahren = 0;
	LCDSetup(LCD_CURSOR_NONE);	
	adc_init();
	USART_init();

	USART_putstring("Measurements are being taken 1s apart. \r \n");
	USART_putstring("To specify the spacing of measurements in seconds, enter the command Sx, where x is desired seconds. \r \n");
	USART_putstring("To specify which measurement to view, enter the command: \r \n");
	USART_putstring("T for temperature, \r \n");
	USART_putstring("B for brightness, \r \n");
	USART_putstring("H for humidity, or \r \n");
	USART_putstring("A for all measurements. \r \n");
	USART_putstring("To display temperature in Fahrenheit, enter the command F. \r \n");
	USART_putstring("To display temperature in Celsius, enter the command C. \r \n");
	USART_putstring("To store brightness in EEPROM, enter the command W. \r \n");
	USART_putstring("To read brightness stored in EEPROM, enter the command D. \r \n");
	USART_putstring("To store temperature (in C) in EEPROM, enter the command Q. \r \n");
	USART_putstring("To read temperature (in C) stored in EEPROM, enter the command G. \r \n");	
	USART_putstring("To store humidity in EEPROM, enter the command J. \r \n");
	USART_putstring("To read humidity stored in EEPROM, enter the command K. \r \n");
		
    while (1) 
    {
		_delay_ms(dt*1000);
	
		DHTreturnCode = DHT11ReadData();		//Function to read and check the sensor data
		if(DHTreturnCode == -1){
			LCDHome();
			LCDWriteString("Checksum Error");	//Error message on lcd to show data was received incorrectly
		} else {
			if (all == 1) {
				LCDHome();
				if (fahren == 1) {
					DHT11DisplayTemperatureF();			//Display Temp F
				} else {
					DHT11DisplayTemperatureC();			//Display Temp C
				}
				LCDGotoXY(8,1);
				LCDWriteString("B:");				//Display Brightness
				adc_value =  ((float)read_adc(0)/2000)*100;
				LCDWriteInt(adc_value,3);
				LCDWriteString("%");
				LCDGotoXY(1,2);
				DHT11DisplayHumidity();				//Display Humidity
			}
			else if (temp == 1) {
				LCDHome(); 
				if (fahren == 1) {
					DHT11DisplayTemperatureF();			//Display Temp F
				} else {
					DHT11DisplayTemperatureC();			//Display Temp C
				}
			}
			else if (bright == 1) {
				LCDHome();
				LCDWriteString("B:");				//Display Brightness
				adc_value =  ((float)read_adc(0)/5050)*100;
				LCDWriteInt(adc_value,3);
				LCDWriteString("%");
			}
			else if (humid == 1) {
				LCDHome();
				DHT11DisplayHumidity();				//Display Humidity
			}
		}
    }
}

void adc_init(void){
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));		//16Mhz/128 = 125Khz the ADC reference clock
	ADMUX |= (1<<REFS0);								//Voltage reference from Avcc (5v)
	ADCSRA |= (1<<ADEN);								//Turn on ADC
	ADCSRA |= (1<<ADSC);								//Do an initial conversion because this one is the slowest
}

uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xF0;						//Clear the older channel that was read
	ADMUX |= channel;					//Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));          //Wait until the conversion is done
	return (ADCW*5);                    //Returns the ADC value of the chosen channel
}

void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);	//Setting Baud rate
	UBRR0L = (uint8_t)(BAUD_PRESCALER);		//Setting Baud rate
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);			//Enable receiver (RXEN0) and transmitter (TXENO)
	UCSR0C = (1<<UCSR0B)|(3<<UCSZ00);		//From datasheet, set format:8data, 2stop bit
	UCSR0B |= (1 << RXCIE0); // Enable the USART Receive Complete interrupt 
	sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed
}

ISR(USART_RX_vect) {
	char ReceivedByte;
	ReceivedByte = UDR0; // Fetch the received byte value into the variable "ByteReceived"
	if (ReceivedByte == 'S') {
		ReceivedByte = UDR0;
		dt = ReceivedByte - 48; //Change interval with which measurements are taken
	}
	else if (ReceivedByte == 'T') {
		LCDClear();
		temp = 1;
		all = 0;
		bright = 0;
		humid = 0;
	}
	else if (ReceivedByte == 'B') {
		LCDClear();
		bright = 1;
		all = 0;
		temp = 0;
		humid = 0;
	}
	else if (ReceivedByte == 'H') {
		LCDClear();
		humid = 1;
		all = 0;
		bright = 0;
		temp = 0;
	}
	else if (ReceivedByte == 'A') {
		LCDClear();
		all = 1;
		temp = 0;
		bright = 0;
		humid = 0;
	}
	else if (ReceivedByte == 'F') {
		fahren = 1;
	}
	else if (ReceivedByte == 'C') {
		fahren = 0;
	}
	else if (ReceivedByte == 'W') { //store brightness in EEPROM
		while (!eeprom_is_ready());
		cli();
		eeprom_write_word((uint16_t*)Baddr, adc_value);
		sei();
	}
	else if (ReceivedByte == 'D') { //read brightness stored in EEPROM
		while (!eeprom_is_ready());
		cli();
		readValue = eeprom_read_word((uint16_t*)Baddr); // => sensorValue
		sei();
		itoa(readValue, buffer,10);
		USART_putstring("\r \n Stored brightness value = ");
		USART_putstring(buffer);
	}
	else if (ReceivedByte == 'Q') { //store temperature (in C) in EEPROM
		DHT11WriteTemperatureEEPROM();
	}
	else if (ReceivedByte == 'G') { //read temperature (in C) stored in EEPROM
		while (!eeprom_is_ready());
		cli();
		readValue = eeprom_read_word((uint16_t*)Taddr); // => sensorValue
		sei();
		itoa(readValue, buffer,10);
		USART_putstring("\r \n Stored temperature value = ");
		USART_putstring(buffer);
		USART_putstring("C \r \n");
	}
	else if (ReceivedByte == 'J') { //store humidity in EEPROM
		DHT11WriteHumidityEEPROM();
	}
	else if (ReceivedByte == 'K') { //read humidity stored in EEPROMwhile (!eeprom_is_ready());
		cli();
		readValue = eeprom_read_word((uint16_t*)Haddr); // => sensorValue
		sei();
		itoa(readValue, buffer,10);
		USART_putstring("\r \n Stored humidity value = ");
		USART_putstring(buffer);
		USART_putstring("% \r \n");
	}
	
	
	UDR0 = ReceivedByte; //echo
	ReceivedByte = UDR0; // Next char
}

void USART_send(unsigned char data){
	while(!(UCSR0A & (1<<UDRE0)));			//While UDRE0 (USART Data Register flag) is clear,
	UDR0 = data;	
}

void USART_putstring(char* StringPtr){
	while(*StringPtr != 0x00){
		USART_send(*StringPtr);
	StringPtr++;}	
}
