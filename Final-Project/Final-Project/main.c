/*
 * FinalProject.c
 *
 * Created: 27-Apr-21 9:43:16 PM
 * Author : rojva
 */ 

#include <avr/io.h>
#include  <avr/interrupt.h>
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

uint16_t read_adc(uint8_t channel);				//Function to read an arbitrary analog pin
uint16_t adc_value;								//Variable used to store the value read from the ADC
int8_t DHTreturnCode;
int dt = 1;  //Length of time between measurements in s
int delay;   //Length of time between measurements in ns
int temp;    //Boolean whether to display temperature
int bright;  //Boolean whether to display brightness
int humid;   //Boolean whether to display humidity

int main(void)
{
	//Initialize LCD
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
	
    while (1) 
    {
		delay = dt*1000000000;		//Delay is in ns
		_delay_ms(delay);
		DHTreturnCode = DHT11ReadData();		//Function to read and check the sensor data
		
		if(DHTreturnCode == -1){
			LCDHome();
			LCDWriteString("Checksum Error");	//Error message on lcd to show data was received incorrectly
		} else{
			LCDHome();
			DHT11DisplayTemperature();			//Display Temp
			LCDGotoXY(8,1);
			LCDWriteString("B:");				//Display Brightness
			adc_value = ((float)read_adc(0)/5050)*100;
			LCDWriteInt(adc_value,3);
			LCDWriteString("%");
			LCDGotoXY(1,2);
			DHT11DisplayHumidity();				//Display Humidity
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
	UCSR0B |= (1 << RXCIE0); // Enable the USART Receive Complete interrupt (USART_RXC)
	sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed
}

ISR(USART_RX_vect) {
	char ReceivedByte;
	ReceivedByte = UDR0; // Fetch the received byte value into the variable "ByteReceived"
	if (ReceivedByte == 'S') {
		ReceivedByte = UDR0;
		dt = ReceivedByte - 48;
	}
	UDR0 = ReceivedByte; // Echo back the received byte back to the computer
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
