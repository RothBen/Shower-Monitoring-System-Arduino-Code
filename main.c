/*
 * showerMonitor.c
 *
 * Created: 4/26/2018 1:13:38 PM
 * Author : Mike Piron
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "uart.h"

#define	ADC_PIN 0
#define LED_PIN PB0
#define ADC_THRESHOLD 512
#define UART_BAUD_RATE 115200

uint16_t adcVal;
uint16_t flow = 123;
uint16_t readings[2] = {0, 0};
uint16_t data[5] = {0,0,0,0,0};
uint8_t repeat = 0;
uint8_t stopBit = 0;
uint8_t i = 0;
uint8_t reg = 0;
uint8_t endFlag = 0;
char adcBuffer[255];
char flowBuffer[255];
char btBuffer[255];

void sendCommand(uint8_t command)
{
	PORTB = 0x00;
	PORTC = command;
	_delay_ms(3);
	PORTB = 0b100;
	_delay_ms(3);
	PORTB = 0b000;
	_delay_ms(3);
}

void sendChar(uint8_t character)
{
	PORTB = 0b001;
	PORTC = character;
	_delay_ms(5);
	PORTB = 0b101;
	_delay_ms(5);
	PORTB = 0b001;
	_delay_ms(5);
}

void sendInt(int num)
{
	double frac = 0;
	int place = 0;
	int temp;

	frac = num;

	while (frac >= 1)
	{
		place++;
		frac = frac/10;
	}

	while (place > 0)
	{
		place--;
		frac = frac*10;
		temp = (int) (frac);
		sendChar(0b00110000+temp);
		frac = frac - temp;
	}
}

void lcdInit()
{
	sendCommand(0b00111100);
	sendCommand(0b1110); //set cursor to be invisible
	sendCommand(0b1); //clear display
}

uint16_t read_adc(uint8_t channel){
	ADMUX &= 0xF0;                    //Clear the older channel that was read
	ADMUX |= channel;                //Defines the new ADC channel to be read
	ADCSRA |= (1<<ADSC);                //Starts a new conversion
	while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
	return ADCW;                    //Returns the ADC value of the chosen channel
}

void adc_init()
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
	ADMUX |= (1<<REFS0);                //Voltage reference from Avcc (5v)
	ADCSRA |= (1<<ADEN);                //Turn on ADC
	ADCSRA |= (1<<ADSC);                //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}

int main(void)
{
	DDRC = 0xFF;
	DDRB = 0xFF;
	DDRL = 0x00;
	PORTB = 0;
	PORTC = 0;
	
	sei();
	
	uart1_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	uart1_puts("$");
	uart1_puts("$");
	uart1_puts("$");
	lcdInit();
	adc_init();
	//init input capture mode
	TCCR4B |= (1 << ICNC4)|(1 << ICES4); //trigger interrupt on rising edge. no prescaler
	TIMSK4 |= (1 << ICIE4)|(1 << TOIE4);//set input capture and timer overflow interrupts
	
	ADCSRA |= (1 << ADEN); //start adc
	TCCR4B |= (1 << CS40);
	
	
    while (1) 
    {
		
	}
	return 0;
}

ISR (TIMER4_OVF_vect)
{
	uart1_puts("z");
	ADCSRA &= (0 << ADEN); //stop adc
	sendCommand(1);
	//TCCR4B &= (0 << CS40); //stop input capture timer
}

ISR(TIMER4_CAPT_vect)
{
	if (repeat < 5)
	{
		if (i == 0)
		{
			readings[i] = ICR4;
			i++;
		}
		else if (i == 1)
		{
			readings[i] = ICR4;
			if (readings[1] > readings[0])
			{
				readings[2] = readings[1] - readings[0];
				repeat++;
			}
			i = 0;
		}
	}
	else
	{
		sendCommand(0b1);
		//read adc
		adc_init();
		ADCSRA |= (1 << ADEN); //start adc
		adcVal = read_adc(0);
		//send values to lcd
		sendInt(readings[2]);
		sendChar(0b00100000);
		sendInt(adcVal);
		//send values to bluetooth
		itoa(adcVal, adcBuffer, 10);
		itoa(readings[2], flowBuffer, 10);	
		
		uart1_puts("a");
		uart1_puts(adcBuffer);
		uart1_puts("f");
		uart1_puts(flowBuffer);
		uart1_puts("x");
		
		repeat = 0;
		i = 0;
		TCCR4B |= (1 << CS40);
		_delay_ms(500);
	}
}