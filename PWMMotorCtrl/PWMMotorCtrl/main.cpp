/*
 * PWMMotorCtrl.cpp
 *
 * Created: 06.05.2020 23:15:51
 * Author : V.S.Kozlov
 * firmware for controlling the water pump
 * URL https://easyeda.com/MasterWind007/rpm-controller
 */ 

#define F_CPU 9600000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#define BtnDwn		PINB1 // RPM Down
#define BtnUp		PINB2 // RPM Up
#define Jp1			PINB4 //jumper to ADC Enabled on board
#define MotPWM		OCR0A 

void portInit(void)
{
	DDRB = 0b00000001;
	PORTB =0b00010110;
}

void adcInit(void)
{
	ADCSRA = 0b11000111; //Enabled, StartConversion, F_CPU/128  
	ADMUX  = 0b01100011; // AREF-internal, ADLAR-Right, input-ADC3
}

void pwmInit(void)
{
	TCCR0A = 0b10000011; //FastPWM non invert OCR0A enabled
	TCCR0B = 0b00000011; //PWM frequency  F_CPU/256/64
	TCNT0 = 255;
	OCR0A = 0;
}

uint8_t adcRead(void)
{
	uint8_t adc = 0;
	ADCSRA |= (1 << ADSC);
	adc = ADCH;
	return adc;
} 
void wdt_On(void)
{
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE); //WDT Change Enabled, WDT enabled
	WDTCR = (1<<WDE) | (1<<WDP2) | (1<<WDP1); // WDT prescaler (time-out) 1s
}

void wdt_Off(void)
{
	wdt_reset();
	MCUSR &= ~(1<<WDRF);	
	WDTCR |= (1<<WDCE) | (1<<WDE);
	WDTCR = 0;
}

int main(void)
{
	wdt_Off();
	uint8_t Motlvl = 0;
	portInit();
	pwmInit();
	wdt_On();
    adcInit();
	while (1) 
    {
		if (~PINB & (1<<Jp1)) Motlvl = adcRead(); 
		if (~PINB & (1<<BtnDwn)) Motlvl--;
		if (~PINB & (1<<BtnUp)) Motlvl++;
		if (Motlvl>=254) Motlvl = 255;
		if (Motlvl <=0) Motlvl = 1;
		MotPWM = Motlvl;
		wdt_reset();
		_delay_ms(1);
    }
}

