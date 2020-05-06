/*
 * PWMMotorCtrl.cpp
 *
 * Created: 06.05.2020 23:15:51
 * Author : V.S.Kozlov
 */ 

#define F_CPU 9600000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#define BtnDwn		PINB1
#define BtnUp		PINB2
#define MotPWM		OCR0A

void portInit(void)
{
	DDRB = 0b00000001;
	PORTB =0b00000110;
}

void pwmInit(void)
{
	TCCR0A = 0b10000011; //FastPWM non invert
	TCCR0B = 0b00000011;	// PWM 9600000/256/64
	TCNT0 = 255;
	OCR0A = 0;
}

void wdt_On(void)
{
	wdt_reset();
	WDTCR |= (1<<WDCE) | (1<<WDE);
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
   
    while (1) 
    {
		if (~PINB & (1<<BtnDwn)) Motlvl--;
		if (~PINB & (1<<BtnUp)) Motlvl++;
		if (Motlvl>=255) Motlvl = 254;
		if (Motlvl <=0) Motlvl = 1;
		MotPWM = Motlvl;
		wdt_reset();
		_delay_ms(1);
    }
}

