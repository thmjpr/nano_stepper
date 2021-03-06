/**********************************************************************
 *      Author: tstern
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

//#ifdef A4954_DRIVER
#include "A4954.h"
#include "wiring_private.h"
#include "syslog.h"
#include "angle.h"
#include "Arduino.h"
#include "sine.h"

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

#define DAC_MAX (0x01FFL)		//PWM top counter value
static uint8_t pinState = 0;

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));

static void syncTCC(Tcc* TCCx) {
	while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK)
	{
	}
}

//Configure pins going to A4954
void A4954::begin()
{
	//setup the A4954 pins
	digitalWrite(PIN_A4954_IN3, LOW);
	pinMode(PIN_A4954_IN3, OUTPUT);
	digitalWrite(PIN_A4954_IN4, LOW);
	pinMode(PIN_A4954_IN4, OUTPUT);
	digitalWrite(PIN_A4954_IN2, LOW);
	pinMode(PIN_A4954_IN2, OUTPUT);
	digitalWrite(PIN_A4954_IN1, LOW);
	pinMode(PIN_A4954_IN1, OUTPUT);

	//setup the PWM for current on the A4954, set for low current
	digitalWrite(PIN_A4954_VREF12, LOW);
	digitalWrite(PIN_A4954_VREF34, LOW);
	pinMode(PIN_A4954_VREF34, OUTPUT);
	pinMode(PIN_A4954_VREF12, OUTPUT);

	enabled = true;
	lastStepMicros = 0;
	forwardRotation = true;

	//enableTCC0(70);  					//Init at moderate current, for initial step test
	setupDAC();
	
	return;
}


//TCC0 has 4 channels (0-3, 4-7 shared). TCC1, TCC2 2 channels

//IN1 PA05 - TCC0 WO[1]	- E		(shared with 5)
//IN2 PA21 - TCC0 WO[7] - F
//IN3 PA15 - TCC0 WO[5] - F
//IN4 PA20 - TCC0 WO[6] - F

#define		TMTR	TCC0
#define		IN1		TMTR->CC[1].reg
#define		IN2		TMTR->CC[3].reg
#define		IN3		TMTR->CC[1].reg		//can change?
#define		IN4		TMTR->CC[2].reg

//PINCFGy.PMUXEN can be 1 to enable connection between peripheral functions and IO pins
//PMUXn select peripheral function for corresponding pin

//Peripheral function selected by setting PMUXO or PMUXE in PMUXn register
//PMUXO = odd number pin
//PMUXE = even number pin

//can disable completely: PULLEN, INEN, DIR = 0
//DIRSET/DIRCLR register

static inline void bridge1(drvStates state)
{
	if (state == drvStates::Forward)		//Forward -> chop if Isns > Vref
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN1].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN1].ulPin].bit.PMUXEN = 0;		//Disable mux to peripheral
		GPIO_OUTPUT(PIN_A4954_IN1);
		GPIO_OUTPUT(PIN_A4954_IN2);			//any point in doing this?
		GPIO_HIGH(PIN_A4954_IN1);
		GPIO_LOW(PIN_A4954_IN2);
		//pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT);
		pinState = (pinState & 0x0C) | 0x1;
	}
	else if (state == drvStates::Reverse)	//Reverse -> chop if Isns > Vref
	{
		PORT->Group[g_APinDescription[PIN_A4954_IN2].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN2].ulPin].bit.PMUXEN = 0;
		GPIO_OUTPUT(PIN_A4954_IN2);
		GPIO_OUTPUT(PIN_A4954_IN1);
		//pinMode(PIN_A4954_IN1, OUTPUT);  //needed? can PWM be set to 100% or something instead.
		GPIO_LOW(PIN_A4954_IN1);
		GPIO_HIGH(PIN_A4954_IN2);
		//pinPeripheral(PIN_A4954_IN1, PIO_TIMER);
		pinState = (pinState & 0x0C) | 0x2;
	}
	else if(state == drvStates::Brake)		//Short coils out, causing braking
	{
		GPIO_HIGH(PIN_A4954_IN1);
		GPIO_HIGH(PIN_A4954_IN2);
	}	
	else if (state == drvStates::HiZ)		//high-z output (coasting)
	{
		GPIO_LOW(PIN_A4954_IN1);
		GPIO_LOW(PIN_A4954_IN2);
	}
	
	else
	{
	//nogo
	}
}

static inline void bridge2(drvStates state)
{
	if (state == drvStates::Forward)
		{
			PORT->Group[g_APinDescription[PIN_A4954_IN3].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN3].ulPin].bit.PMUXEN = 0;
			GPIO_OUTPUT(PIN_A4954_IN3);
			GPIO_OUTPUT(PIN_A4954_IN4);
			GPIO_HIGH(PIN_A4954_IN3);
			GPIO_LOW(PIN_A4954_IN4);
			pinState = (pinState & 0x03) | 0x4;
		}
	else if (state == drvStates::Reverse)
		{
			PORT->Group[g_APinDescription[PIN_A4954_IN4].ulPort].PINCFG[g_APinDescription[PIN_A4954_IN4].ulPin].bit.PMUXEN = 0;
			GPIO_OUTPUT(PIN_A4954_IN3);
			GPIO_OUTPUT(PIN_A4954_IN4);
			GPIO_LOW(PIN_A4954_IN3);
			GPIO_HIGH(PIN_A4954_IN4);
			pinState = (pinState & 0x03) | 0x8;
		}
	else if(state == drvStates::Brake)
		{
			GPIO_HIGH(PIN_A4954_IN3);
			GPIO_HIGH(PIN_A4954_IN4);
		}
	else if (state == drvStates::HiZ)
		{
			GPIO_LOW(PIN_A4954_IN3);
			GPIO_LOW(PIN_A4954_IN4);
		}
	else
	{
		
	}
}


//TCC0 PWM signals to H-bridge IN1,2,3,4
void A4954::enableTCC0(uint8_t percent)
{
#ifdef MECHADUINO_HARDWARE
	return;
#else
	Tcc* TCCx = TCC0;

	uint32_t ulValue = ((uint32_t)(100 - percent) * (DAC_MAX - 1)) / 100;		//Convert percent to 0 to DAC_MAX

	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));

	while (GCLK->STATUS.bit.SYNCBUSY == 1);

	//ERROR("Setting TCC %d %d",ulValue,ulPin);
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	syncTCC(TCCx);

	// Set TCx as normal PWM
	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	syncTCC(TCCx);
	
	// Set TCx in waveform mode Normal PWM
	//TOP = PER
	//output on match = set
	//output on update = clear
	
	TCCx->CC[1].reg = (uint32_t)ulValue; //ch5 //IN3
	TCCx->WAVE.bit.POL1 = 0;			//Compare initialized to ~DIR, set to DIR on CCx match.
	syncTCC(TCCx);

	TCCx->CC[2].reg = (uint32_t)ulValue; //ch6 //IN4
	syncTCC(TCCx);

	TCCx->CC[3].reg = (uint32_t)ulValue; //ch7  //IN2
	syncTCC(TCCx);

	TCCx->CC[1].reg = (uint32_t)ulValue; //ch1 == ch5 //IN1
	syncTCC(TCCx);

	// Set PER to maximum counter value (resolution : 0x1FF)
	TCCx->PER.reg = DAC_MAX;
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE;
	syncTCC(TCCx);
	
	//Set pins to output A-H peripheral
	pinPeripheral(PIN_A4954_IN1, PIO_TIMER);		//mux to E
	pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT);	//mux to F
	pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);	//
	pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);	//
#endif
}

//Set motor percent
void A4954::setMotorPWM(uint8_t percent)
{
	Tcc* TCCx = TCC0;
	uint32_t ulValue = ((uint32_t)(100 - percent) * (DAC_MAX - 1)) / 100;
	
	// Set TCx in waveform mode Normal PWM
	TCCx->CC[1].reg = (uint32_t)ulValue;  //ch5 //IN3
	TCCx->CC[2].reg = (uint32_t)ulValue;  //ch6 //IN4
	TCCx->CC[3].reg = (uint32_t)ulValue;  //ch7  //IN2
	//TCCx->CC[1].reg = (uint32_t)ulValue;  //ch1 == ch5 //IN1  don't need to set twice
	syncTCC(TCCx);			//sync necessary where?
}


//Set the specific H-bridge inputs to use PWM signal to limit current
void A4954::limitCurrent(uint8_t percent)
{
#ifdef MECHADUINO_HARDWARE
	return;
#else
	setMotorPWM(percent);
	
	if(pinState & 0x01)
	{
		pinPeripheral(PIN_A4954_IN2, PIO_TIMER_ALT);	//TCC0 WO[7]
	}
	if (pinState & 0x02)
	{
		pinPeripheral(PIN_A4954_IN1, PIO_TIMER);		//TCC0 WO[1]
	}
	if (pinState & 0x04)
	{
		pinPeripheral(PIN_A4954_IN4, PIO_TIMER_ALT);
	}
	if (pinState & 0x08)
	{
		pinPeripheral(PIN_A4954_IN3, PIO_TIMER_ALT);
	}
#endif
}

//This function assumes pin is setup correctly and only mux change is needed
//
void setPinMux(uint32_t ulPin, EPioType ulPeripheral)
{
	uint32_t temp;
		
	if (g_APinDescription[ulPin].ulPin & 1) // is pin odd?
		{
			temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE(0xF);			//Read existing mux setup
			PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXO(ulPeripheral);	//Set new muxing
		}
	else // even pin
		{
			temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO(0xF);			//Read existing mux setup
			PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp | PORT_PMUX_PMUXE(ulPeripheral);	//Set new muxing
		}
	
	PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;   // Enable port mux, high drive strength
	//PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].bit.PMUXEN = 1;  		//Enable port mux
}


//PWM output to create voltage on Vref pins, no DAC used
//one DAC on samd21 10-bit output on (PA02)
//two DAC on samd51 - PA02, PA05
void A4954::setDAC(uint32_t DAC1, uint32_t DAC2)
{
	TCC1->CC[1].reg = (uint32_t)DAC1; //D9 PA07 - VREF12
	syncTCC(TCC1);
	TCC1->CC[0].reg = (uint32_t)DAC2; //D4 PA08 - VREF34
	syncTCC(TCC1);
}

//Setup PWM outputs
//TCC1 up to 96MHz?
void A4954::setupDAC(void)
{
	Tcc* TCCx = TCC1;

	pinPeripheral(PIN_A4954_VREF34, PIO_TIMER_ALT);		//
	pinPeripheral(PIN_A4954_VREF12, PIO_TIMER);			//

	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));
	while (GCLK->STATUS.bit.SYNCBUSY == 1);

	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;		//Disable timer TCx
	syncTCC(TCCx);

	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;	// Set TCx in waveform mode Normal PWM
	syncTCC(TCCx);

	TCCx->CC[1].reg = (uint32_t)0;		//Set initial Vref = 0V
	syncTCC(TCCx);

	TCCx->CC[0].reg = (uint32_t)0;		//Set initial Vref = 0V
	syncTCC(TCCx);

	TCCx->PER.reg = DAC_MAX;			// Set PER to maximum counter value		//48MHz / 0x1FF = 90kHz
	syncTCC(TCCx);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE;
	syncTCC(TCCx);
}


//
void A4954::enable(bool enable)
{
	enabled = enable;
	if (enabled == false)
	{
		WARNING("A4954 disabled");
		setDAC(0, 0);		//turn current off
		bridge1(drvStates::HiZ);			//tri state bridge outputs
		bridge2(drvStates::HiZ);			//tri state bridge outputs
	}
}


// This is precise move and modulo of A4954_NUM_MICROSTEPS is a full step.
// stepAngle is in A4954_NUM_MICROSTEPS units.
// The A4954 has no idea where the motor is, so the calling function has to tell the A4954 what phase to drive motor coils.
// A4954_NUM_MICROSTEPS is 256 by default so stepAngle of 1024 is 360 degrees
// Note you can only move up to +/-A4954_NUM_MICROSTEPS from where you currently are.
int32_t A4954::move(int32_t stepAngle, uint32_t mA)
{
	uint16_t angle;
	int32_t cos, sin;
	int32_t dacSin, dacCos;

	if (enabled == false)
	{
		//WARNING("A4954 disabled");
		setDAC(0, 0); //turn current off
		bridge1(drvStates::HiZ);  //tri state bridge outputs
		bridge2(drvStates::HiZ);  //tri state bridge outputs
		return stepAngle;
	}

	//WARNING("move %d %d",stepAngle,mA);

	//handle roll overs
	//stepAngle = stepAngle % SINE_STEPS;
	stepAngle = abs(stepAngle % SINE_STEPS);
	
	angle = stepAngle;
	
	if (angle > SINE_TABLE_SIZE)
		ERROR("angle (%d) exceeds table", angle);

	//calculate the sine and cosine of our angle
	sin = sine(angle);
	cos = cosine(angle);

	//if we are reverse swap the sign of one of the angels
	if (false == forwardRotation)
	{
		cos = -cos;
	}

	//scale sine result by current(mA)
	dacSin = ((int32_t)mA * (int64_t)abs(sin)) / SINE_MAX;

	//scale cosine result by current(mA)
	dacCos = ((int32_t)mA * (int64_t)abs(cos)) / SINE_MAX;

	//	if (i==0)
	//	{
	//		WARNING("dacs are %d %d",dacSin,dacCos);
	//	}

	//convert value into DAC scaled to 3300mA max
	dacCos = (int32_t)((int64_t)dacCos*(DAC_MAX)) / 3300;

	//convert value into DAC scaled to 3300mA max
	dacSin = (int32_t)((int64_t)dacSin*(DAC_MAX)) / 3300;

	//WARNING("dacs are %d %d ",dacSin,dacCos);

	setDAC(dacSin, dacCos);

	if (sin > 0)
	{
		bridge1(drvStates::Reverse);
	}
	else
	{
		bridge1(drvStates::Forward);
	}
	
	if (cos > 0)
	{
		bridge2(drvStates::Reverse);
	}
	else
	{
		bridge2(drvStates::Forward);
	}

	lastStepMicros = micros();
	return stepAngle;
}


#pragma GCC pop_options