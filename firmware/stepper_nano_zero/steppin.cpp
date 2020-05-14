#include "steppin.h"
#include "stepper_controller.h"
#include "wiring_private.h"
#include "Arduino.h"

extern StepperCtrl stepperCtrl;

volatile int32_t stepsChanged = 0;
volatile int64_t steps = 0;

#if (PIN_STEP_INPUT != 0)
#error "this code only works with step pin being D0 (PA11, EXTINT11)"
#endif

#define WAIT_TCC2_SYNC() while(TCC2->SYNCBUSY.reg)

void checkDir()
{
	int dir = 1;
	static int lastDir = -1;
	
	if (RotationDir::CW == NVM->SystemParams.dirPinRotation)
	{
		dir = 0; 		//reverse the direction
	}
	
	if (lastDir != dir)
	{
		if (dir)
		{
			EIC->CONFIG[1].reg &= ~EIC_CONFIG_SENSE2_Msk;
			EIC->CONFIG[1].reg |=  EIC_CONFIG_SENSE2_HIGH;

		} 
		else
		{
			EIC->CONFIG[1].reg &= ~EIC_CONFIG_SENSE2_Msk;
			EIC->CONFIG[1].reg |=  EIC_CONFIG_SENSE2_LOW;
		}
		
		lastDir = dir;
	}
}

//this function cannot be called in interrupt context as the overflow interrupt for tC4 needs to run.
int64_t getSteps(void)
{
	int64_t x;
#ifdef USE_TC_STEP
	uint16_t y;
	static uint16_t lasty = 0;

	TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;		//synchronize timer register
	WAIT_TCC2_SYNC();

	y = (uint16_t)(TCC2->COUNT.reg & 0x0FFFFul);  //use only lowest 16bits
	
	steps += (int16_t)(y - lasty);
	lasty = y;
	checkDir();
	
	return steps;
#else
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT11;
	x = stepsChanged;
	stepsChanged = 0;
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT11;
	return x;
#endif
}

//this function is called on the rising edge of a step from external device
static void stepInput(void)
{
	static int dir;

	//read our direction pin
	dir = digitalRead(PIN_DIR_INPUT);

	if (RotationDir::CW == NVM->SystemParams.dirPinRotation)
	{
		dir = !dir; //reverse the rotation
	}

#ifndef USE_NEW_STEP
	stepperCtrl.requestStep(dir, 1);
#else
	if (dir)
	{
		stepsChanged++;
	}
	else
	{
		stepsChanged--;
	}
#endif
}

void enableEIC(void)
{
	PM->APBAMASK.reg |= PM_APBAMASK_EIC;
	if (EIC->CTRL.bit.ENABLE == 0)
	{
		// Enable GCLK for IEC (External Interrupt Controller)
		GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));

		// Enable EIC
		EIC->CTRL.bit.ENABLE = 1;
		while (EIC->STATUS.bit.SYNCBUSY == 1)
		{
		}
	}
}

//
void setupStepEvent(void)
{
	//we will set up the EIC to generate an even on rising edge of step pin
	enableEIC();					//make sure EIC is setup

	// Assign step pin to EIC
	pinPeripheral(PIN_STEP_INPUT, PIO_EXTINT);		// Step pin is EXTINT11
	pinPeripheral(PIN_DIR_INPUT, PIO_EXTINT);		//

	//***** setup EIC ******
	EIC->EVCTRL.bit.EXTINTEO11 = 1; //enable event for EXTINT11
	EIC->EVCTRL.bit.EXTINTEO10 = 1;  //enable event for EXTINT10
	
	//setup up external interurpt 11 to be rising edge triggered
	EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE3_RISE | EIC_CONFIG_SENSE2_HIGH;
	
	checkDir();

	//disable actually generating an interrupt, we only want event triggered
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT11;
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT10;

	//**** setup the event system ***
	// Enable GCLK for EVSYS channel 0
	PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EVSYS_CHANNEL_0));
	while (GCLK->STATUS.bit.SYNCBUSY);
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EVSYS_CHANNEL_1));
	while (GCLK->STATUS.bit.SYNCBUSY) ;	

	//Setup the step pin to trigger on event 0 on TCC2 (step)
	EVSYS->CHANNEL.reg =	EVSYS_CHANNEL_CHANNEL(0) | 
							EVSYS_CHANNEL_EDGSEL_RISING_EDGE | 
							EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_11) | 
							EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

	EVSYS->USER.reg =		EVSYS_USER_CHANNEL(1) | 
							EVSYS_USER_USER(EVSYS_ID_USER_TCC2_EV_0);
	
	//Setup the dir pin to trigger on event 2 on TCC2 (direction change)
	EVSYS->CHANNEL.reg =	EVSYS_CHANNEL_CHANNEL(1) | 
							EVSYS_CHANNEL_EDGSEL_BOTH_EDGES | 
							EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_10) | 
							EVSYS_CHANNEL_PATH_ASYNCHRONOUS;

	EVSYS->USER.reg =		EVSYS_USER_CHANNEL(2) | 
							EVSYS_USER_USER(EVSYS_ID_USER_TCC2_EV_1);
	
	
	//Setup the timer counter
	PM->APBCMASK.reg |= PM_APBCMASK_TCC2;		//Enable TCC2 clock
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3)); // Enable GCLK for TCC2 (timer counter input clock)
	while (GCLK->STATUS.bit.SYNCBUSY);

	TCC2->CTRLA.reg &= ~TCC_CTRLA_ENABLE;	//Disable TCC2
	WAIT_TCC2_SYNC();

	TCC2->CTRLA.reg = TCC_CTRLA_SWRST;		//Reset TCC2
	WAIT_TCC2_SYNC();						//
	while (TCC2->CTRLA.bit.SWRST == 1) ;	//wait for reset to complete

	TCC2->EVCTRL.reg =	TCC_EVCTRL_EVACT0_COUNTEV |		//event0: count
						TCC_EVCTRL_TCEI0	|			//enable incoming event 0
						TCC_EVCTRL_EVACT1_DIR |			//event1: direction control?
						TCC_EVCTRL_TCEI1;				//enable incoming event 1
	WAIT_TCC2_SYNC();
	
	TCC2->COUNT.reg = 0;			//reset count to 0
	WAIT_TCC2_SYNC();
	
	TCC2->CTRLBSET.bit.DIR = 1;		//counter direction -> down		**FF??
	WAIT_TCC2_SYNC();
	
	TCC2->CTRLA.reg |= TCC_CTRLA_ENABLE;	//Enable TCC2 peripheral
	WAIT_TCC2_SYNC();
}

/*
static void dirChanged_ISR(void)
{
	int dir = 0;
	//read our direction pin
	//dir = digitalRead(PIN_DIR_INPUT);
	if ((PORT->Group[g_APinDescription[PIN_DIR_INPUT].ulPort].IN.reg & (1ul << g_APinDescription[PIN_DIR_INPUT].ulPin)) != 0)
	{
		dir = 1;
	}

	if (RotationDir_t::CW_ROTATION == NVM->SystemParams.dirPinRotation)
	{
		dir = !dir; //reverse the rotation
	}

	if (dir)
	{
		TC4->COUNT16.CTRLBSET.bit.DIR = 1;
	}
	else
	{
		TC4->COUNT16.CTRLBCLR.bit.DIR = 1;
	}
}*/

void stepPinSetup(void)
{

#ifdef USE_TC_STEP
	setupStepEvent();
#else
	attachInterrupt(digitalPinToInterrupt(PIN_STEP_INPUT), stepInput, RISING);
	NVIC_SetPriority(EIC_IRQn, 0); //set port A interrupt as highest priority
#endif
}
