#include "steppin.h"
#include "stepper_controller.h"
#include "wiring_private.h"
#include "Arduino.h"

extern StepperCtrl stepperCtrl;

volatile int32_t stepsChanged = 0;
volatile int64_t steps = 0;

#ifdef __SAMD51__
	//nothing
#else	//SAMD21
	#if (PIN_STEP_INPUT != 0)
	#error "this code only works with step pin being D0 (PA11, EXTINT11)"
	#endif
#endif

#define WAIT_TCC2_SYNC() while(TCC2->SYNCBUSY.reg)


//Check if direction setting changed in config
void checkDir()
{
	static RotationDir lastDir = RotationDir::Unknown;
	RotationDir dir = NVM->SystemParams.dirPinRotation;	
	
	if (lastDir != dir)
	{
		if (dir == RotationDir::CW)		//CW
		{
			EIC->CONFIG[1].bit.SENSE2 = EIC_CONFIG_SENSE2_HIGH_Val;		//pin high detection
			//EIC->CONFIG[1].reg &= ~EIC_CONFIG_SENSE2_Msk;
			//EIC->CONFIG[1].reg |=  EIC_CONFIG_SENSE2_HIGH;
		} 
		else							//CCW
		{
			EIC->CONFIG[1].bit.SENSE2 = EIC_CONFIG_SENSE2_LOW_Val;		//pin low detection
			//EIC->CONFIG[1].reg &= ~EIC_CONFIG_SENSE2_Msk;
			//EIC->CONFIG[1].reg |=  EIC_CONFIG_SENSE2_LOW;
		}
	
		lastDir = dir;
	}
}

//this function cannot be called in interrupt context as the overflow interrupt for tC4 needs to run.
int32_t getSteps(void)
{
#ifdef USE_TC_STEP
	uint16_t y;
	static uint16_t lasty = 0;

	TCC2->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;		//synchronize timer register
	WAIT_TCC2_SYNC();

	y = (uint16_t)(TCC2->COUNT.reg & 0x0FFFFul);  //use only lowest 16bits
	
	steps += (int16_t)(y - lasty);
	lasty = y;
	checkDir();			//check if direction setting was changed
	return steps;
#else
	int64_t x;
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

//
void enableEIC(void)
{
#ifdef _SAMD21_
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
#else	//SAMD51
	MCLK->APBAMASK.bit.EIC_ = true;  		//	
	
	GCLK->PCHCTRL[GCLK_EIC].bit.CHEN = true;     						//Enable peripheral channel
	GCLK->PCHCTRL[GCLK_EIC].bit.GEN = GCLK_PCHCTRL_GEN_GCLK10_Val;     	//Set GCLK channel to x
	
	//What was 10? need to check...
	
	//EIC->CONFIG[0].bit.FILTEN0
	//EIC->CTRLA.bit.ENABLE
	//**FFF create define for EIC channel*/
	
#endif
		

}


//
void setupStepEvent(void)
{
	//we will set up the EIC to generate an event on rising edge of step pin
	enableEIC();					//make sure EIC is setup

	// Assign step pin to EIC
	pinPeripheral(PIN_STEP_INPUT, PIO_EXTINT);		// Step pin is EXTINT11
	pinPeripheral(PIN_DIR_INPUT, PIO_EXTINT);		//
	
	
#ifdef _SAMD21_
	//***** setup EIC ******
	EIC->EVCTRL.bit.EXTINTEO11 = 1;  //enable event for EXTINT11
	EIC->EVCTRL.bit.EXTINTEO10 = 1;   //enable event for EXTINT10
	
	//setup up external interrupt 11 to be rising edge triggered
	//high traigger
	EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE3_RISE | EIC_CONFIG_SENSE2_HIGH;
	
	checkDir();
	
	//disable actually generating an interrupt, we only want event triggered
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT11;
	EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT10;
		
	//**** setup the event system ***
	// Enable GCLK for EVSYS channel 0
	PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EVSYS_CHANNEL_0));
	while (GCLK->STATUS.bit.SYNCBUSY) ;
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
	PM->APBCMASK.reg |= PM_APBCMASK_TCC2; 		//Enable TCC2 clock
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));  // Enable GCLK for TCC2 (timer counter input clock)
	while(GCLK->STATUS.bit.SYNCBUSY);
	
#else	//SAMD51
	EIC->EVCTRL.bit.EXTINTEO = 1 << 11;			//Enable event for EXTINT 11
	EIC->EVCTRL.bit.EXTINTEO = 1 << 10; 		//Enable event for EXTINT 10
	//check if 1<<10 is correct location?
	
	EIC->CONFIG[1].bit.SENSE3 = EIC_CONFIG_SENSE3_RISE_Val;		//Rising edge trigger
	EIC->CONFIG[1].bit.SENSE2 = EIC_CONFIG_SENSE2_HIGH_Val; 	//High trigger
	
	checkDir();	

	//disable actually generating an interrupt, we only want event triggered
	EIC->INTENCLR.bit.EXTINT = 1 << 11; 			//1 = disable interrupt
	EIC->INTENCLR.bit.EXTINT = 1 << 10; 			//1 = disable interrupt

	//APBA, APBB, APBC, APBD max clock = 120MHz
	MCLK->APBBMASK.bit.EVSYS_ = true;		//Enable clock for EVSYS (APBB)
	
	//Using 7 again?
	GCLK->GENCTRL[7].bit.DIV = 1; 						//Divider = 1
	GCLK->GENCTRL[7].bit.IDC = true; 					//Improve duty cycle (?)
	GCLK->GENCTRL[7].bit.GENEN = true;  				//Enable
	GCLK->GENCTRL[7].bit.SRC = GCLK_GENCTRL_SRC_DFLL;   //Select 48MHz source (?) //100/120MHz available
	while(GCLK->SYNCBUSY.bit.GENCTRL7); 				//Wait for synchronization
	
	//
	GCLK->PCHCTRL[GCLK_EVSYS10].bit.CHEN = true;    						//Enable peripheral channel (EVSYS_x)
	GCLK->PCHCTRL[GCLK_EVSYS10].bit.GEN = GCLK_PCHCTRL_GEN_GCLK7_Val;		//Set GCLK channel to 7
	
	GCLK->PCHCTRL[GCLK_EVSYS11].bit.CHEN = true;     						//Enable peripheral channel (EVSYS_x)
	GCLK->PCHCTRL[GCLK_EVSYS11].bit.GEN = GCLK_PCHCTRL_GEN_GCLK7_Val;		//Set GCLK channel to 7
	
	
	//Setup step pin to trigger on event 0 on TCC2 (step)
	EVSYS->Channel[0].CHANNEL.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_RISING_EDGE_Val;				//Rising edge
	EVSYS->Channel[0].CHANNEL.bit.EVGEN = EVSYS_ID_GEN_EIC_EXTINT_11;							//?? is this right format
	EVSYS->Channel[0].CHANNEL.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val;					//Asynchronous path
	
	EVSYS->USER[1].bit.CHANNEL = EVSYS_ID_USER_TCC2_EV_0;										//Select channel to connect to event user (TCC2 EV0)
	//value x of this bit field selects channel n = x-1 ???
	
	
	//Setup the dir pin to trigger on event 2 on TCC2 (direction change)
	EVSYS->Channel[1].CHANNEL.bit.EDGSEL = EVSYS_CHANNEL_EDGSEL_BOTH_EDGES_Val; 				//
	EVSYS->Channel[1].CHANNEL.bit.EVGEN = EVSYS_ID_GEN_EIC_EXTINT_10; 							//?? is this right format
	EVSYS->Channel[1].CHANNEL.bit.PATH = EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val; 					//Asynchronous path
	
	EVSYS->USER[2].bit.CHANNEL = EVSYS_ID_USER_TCC2_EV_1; 										//Select channel to connect to event user (TCC2 EV1)	
	
	//Setup the timer counter
	MCLK->APBCMASK.bit.TCC2_ = 1;					//Enable clock TCC2
	
	GCLK->PCHCTRL[GCLK_TCC2].bit.CHEN = true;    							//Enable peripheral channel (TCC2)
	GCLK->PCHCTRL[GCLK_TCC2].bit.GEN = GCLK_PCHCTRL_GEN_GCLK7_Val;  			//Set GCLK channel to 7	
#endif

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
	
	TCC2->CTRLBSET.bit.DIR = 1;		//counter direction -> down
	WAIT_TCC2_SYNC();
	
	TCC2->CTRLA.reg |= TCC_CTRLA_ENABLE;	//Enable TCC2 peripheral
	WAIT_TCC2_SYNC();
}

/* not used?
static void dirChanged_ISR(void)
{
	int dir = 0;
	//read our direction pin
	//dir = digitalRead(PIN_DIR_INPUT);
	if ((PORT->Group[g_APinDescription[PIN_DIR_INPUT].ulPort].IN.reg & (1ul << g_APinDescription[PIN_DIR_INPUT].ulPin)) != 0)
	{
		dir = 1;
	}

	if (RotationDir::CW == NVM->SystemParams.dirPinRotation)
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
