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
#include "planner.h"
#include "board.h"
#include "wiring_private.h"
#include "syslog.h"
#include "angle.h"
#include "Arduino.h"

//define the planner class as being global
Planner SmartPlanner;

static bool enterTC3CriticalSection()
{	
	bool state = NVIC_IS_IRQ_ENABLED(TC3_IRQn);
	NVIC_DisableIRQ(TC3_IRQn);
	return state;
}

static void exitTC3CriticalSection(bool prevState)
{
	if (prevState)
	{
		NVIC_EnableIRQ(TC3_IRQn);
	} //else do nothing
}

void TC3_Init(void)
{
	Tc* TCx = TC3;
	
#ifdef _SAMD21_
	// Enable GCLK for TC3
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
	while (GCLK->STATUS.bit.SYNCBUSY) ;
#else	//SAMD51
	MCLK->APBBMASK.bit.TC3_ = true; 			//Enable TC3 clock
	//Unsure if genctrl 8 is used elsewhere, assume they all have to be unique? goes up to [11]
	GCLK->GENCTRL[8].bit.DIV = 1; 				//Divider = 1
	GCLK->GENCTRL[8].bit.IDC = true; 			//Improve duty cycle (?)
	GCLK->GENCTRL[8].bit.GENEN = true;  		//Enable
	GCLK->GENCTRL[8].bit.SRC = GCLK_GENCTRL_SRC_DFLL;   	//Select 48MHz source (?) //100/120MHz available
	while(GCLK->SYNCBUSY.bit.GENCTRL8); 		//Wait for synchronization
	
	GCLK->PCHCTRL[26].bit.CHEN = true;							//Enable peripheral channel (26 = TC3)
	GCLK->PCHCTRL[26].bit.GEN = GCLK_PCHCTRL_GEN_GCLK8_Val;		//Set GCLK channel to 8
		
	//**FF may not work without a single 32-bit write?*/
#endif // _SAMD21_
	
	TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;      // Disable TCx
	WAIT_TC16_REGS_SYNC(TC3)                      // wait for sync

	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;       // Set Timer counter Mode to 16 bits
	WAIT_TC16_REGS_SYNC(TC3)
		
#ifdef _SAMD21_
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;    // Set TC as normal Normal Frq
#else
	TCx->COUNT16.WAVE.bit.WAVEGEN = TC_WAVE_WAVEGEN_MFRQ_Val;
#endif
	WAIT_TC16_REGS_SYNC(TC3)

	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2;       // Set prescaler
	WAIT_TC16_REGS_SYNC(TC3)

	TCx->COUNT16.CC[0].reg = F_CPU / PLANNER_UPDATE_RATE_HZ / 2;     //divide by two because of prescaler

	WAIT_TC16_REGS_SYNC(TC3)	

	TCx->COUNT16.INTENSET.reg = 0;                // disable all interrupts
	TCx->COUNT16.INTENSET.bit.OVF = 1;            // enable overflow

	//Interrupts
	NVIC_ClearPendingIRQ(TC3_IRQn);		//Clear
	NVIC_SetPriority(TC3_IRQn, 3);		//Set priority low
	NVIC_EnableIRQ(TC3_IRQn);			//Enable InterruptVector

	// Enable TC
	TCx->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	WAIT_TC16_REGS_SYNC(TCx);
}


void TC3_Handler(void)
{
	TC3->COUNT16.INTFLAG.bit.OVF = 1;   //clear overflow flag
	interrupts();						//allow other interrupts
	SmartPlanner.tick();				//do the planner tick
}

void Planner::begin(StepperCtrl *ptrStepper)
{
	ptrStepperCtrl = ptrStepper;
	currentMode = PlannerMode::NONE;
	//setup the timer and interrupt as the last thing
	TC3_Init();
}

void Planner::tick(void)
{
	if (currentMode == PlannerMode::NONE)
	{
		return; //do nothing
	}

	if (currentMode == PlannerMode::CV)
	{
		//		SerialUSB.println(currentSetAngle);
		//		SerialUSB.println(endAngle);
		//		SerialUSB.println(tickIncrement);
		//		SerialUSB.println(fabs(currentSetAngle-endAngle));
		//		SerialUSB.println(fabs(tickIncrement*2));
		//		SerialUSB.println();
				int32_t x;
		if (fabs(currentSetAngle - endAngle) >= fabs(tickIncrement))
		{
			currentSetAngle += tickIncrement;
			x = ANGLE_FROM_DEGREES(currentSetAngle);
			ptrStepperCtrl->moveToAbsAngle(x);
		}
		else
		{
			//we are done, make sure we end at the right point
			//SerialUSB.println("done");
			x = ANGLE_FROM_DEGREES(endAngle);
			ptrStepperCtrl->moveToAbsAngle(x);
			currentMode = PlannerMode::NONE;
		}
	}
}

void Planner::stop(void)
{
	bool state;
	state = enterTC3CriticalSection();
	currentMode = PlannerMode::NONE;
	exitTC3CriticalSection(state);
}

bool Planner::moveConstantVelocity(float finalAngle, float rpm)
{
	bool state;
	state = enterTC3CriticalSection();

	//first determine if operation is in progress
	if(PlannerMode::NONE != currentMode)
	{
		//we are in operation return false
		SerialUSB.println("planner operational");
		exitTC3CriticalSection(state);
		return false;
	}

	//get current posistion
	startAngle = ANGLE_T0_DEGREES(ptrStepperCtrl->getCurrentAngle());

	//deterime the tick increment
	tickIncrement = 360.0*fabs(rpm) / 60 / PLANNER_UPDATE_RATE_HZ;

	//set the desired end angle
	endAngle = finalAngle;

	//set the current angle
	currentSetAngle = startAngle;

	if (startAngle > endAngle)
	{
		SerialUSB.println("reverse");
		tickIncrement = -tickIncrement;
	}

	//	SerialUSB.println(currentSetAngle);
	//		SerialUSB.println(endAngle);
	//		SerialUSB.println(tickIncrement);
	//		SerialUSB.println();

	currentMode = PlannerMode::CV;

	exitTC3CriticalSection(state);
	return true;
}
