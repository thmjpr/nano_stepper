
#include "board.h"
#include "Watchdog.h"


//If we reach here system has crashed, should safe shutdown motor pins etc.
//
void WDT_Handler()
{
	//might need to disable pinmux thing?
	GPIO_OUTPUT(PIN_A4954_IN1);
	
	GPIO_LOW(PIN_A4954_IN1);
	GPIO_LOW(PIN_A4954_IN2);
	GPIO_LOW(PIN_A4954_IN3);
	GPIO_LOW(PIN_A4954_IN4);
	
	GPIO_LOW(PIN_A4954_VREF12);
	GPIO_LOW(PIN_A4954_VREF34);
	
	//ERROR("Watchdog triggered");
	
	while (1)
	{
		//Wait for system to reset
	}
}

void Watchdog::clear()
{
	WDT->CLEAR.reg = 0xA5;
#ifdef _SAMD21_	
	while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY) ;
#else	//SAMD51
	while (WDT->SYNCBUSY.reg) {}					//synchronizing in progress
#endif // _SAMD21_
}

void Watchdog::setup()
{
#ifdef _SAMD21_
	// Generic clock generator 2, divisor = 32 (2^(DIV+1))
	GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
	
	// Enable clock generator 2 using low-power 32KHz oscillator.
	// With /32 divisor above, this yields 1024Hz(ish) clock.
	GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) |
	                    GCLK_GENCTRL_GENEN |
	                    GCLK_GENCTRL_SRC_OSCULP32K |
	                    GCLK_GENCTRL_DIVSEL;
	while (GCLK->STATUS.bit.SYNCBUSY) ;
	
	// WDT clock = clock gen 2
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | 			//
	                    GCLK_CLKCTRL_CLKEN | 			//
	                    GCLK_CLKCTRL_GEN_GCLK2; 		//
	
	WDT->CONFIG.bit.PER = WDT_CONFIG_PER_16K_Val; 				//Number of gclk_wdt cycles before timeout
	WDT->EWCTRL.bit.EWOFFSET = WDT_EWCTRL_EWOFFSET_8K_Val;  	//Early warning timeout occurs first
	
	WDT->INTENSET.bit.EW = true; 				//Enable early warning interrupt
	NVIC_ClearPendingIRQ(WDT_IRQn);  			//Clear any pending
	NVIC_SetPriority(WDT_IRQn, 0);  			//Set priority of interrupt to highest priority
	NVIC_EnableIRQ(WDT_IRQn);   				//Enable InterruptVector
	
	WDT->CTRL.bit.WEN = false; 			//Normal mode, no window time
	WDT->CTRL.bit.ENABLE = true; 		//Enable WDT
	while(GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);		
#else	//SAMD51
	//WDT uses internal 1.024kHz OSCULP32K clock, running all the time
	
	//needed??
	//MCLK->APBAMASK.bit.WDT_ = true;
	
	WDT->CONFIG.bit.PER = WDT_CONFIG_PER_CYC4096_Val;		//Number of clock cycles before timeout = ~3s
	WDT->EWCTRL.bit.EWOFFSET = WDT_EWCTRL_EWOFFSET_CYC2048_Val;	//Early warning timeout occurs first
	WDT->INTENSET.bit.EW = true; 				//Enable early warning interrupt
	NVIC_ClearPendingIRQ(WDT_IRQn);  			//Clear any pending
	NVIC_SetPriority(WDT_IRQn, 0);  			//Set priority of interrupt to highest priority
	NVIC_EnableIRQ(WDT_IRQn);   				//Enable InterruptVector
	
	WDT->CTRLA.bit.WEN = false;					//Normal mode, no window time
	WDT->CTRLA.bit.ENABLE = true; 				//Enable WDT
	
	//CLK_WDT_OSC is asynchronous and writing certain registers will require clk domain sync
	while(WDT->SYNCBUSY.reg) {}					//synchronizing in progress
#endif
			
}