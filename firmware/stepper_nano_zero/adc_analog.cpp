#include "adc_analog.h"
#include "printf.h"
#include "board.h"
#include "variant.h"
#include <Arduino.h>

//port [0] = PA, 1 = PB

//x.reg = REG_CONST
//x.bit = REG_CONST_VAL	

//ADC conversion range: 
//- vref 1v to vddana - 0.6V
//ADCx * GAIN [0V to -Vref]
//Vref 1/2 or 1/1.48


#define ADC_MAX_F 4095.0		//12-bit ADC
#define V_SUPPLY  3.33			//3.33V supply
#define ADC_GAIN  0.50			//gain is half


void ADC_Peripheral::begin()
{
#if defined(__SAMD51__)
	#define NUM_ADCS 2

	//Setup ADC0 and ADC1
	for(int i = 0 ; i < NUM_ADCS ; i++)
	{
		Adc *adc;
		if (i == 0) adc = ADC0;
		else if (i==1) adc = ADC1;
		else return;

		//Can use VREF SEL to select 1V to 2.5V values
		adc->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; 		//Ref = VddANA
	
		adc->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXNEG_AIN0_Val; 		 // Selection for the positive ADC input
		adc->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val; 	     // Selection for the negative ADC input
		//adc->CTRLB.bit.CORREN = 1;										//gain and offset correction enabled (slower)
		//adc->CTRLA.bit.R2R =	//only set R2R in differential mode
  
		adc->CTRLA.bit.ENABLE = 0x01;									// Enable ADC
		adc->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV64_Val;		//Divide clock by 64 = 750kHz
	
		adc->CTRLB.bit.RESSEL =	ADC_CTRLB_RESSEL_12BIT_Val; 			//12-bit resolution (8-12 avail)
	
		adc->AVGCTRL.reg =	ADC_AVGCTRL_SAMPLENUM_1	| 		//Average x samples
							ADC_AVGCTRL_ADJRES(0x00); 		//
	
		adc->SAMPCTRL.reg = 0x02; 							//Sample time
		syncADC(adc);					//test
		
		adc->SWTRIG.bit.START = 1;
	
		//Wait for conversion to complete
		while(adc->INTFLAG.bit.RESRDY == 0) {}
		
	/*
	//Does ADC clock need to be setup????
	MCLK->APBDMASK.bit.ADC0_ = true;		//
	MCLK->APBDMASK.bit.ADC1_ = true; 		//
		
	//9
	GCLK->GENCTRL[9].bit.DIV = 1; 				//Divider = 1
	GCLK->GENCTRL[9].bit.IDC = true; 			//Improve duty cycle (?)
	GCLK->GENCTRL[9].bit.GENEN = true;  		//Enable
	GCLK->GENCTRL[9].bit.SRC = GCLK_GENCTRL_SRC_DFLL;   	//Select 48MHz source (?)
	while(GCLK->SYNCBUSY.bit.GENCTRL9); 			//Wait for synchronization
	
	GCLK->PCHCTRL[40].bit.CHEN = true;  						//Enable peripheral channel (40 = ADC0)
	GCLK->PCHCTRL[40].bit.GEN = GCLK_PCHCTRL_GEN_GCLK9_Val;  	//Set GCLK channel to x

	GCLK->PCHCTRL[41].bit.CHEN = true;   						//Enable peripheral channel (41 = ADC1)
	GCLK->PCHCTRL[41].bit.GEN = GCLK_PCHCTRL_GEN_GCLK9_Val;   	//Set GCLK channel to x
	*/
	}
#else
	syncADC();
	ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;		//Ref = 1/2 Vdda
	
	ADC->INPUTCTRL.bit.MUXPOS = ADC_INPUTCTRL_MUXPOS_PIN0_Val;		 // Selection for the positive ADC input
	ADC->INPUTCTRL.bit.MUXNEG = ADC_INPUTCTRL_MUXNEG_GND_Val;	     // Selection for the negative ADC input
	ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;			 // Set gain to 0.5x
  
	//ADC->CTRLB.bit.CORREN = 1;										//gain and offset correction enabled (slower)
	ADC->CTRLB.bit.DIFFMODE = 0;									//Single ended mode (muxneg ignored)
	
	// Control A
	/*
	 * Bit 1 ENABLE: Enable
	 *   0: The ADC is disabled.
	 *   1: The ADC is enabled.
	 * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
	 * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
	 * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
	 *
	 * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
	 * configured. The first conversion after the reference is changed must not be used.
	 */
	syncADC();
	ADC->CTRLA.bit.ENABLE = 0x01;                 // Enable ADC
	
	ADC->CTRLB.reg =	ADC_CTRLB_PRESCALER_DIV64 | 	//Divide clock by 64 = 750kHz
						ADC_CTRLB_RESSEL_12BIT;			//12-bit resolution (8-12 avail)
	
	ADC->AVGCTRL.reg =	ADC_AVGCTRL_SAMPLENUM_1	|		//Average x samples
						ADC_AVGCTRL_ADJRES(0x00);		//
	
	ADC->SAMPCTRL.reg = 0x02;							//Sample time
	
	syncADC();				//Wait for sync
	
	// Start conversion
	ADC->SWTRIG.bit.START = 1;

	// Clear the Data Ready flag
	//ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;
	
	//Wait for conversion to complete
	while(ADC->INTFLAG.bit.RESRDY == 0) {}
#endif

}

#ifdef _SAMD51_
uint32_t ADC_Peripheral::read_blocking(adc0PortMap pin)
{
	return 0;
}

uint32_t ADC_Peripheral::read_blocking(adc1PortMap pin)
	
#else
//Returns x-bit ADC value from selected ADC pin
uint32_t ADC_Peripheral::read_blocking(adcPortMap pin)		//need to pass somehow..
#endif // 
{
	uint32_t valueRead = 0;

#if defined(__SAMD51__)
	Adc *adc;
	adc = ADC0;
	//if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
	//else if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
	//else return 0;

	while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL) ; //wait for sync
	//adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;   // Selection for the positive ADC input
  
	// Control A
	/*
	 * Bit 1 ENABLE: Enable
	 *   0: The ADC is disabled.
	 *   1: The ADC is enabled.
	 * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
	 * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
	 * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
	 *
	 * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
	 * configured. The first conversion after the reference is changed must not be used.
	 */
	while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);   //wait for sync
	adc->CTRLA.bit.ENABLE = 0x01;               // Enable ADC

	// Start conversion
	while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);   //wait for sync
	
	adc->SWTRIG.bit.START = 1;				//Start conversion
	while(adc->INTFLAG.bit.RESRDY == 0);    //Waiting for conversion to complete
	valueRead = adc->RESULT.reg;			//Store result

#else
	//adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;    // Selection for the positive ADC input
	
	ADC->INPUTCTRL.bit.MUXPOS = static_cast<uint32_t>(pin);		//select ADC channel (AIN0 -> AINxx)
	ADC->INTFLAG.bit.RESRDY = 1;				//Clear flag
	syncADC();
	
	// Start conversion
	ADC->SWTRIG.bit.START = 1;					//
	while(ADC->INTFLAG.bit.RESRDY == 0);		// Waiting for conversion to complete
	syncADC();
	valueRead = ADC->RESULT.reg;				// Store the value
	
	ADC->SWTRIG.reg = 0x01;		//test
#endif

	return valueRead;
}

#if defined(NZ_STEPPER_REV1) || defined(NZ_STEPPER_REV2) || defined(NZ_STEPPER_REV3) || defined(NZ_STEPPER_NEMA23) || defined(NEMA17_SMART_STEPPER_3_21_2017)
float ADC_Peripheral::getMotorVoltage()
{
	uint32_t x;
	float f;

#ifdef _SAMD21_
	x = read_blocking(adcPortMap::_PA04);						//Vmotor tap
#else	//SAMD51
	
#endif	
	f = (float)x * V_SUPPLY / ADC_MAX_F * ((10.0 + 1.0) / 1);  //10k/1k resistor divider
	return f;
}

//Read temperature near motor chip
//will lag compared to junction temp
//10k resistor + 10k NTC
float ADC_Peripheral::getTemperature()
{
	float ntc, vratio;
	const float invBeta = 1.00 / 3380.00; 					// Thermistor Beta
	const float invT0 = 1.00 / (273.15 + 25.0); 		    // room temp in Kelvin
	
	int adcVal = read_blocking(adcPortMap::_PB03);
	
	vratio = (float)adcVal / ADC_MAX_F;
	ntc = 1.00 / (invT0 + invBeta * (log( 1/(1/vratio - 1)  ))); 		//Log(R/R0)
	ntc -= 273.15;								//convert to Celsius
	ntc = constrain(ntc, 0, 200);				//limit reading 0 - 200 C
	
	return ntc;
}
#endif