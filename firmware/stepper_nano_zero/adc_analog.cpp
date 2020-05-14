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
	Adc *adc;
	if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
	else if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
	else return 0;

	while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL) ; //wait for sync
	adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;     // Selection for the positive ADC input
  
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
while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);     //wait for sync
adc->CTRLA.bit.ENABLE = 0x01;                 // Enable ADC

// Start conversion
while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);     //wait for sync
  
adc->SWTRIG.bit.START = 1;

	// Clear the Data Ready flag
	//adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;  
	
	//Wait for conversion to complete
	while(ADC->INTFLAG.bit.RESRDY == 0) {}
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


//Returns x-bit ADC value from selected ADC pin
uint32_t ADC_Peripheral::read_blocking(adcPortMap pin)
{
	uint32_t valueRead = 0;

#if defined(__SAMD51__)
	Adc *adc;
	if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG) adc = ADC0;
	else if (g_APinDescription[pin].ulPinAttribute & PIN_ATTR_ANALOG_ALT) adc = ADC1;
	else return 0;

	while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL) ; //wait for sync
	adc->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;   // Selection for the positive ADC input
  
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
  
adc->SWTRIG.bit.START = 1;

	// Clear the Data Ready flag
	adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;

	// Start conversion again, since The first conversion after the reference is changed must not be used.
	adc->SWTRIG.bit.START = 1;

	// Store the value
	while(adc->INTFLAG.bit.RESRDY == 0);     // Waiting for conversion to complete
	valueRead = adc->RESULT.reg;

	while (adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE) ; //wait for sync
	adc->CTRLA.bit.ENABLE = 0x00;               // Disable ADC
	while(adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE);   //wait for sync
  
#else

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

#if defined(NZ_STEPPER_REV1) || defined(NZ_STEPPER_REV2) || defined(NZ_STEPPER_5995) || defined(NEMA17_SMART_STEPPER_3_21_2017)
float ADC_Peripheral::getMotorVoltage()
{
	uint32_t x;
	float f;

	x = read_blocking(adcPortMap::_PA04);						//Vmotor tap
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