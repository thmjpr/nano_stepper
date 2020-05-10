#pragma once
#include <stdio.h>
#include <Arduino.h>

#if !defined(__SAMD51__)
// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
	while (ADC->STATUS.bit.SYNCBUSY == 1) ;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
	while (TCx->COUNT16.STATUS.bit.SYNCBUSY) ;
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
	while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK) ;
}

#else
static bool dacEnabled[2];
#endif

//auto casting, -std-c++14 required
#include <type_traits>
template <typename E>
	constexpr auto to_underlying(E e) noexcept
{
	return static_cast<std::underlying_type_t<E>>(e);
}


//samd21 only? need to check
#if defined(__SAMD51__)

#else
//Map Analog in to register number
enum class adcMuxPins
{
	AIN0 = ADC_INPUTCTRL_MUXPOS_PIN0_Val,	//use _Val
	AIN1,	AIN2,	AIN3,	AIN4,	AIN5,
	AIN6,	AIN7,	AIN8,	AIN9,	AIN10,
	AIN11,	AIN12,	AIN13,	AIN14,	AIN15,
	AIN16,	AIN17,	AIN18,	AIN19,
	
	Temp = ADC_INPUTCTRL_MUXPOS_TEMP,
	Bandgap,
	CoreVcc,		//1/4 scaled
	IOVcc,			//1/4 scaled
	DACvout,		//
};

//Map Analog in to port pin
enum class adcPortMap
{
	_PA02 = static_cast<uint32_t>(adcMuxPins::AIN0),
	_PA03 = static_cast<uint32_t>(adcMuxPins::AIN1),
	_PB08 = static_cast<uint32_t>(adcMuxPins::AIN2),
	_PB09 = static_cast<uint32_t>(adcMuxPins::AIN3),
	_PA04 = static_cast<uint32_t>(adcMuxPins::AIN4),
	_PA05 = static_cast<uint32_t>(adcMuxPins::AIN5),
	_PA06 = static_cast<uint32_t>(adcMuxPins::AIN6),
	_PA07 = static_cast<uint32_t>(adcMuxPins::AIN7),
	_PB00 = static_cast<uint32_t>(adcMuxPins::AIN8),
	_PB01 = static_cast<uint32_t>(adcMuxPins::AIN9),
	_PB02 = static_cast<uint32_t>(adcMuxPins::AIN10),
	_PB03 = static_cast<uint32_t>(adcMuxPins::AIN11),
	_PB04 = static_cast<uint32_t>(adcMuxPins::AIN12),
	_PB05 = static_cast<uint32_t>(adcMuxPins::AIN13),
	_PB06 = static_cast<uint32_t>(adcMuxPins::AIN14),
	_PB07 = static_cast<uint32_t>(adcMuxPins::AIN15),
	_PA08 = static_cast<uint32_t>(adcMuxPins::AIN16),
	_PA09 = static_cast<uint32_t>(adcMuxPins::AIN17),
	_PA10 = static_cast<uint32_t>(adcMuxPins::AIN18),
	_PA11 = static_cast<uint32_t>(adcMuxPins::AIN19),
	//...
};

#endif


class ADC_Peripheral
{
private:
	int test;

public:
	void begin(void);
	uint32_t read_blocking(adcPortMap pin);
	float GetMotorVoltage(void);
	int32_t getTemperature(void);
	
	//int32_t readPort(); 			//read using standard port designation
	//int32_t readPinArduino()		//read using pin number
};
