#pragma once

#define GCLK_OSCCTRL_DFLL48							0 //DFLL48 input clock source
#define GCLK_OSCCTRL_FDPLL0							1	//Reference clock for FDPLL1
#define GCLK_OSCCTRL_FDPLL1							2	//Reference clock for FDPLL2
#define GCLK_OSCCTRL_FDPLL0_32K                     3 
#define GCLK_OSCCTRL_FDPLL1_32K                     3
#define GCLK_SDHC0_SLOW                             3
#define GCLK_SDHC1_SLOW                             3
#define GCLK_SERCOM_SLOW		                    3
#define GCLK_EIC	                                4 
#define GCLK_FREQM_MSR				                5	//FREQM Measure
#define GCLK_FREQM_REF					            6	//FREQM Reference
#define GCLK_SERCOM0_CORE				            7 
#define GCLK_SERCOM1_CORE			                8 
#define GCLK_TC0					                9 
#define GCLK_TC1									9
#define GCLK_USB USB                                10
//#defin 22 : 11 GCLK_EVSYS[0..11] EVSYS[0..11]     e
#define GCLK_EVSYS0									11
#define GCLK_EVSYS1									12
#define GCLK_EVSYS2									13
#define GCLK_EVSYS3									14
#define GCLK_EVSYS4									15
#define GCLK_EVSYS5									16
#define GCLK_EVSYS6									17
#define GCLK_EVSYS7									18
#define GCLK_EVSYS8									19
#define GCLK_EVSYS9									20
#define GCLK_EVSYS10								21
#define GCLK_EVSYS11								22

#define GCLK_SERCOM2_CORE				            23
#define GCLK_SERCOM3_CORE				            24
#define GCLK_TCC0						            25
#define GCLK_TCC1									25
#define GCLK_TC2									26
#define GCLK_TC3									26 
#define GCLK_CAN0									27
#define GCLK_CAN1									28
#define GCLK_TCC2									29
#define GCLK_TCC3									29
#define GCLK_TC4									30
#define GCLK_TC5									30
#define GCLK_PDEC									31
#define GCLK_AC                                     32
#define GCLK_CCL									33
#define GCLK_SERCOM4_CORE							34
#define GCLK_SERCOM5_CORE							35
#define GCLK_SERCOM6_CORE							36
#define GCLK_SERCOM7_CORE							37
#define GCLK_TCC4									38
#define GCLK_TC6									39
#define GCLK_TC7									39
#define GCLK_ADC0									40
#define GCLK_ADC1									41
#define GCLK_DAC									42
//#define 44:43 GCLK_I2S I2S                        e 
#define GCLK_SDHC0									45
#define GCLK_SDHC1									46
#define GCLK_CM4_TRACE								47
