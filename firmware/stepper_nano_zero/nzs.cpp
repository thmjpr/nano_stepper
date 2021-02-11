/*
 * nzs.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
 *
 *	
 invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

#include "nzs.h"
#include "commands.h"
#include "nonvolatile.h"
#include "angle.h"
#include "eeprom.h"
#include "steppin.h"
#include "Watchdog.h"

#pragma GCC push_options
#pragma GCC optimize ("-Ofast")

#define skip_when_no_display() if(!Lcd.displayEn()){return 0;}

//SAMD51 notes:
//- try enable cache
//- should be compiling for G18 not J19
// hack for now: changed 0x3000 sram to 0x2000 in linker script, flash to 0x4000


//TODO:
//can show direction/step/error/enable pin status on LCD, maybe just enable?

//can show motor current or power level on LCD

//adapter board required (JST see mks)

//anti-hunt and enter standby after x seconds (?), maybe not needed w/ existing hold current setting 

//Backlash: a way to configure speed of the backlash move? or could it always be max. or implement a general maximum speed


/*
enter brake mode if not moving ?

peak current if error is excessive ? 

safety, if temp > x, or prevent certain pwm levels ..

Read sensor more often, then average it ?

on Nema23 version : change timer to different pin WO[0] for example ..
maybe check samd51 too ?

thermal cam of motor ?

show current on main display(if we drop vref or something ?)

20 % fan cooling from both sides best
*/
	
eepromData_t PowerupEEPROM = { 0 };
volatile bool enableState = true;
int32_t dataEnabled = 0;	//print out
StepperCtrl stepperCtrl;	//
LCD Lcd;					//
ADC_Peripheral adc;			//
Watchdog wdtimer;			//

int menuInfo(int argc, char *argv[])
{
	skip_when_no_display();
	
	Lcd.showInfo(NVM->motorParams.currentMa, NVM->SystemParams.microsteps);
	
	delay(1000);			//show for a bit
	return 0;
}
	
int menuCalibrate(int argc, char *argv[])
{
	skip_when_no_display();
	stepperCtrl.calibrateEncoder(&Lcd);
}

int menuTestCal(int argc, char *argv[])
{
	skip_when_no_display();
	
	Angle error;
	int32_t x, y;
	char str[25];
	error = stepperCtrl.maxCalibrationError();

	x = (36000 * (int32_t)error) / ANGLE_STEPS;
	LOG("Error %d %d", (int32_t)error, x);
	y = x / 100;
	x = x - (y * 100);
	x = abs(x);
	sprintf(str, "%d.%02d deg", y, (int)x);

	Lcd.lcdShow("Cal Error", str, "");

	LOG("Calibration error %s", str);

	while (digitalRead(PIN_SW3) == 1)
	{
		//wait for button press
	}
	while (digitalRead(PIN_SW3) == 0)
	{
		//wait for button release
	}

	return 0;
}


//search through options list to find current setting
int get_idx_from_menu_val(options_t (*options), uint32_t val)
{
	int i;
	
	for (i = 0; i < 30; i++)	
	{	
		if (options[i].str[0] == NULL)
			return 0;
		
		if (options[i].val == val)
			return i;
	}
	return 0;
}


//Number of steps per revolution
static options_t fullStepsOptions[]{
		{"200-1.8", 200},		//1.8 deg
		{"400-0.9", 400},		//0.9 deg
		{""},
};


//returns the index of the stepOptions when called with no arguments.
int motorSteps(int argc, char *argv[])
{
	MotorParams_t params;
	int32_t i, idx;

	memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params)); 		//retrieve all motor params
	
	if (argc == 1)
	{
		idx = atol(argv[0]); 			//index
		i = fullStepsOptions[idx].val; 		//get numerical value
		
		if (i != params.fullStepsPerRotation)
		{
			params.fullStepsPerRotation = i;
			nvmWriteMotorParms(params);
		}
	}

	return get_idx_from_menu_val(&fullStepsOptions[0], params.fullStepsPerRotation);
}

//Available motor currents
static  options_t currentOptions[]{
		{"0", 0},			//useful for measurement mode?
		{"100", 100},
		{"200", 200},
		{"300", 300},
		{"400", 400},
		{"500", 500},
		{"600", 600},
		{"700", 700},
		{"800", 800},
		{"900", 900},
		{"1.0A", 1000},
		{"1.2A", 1200},
		{"1.4A", 1400},
		{"1.5A", 1500},
		{"1.7A", 1700},
		{"1.8A", 1800},
		{"1.9A", 1900},
		{"2.0A", 2000},		//Max of A4954
		{""},
};

//Limit main motor current of stepper
int motorCurrent(int argc, char *argv[])
{
	MotorParams_t params;
	int32_t i, idx;

	memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params));		//retrieve all motor params
	
	if (argc == 1)
	{
		idx = atol(argv[0]);			//index
		i = currentOptions[idx].val;	//get numerical value
		
		if (i != params.currentMa)
		{
			params.currentMa = i;
			nvmWriteMotorParms(params);
		}
	}
	
	return get_idx_from_menu_val(&currentOptions[0], params.currentMa);
}


//Limit current in hold position (low error state)
int motorHoldCurrent(int argc, char *argv[])
{
	MotorParams_t params;
	int32_t i, idx;

	memcpy((void *)&params, (void *)&NVM->motorParams, sizeof(params)); 		//retrieve all motor params
	
	if (argc == 1)
	{
		idx = atol(argv[0]); 			//index
		i = currentOptions[idx].val; 	//get numerical value

		if (i != params.currentHoldMa)
		{
			params.currentHoldMa = i;
			nvmWriteMotorParms(params);
		}
	}
	
	return get_idx_from_menu_val(&currentOptions[0], params.currentHoldMa);
}

static  options_t microstepOptions[]{
		{"1", 1},
		{"2", 2},
		{"4", 4},
		{"8", 8},
		{"16", 16},
		{"32", 32},
		{"64", 64},
		{"128", 128},
		{"256", 256},
		{""}
};

//argc = 1 to write new value to NVM
//argc = 0 to return current setting?
int microsteps(int argc, char *argv[])
{
	SystemParams_t params;
	int i, idx;
	memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
	
	if (argc == 1)
	{
		idx = atol(argv[0]);
		i = microstepOptions[idx].val;  	//get numerical value

		if (i != params.microsteps)
		{
			params.microsteps = i;
			nvmWriteSystemParms(params);
			
		//calculate backlash degrees -> # of steps
		if(params.backlashDegrees > 0)
			stepperCtrl.setBacklashSteps((params.backlashDegrees * NVM->motorParams.fullStepsPerRotation * params.microsteps) / 360);
		}
	}
	
	return get_idx_from_menu_val(&microstepOptions[0], params.microsteps);
}

static  options_t controlLoopOptions[]{
		{"Off"},
		{"Open"},
		{"Simple"},
		{"Pos PID"},
		{"Vel PID"},
		{""}
};

int controlLoop(int argc, char *argv[])
{
	if (argc == 1)
	{
		feedbackCtrl i = (feedbackCtrl)atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i != params.controllerMode)
		{
			params.controllerMode = i;
			nvmWriteSystemParms(params);
		}
		return (int)i;
	}
	return (int)NVM->SystemParams.controllerMode;
}

#ifdef PIN_ERROR
static  options_t errorPinOptions[]{
		{"Error"},
		{"!Error"},
		{""}
};

int errorPin(int argc, char *argv[])
{
	if (argc == 1)
	{
		ErrorPinMode i = (ErrorPinMode)atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i != params.errorPinMode)
		{
			params.errorPinMode = i;
			nvmWriteSystemParms(params);
		}
		return (int)i;
	}
	return (int)NVM->SystemParams.errorPinMode;
}
#endif

#ifdef PIN_ENABLE
static  options_t enablePinOptions[]{
		{"Enable"},			//Active high
		{"!Enable"},		//Active low
		{"Ignore"},			//Always enabled
		{""}
};

int enablePin(int argc, char *argv[])
{
	if (argc == 1)
	{
		EnablePinMode i = (EnablePinMode) atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if (i != params.enablePinMode)
		{
			params.enablePinMode = i;
			nvmWriteSystemParms(params);
		}
		return (int)i;
	}
	return (int)NVM->SystemParams.enablePinMode;
}
#endif

static  options_t dirPinOptions[]{
		{"High CW"},
		{"High CCW"},
		{""}
};

int dirPin(int argc, char *argv[])
{
	if (argc == 1)
	{
		int i;
		i = atol(argv[0]);
		SystemParams_t params;
		memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
		if ((RotationDir)i != params.dirPinRotation)
		{
			params.dirPinRotation = (RotationDir)i;
			nvmWriteSystemParms(params);
		}
		return (int)i;
	}
	return (int)NVM->SystemParams.dirPinRotation;
}


//Backlash options in degrees
static  options_t backlashOptions[] {
	{"0 deg", 0},
	{"1 ", 1},
	{"2 ", 2},
	{"3 ", 3},
	{"4 ", 4},
	{"5 ", 5},
	{"6 ", 6},
	{"7 ", 7},
	{"8 ", 8},
	{"9 ", 9},
	{"10", 10},
	{""}
};

int backlashComp(int argc, char *argv[])
{
	int idx, i;
	SystemParams_t params;
	memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
	
	if (argc == 1)
	{
		idx = atol(argv[0]);
		i = backlashOptions[idx].val;
		
		if (i != params.backlashDegrees)		//if setting was changed
			{
				params.backlashDegrees = i;
				nvmWriteSystemParms(params);
			}
		
		//calculate backlash degrees -> # of steps
		if(params.backlashDegrees > 0)
			stepperCtrl.setBacklashSteps((params.backlashDegrees * NVM->motorParams.fullStepsPerRotation * params.microsteps) / 360);
	}
	
	return get_idx_from_menu_val(&backlashOptions[0], params.backlashDegrees);
}


//Auto sleep in seconds
static  options_t autoSleepOptions[] {
	{"0", 0},
	{"30s", 30},
	{"60s", 60},
	{"2min", 120},
	{"5min", 300},
	{"10min", 600},
	{""}
};


int autoSleep(int argc, char *argv[])
{
	int idx, i;
	SystemParams_t params;
	memcpy((void *)&params, (void *)&NVM->SystemParams, sizeof(params));
	
	if (argc == 1)
	{
		idx = atol(argv[0]);
		i = autoSleepOptions[idx].val;							//monstrosity
		
		if (i != params.autoSleepSeconds)		//if setting was changed
		{
			params.autoSleepSeconds = i;
			nvmWriteSystemParms(params);
		}
		return i;
	}
	
	return get_idx_from_menu_val(&autoSleepOptions[0], params.autoSleepSeconds);
}



//----------------------------------------------------------------------------------
//Main menu structure
static  menuItem_t MenuMain[]{
		{"Info",		menuInfo,		NULL},
		{"Motor mA",	motorCurrent,	currentOptions},
		{"Hold mA",		motorHoldCurrent, currentOptions},
		{"Microstep",	microsteps,		microstepOptions},
		{"Ctlr Mode",	controlLoop,	controlLoopOptions},
#ifdef PIN_ENABLE
		{"EnablePin",	enablePin,		enablePinOptions},
#endif
#ifdef PIN_ERROR
		{"Error Pin",	errorPin,		errorPinOptions},			//Need to test
#endif
		{"Dir Pin",		dirPin,			dirPinOptions},
		{"AutoSleep",	autoSleep,		autoSleepOptions},			//should be hidden/disabled if enable pin is enabled?
#ifdef USE_BACKLASH
		{"Backlash",	backlashComp,	backlashOptions},			//need to implement
#endif
		{"Calibrate",	menuCalibrate,	NULL},
		{"Test Cal",	menuTestCal,	NULL},
		{ "", NULL}
};

static  menuItem_t MenuCal[]{
		{"Calibrate", menuCalibrate, NULL},
		//bypass cal?
		{ "", NULL}
};


//===============================================================
// Enable pin
//this function is called when error pin changes as enable signal
static void enableInput(void)
{
	static bool lastState = true;
	bool enablePin;
#ifdef PIN_ENABLE

	enablePin = digitalRead(PIN_ENABLE);	//read our enable pin

	if (enablePin != lastState)
		{
			WARNING("Enable now %d", enablePin);
			lastState = enablePin;
		}

	if (NVM->SystemParams.enablePinMode == EnablePinMode::ACTIVE_HIGH)
	{
		enableState = enablePin;
	}
	else if (NVM->SystemParams.enablePinMode == EnablePinMode::ACTIVE_LOW)
	{
		enableState = !enablePin;
	}
	else if (NVM->SystemParams.enablePinMode == EnablePinMode::ALWAYS_ON)
	{
		enableState = true;
	}
	else{}
	
#endif

#ifdef USE_STEP_DIR_SERIAL

	static uint8_t pinCFG[2];
	static uint8_t pinMux[2];
	if (enableState == false && laststate == true)
	{
		// turn the step/dir to serial port
		//save pin config for restoring
		pinCFG[0] = getPinCfg(PIN_STEP_INPUT);
		pinCFG[1] = getPinCfg(PIN_DIR_INPUT);
		pinMux[0] = getPinMux(PIN_STEP_INPUT);
		pinMux[1] = getPinMux(PIN_DIR_INPUT);

		//lets see if the step pin has interrupt enabled
		if (pinMux[0] == PORT_PMUX_PMUXE_A_Val)
		{
			EExt_Interrupts in = g_APinDescription[PIN_STEP_INPUT].ulExtInt;
			EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in); //disable the interrupt
			//we need to disable the interrupt
		}

		//now we need to set the pins to serial port peripheral (sercom0)
		setPinMux(PIN_STEP_INPUT, PORT_PMUX_PMUXE_C_Val);
		setPinMux(PIN_DIR_INPUT, PORT_PMUX_PMUXE_C_Val);

		//make sure that step pin is input with mux to peripheral
		setPinCfg(PIN_STEP_INPUT, PORT_PINCFG_PMUXEN | PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);

		//make sure that dir pin is an output with mux to peripheral
		setPinCfg(PIN_DIR_INPUT, PORT_PINCFG_PMUXEN);

		Serial1.begin(STEP_DIR_BAUD);

	}
	if (enableState == true && lastState == false)
	{
		Serial1.end();
		setPinMux(PIN_STEP_INPUT, pinMux[0]);
		setPinMux(PIN_DIR_INPUT, pinMux[1]);
		setPinCfg(PIN_STEP_INPUT, pinCFG[0]);
		setPinCfg(PIN_DIR_INPUT, pinCFG[1]);
		//turn step/dir pins back to GPIO
		if (PORT_PMUX_PMUXE_A_Val == pinMux[0])
		{
			//if interrupt was enabled for step pin renable it.
			EExt_Interrupts in = g_APinDescription[PIN_STEP_INPUT].ulExtInt;
			EIC->INTENSET.reg = EIC_INTENCLR_EXTINT(1 << in); //enable the interrupt
		}

	}
#endif //USE_STEP_DIR_SERIAL
	lastState = enableState;
}


static void enable_callback(void)
{
	enableInput();
}

void enablePinSetup(void)
{
	attachInterrupt(digitalPinToInterrupt(PIN_ENABLE), enable_callback, CHANGE);
	
#ifdef _SAMD21_	
	
#else	//SAMD51
	error implement
#endif
}


static void time_handler(void)
{
	static bool led = false;
	if (led)
		GPIO_HIGH(PIN_GREEN_LED)
	else
		GPIO_LOW(PIN_GREEN_LED)
	led = !led;
}

//Timer 5 interrupt - set to occur at rate NZS_CONTROL_LOOP_HZ.
//__attribute__((section(".data")))  not enough space to put in RAM

#ifdef _SAMD21_
void TC5_Handler()
{
	TC5->COUNT16.INTFLAG.bit.MC0 = 1;    	//Clear compare match flag
#else
void TC2_Handler()
{
	TC2->COUNT16.INTFLAG.bit.MC0 = 1;      	//Clear compare match flag
#endif
	int error;
	interrupts();   							//allow other interrupts
	error = (stepperCtrl.processFeedback());    //handle the control loop
	RED_LED(error);   							//light red LED when any error

#ifdef PIN_ERROR						//does it need to be handled here? maybe.
	
		bool level = !NVM->SystemParams.errorLogic;
	//GPIO_OUTPUT(PIN_ERROR); should be covered elsewhere?
	
	if(error)		//Set error output pin appropriately if error state
		{
		if (level)
			GPIO_HIGH(PIN_ERROR)
	else
		GPIO_LOW(PIN_ERROR)
	}
	else
	{
		if (!level)
			GPIO_HIGH(PIN_ERROR)	
	else
		GPIO_LOW(PIN_ERROR)
	}
#endif
		
}

//check the NVM and set to defaults if there is any
void validateAndInitNVMParams(void)
{
	if (false == NVM->sPID.parametersValid)
	{
		LOG("sPID invalid %0x %0x %0x, %0x addr %0x", NVM->sPID.Ki, NVM->sPID.Kp, NVM->sPID.Kd, NVM->sPID.parametersValid, &NVM->sPID.parametersValid);
		nvmWrite_sPID(0.9, 0.0001, 0.01);
	}

	if (false == NVM->pPID.parametersValid)
	{
		LOG("pPID invalid %0x", NVM->pPID.parametersValid);
		nvmWrite_pPID(0.8, 0.01, 0.01);
	}

	if (false == NVM->vPID.parametersValid)
	{
		nvmWrite_vPID(2.0, 1.0, 1.0);
	}

	if (false == NVM->SystemParams.parametersValid)
	{
		LOG("sys param invalid");
		SystemParams_t params;
		params.microsteps = 16;
		params.controllerMode = feedbackCtrl::SIMPLE;
		params.dirPinRotation = RotationDir::CW; 				//default to clockwise rotation when dir is high
		params.errorLimit = (int32_t)ANGLE_FROM_DEGREES(1.8);
		params.errorPinMode = ErrorPinMode::ACTIVE_LOW; 		//default to active low
		params.enablePinMode = EnablePinMode::ALWAYS_ON;		//default to always on
		params.autoSleepSeconds = 0;							//default to autosleep disabled
		params.homePin = -1;
		params.errorLogic = false;
		params.enableLogic = false;
		params.homeAngleDelay = ANGLE_FROM_DEGREES(10);
		params.backlashDegrees = 0;
		nvmWriteSystemParms(params);
	}
	//the motor parameters are check in the stepper_controller code
	// as that there we can auto set much of them.
}

void SYSCTRL_Handler(void)
{	
#ifdef _SAMD21_
	if (SYSCTRL->INTFLAG.reg & SYSCTRL_INTFLAG_BOD33DET)
	{
		eepromFlush();  //flush the eeprom
		SYSCTRL->INTFLAG.reg |= SYSCTRL_INTFLAG_BOD33DET;
	}
#else

#endif
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncBOD33(void) __attribute__((always_inline, unused));
static void syncBOD33(void) 
{
#ifdef _SAMD21_
	while (SYSCTRL->PCLKSR.bit.BOD33RDY == 1)
#else
	while(SUPC->STATUS.bit.B33SRDY == 0) {}		//set not cleared???
#endif
	{
	}
}

//Configure brownout level
static void configure_bod(void)
{
#ifdef _SAMD21_
	SYSCTRL->BOD33.reg = SYSCTRL_BOD33_ACTION_INTERRUPT |   //generate interrupt when BOD is triggered
	SYSCTRL_BOD33_LEVEL(40) | 	//about 3V, was 48 (3.2V)
	SYSCTRL_BOD33_HYST | 		//enable hysteresis
	SYSCTRL_BOD33_ENABLE; 		//turn module on

	LOG("BOD33 %02X", SYSCTRL->BOD33.reg);
	SYSCTRL->INTENSET.reg |= SYSCTRL_INTENSET_BOD33DET;

	NVIC_SetPriority(SYSCTRL_IRQn, 1); 			//make highest priority as we need to save eeprom //save what?
	NVIC_EnableIRQ(SYSCTRL_IRQn); 				//Enable InterruptVector	
#else	//SAMD51

	syncBOD33();
	SUPC->BOD33.bit.LEVEL = 40;  		//Set BOD level (? what voltage now should be 3V
	SUPC->BOD33.bit.PSEL = SUPC_BOD33_PSEL_DIV4_Val; 	//Divider?
	//can set configuration in Active/standby/hib mode if needed
	SUPC->BOD33.bit.ACTION = SUPC_BOD33_ACTION_INT_Val;	//generate interrupt when BOD is triggered
	SUPC->BOD33.bit.HYST = true; 		//enable hysterisis
	SUPC->BOD33.bit.ENABLE = true;		//enable BOD
	
	syncBOD33();
	SUPC->INTFLAG.bit.BOD33DET = true;		//clear flag if present
	
	LOG("BOD33 %02X", SUPC->BOD33.reg);
	
	SUPC->INTENSET.bit.BOD33DET = true;		//??

	NVIC_SetPriority(SUPC_1_IRQn, 1);		//Set priority of BOD interrupt (?)
	NVIC_EnableIRQ(SUPC_1_IRQn);			//Enable InterruptVector
#endif

}

//Setup
void NZS::begin(void)
{
	int to = 20;
	volatile stepCtrlError stepCtrlError;

	boardSetupPins();		//set up the pins correctly on the board.
	RED_LED(true);			//
	adc.begin();			//enable ADC
	
	Serial232.begin(SERIAL_BAUD); 	//setup the serial port for syslog

#ifndef CMD_SERIAL_PORT
	SysLogInit(&Serial232, LOG_DEBUG); //use SWO for the sysloging
	//pinPeripheral(PIN_TXD, PIO_SERCOM_ALT);
	//pinPeripheral(PIN_RXD, PIO_SERCOM_ALT);
#else
	SysLogInit(NULL, LOG_WARNING);
#endif
	
	LOG("Power up!");

	if (digitalRead(PIN_USB_PWR))
	{
		//start up the USB serial interface
		SerialUSB.begin(SERIAL_BAUD_USB);
		
		//wait for USB serial port to come alive
		while (!SerialUSB)
		{
			to--;
			if (to == 0)
			{
				break;
			}
			delay(500);
		};     //wait for serial
	}
	else
	{
		WARNING("USB Not connected");
	}

	validateAndInitNVMParams();

	LOG("EEPROM INIT");
	if (EEPROM_OK == eepromInit()) //init the EEPROM
	{
		eepromRead((uint8_t *)&PowerupEEPROM, sizeof(PowerupEEPROM));
	}

	configure_bod(); 					//configure the brownout detect

	LOG("Testing LCD");
	Lcd.begin(&stepperCtrl, &adc);		//Attempt connection to LCD
	Lcd.showSplash();					//Splash screen

	LOG("command init!");
	commandsInit();						//setup command handler system
	stepCtrlError = stepCtrlError::No_CAL;
		
	while (stepCtrlError::No_ERROR != stepCtrlError)
	{
		LOG("init the stepper controller");
		stepCtrlError = stepperCtrl.begin(&Lcd); //start controller before accepting step inputs

		//todo we need to handle error on LCD and through command line
		if(stepCtrlError::No_POWER == stepCtrlError)
		{
			SerialUSB.println("Appears that there is no Motor Power");			//No motor power, driver IC dead, or bad cable to motor
			SerialUSB.println("Connect motor power!");
			Lcd.lcdShow("Waiting:", "Motor", "Power");

			while (stepCtrlError::No_POWER == stepCtrlError)
			{
				stepCtrlError = stepperCtrl.begin(&Lcd);  //start controller before accepting step inputs
			}
		}
		
		//TODO: allow setting motor current prior to calibration...
		if (stepCtrlError:: No_CAL == stepCtrlError)
		{
			SerialUSB.println("You need to Calibrate");
			Lcd.lcdShow("   NOT ", "Calibrated", " ");
			delay(800);
			Lcd.setMenu(MenuCal);
			Lcd.forceMenuActive();

			//TODO add code here for LCD and command line loop
			while(false == stepperCtrl.calibrationValid())
			{
				commandsProcess(); //handle commands
				Lcd.process();
			}
			Lcd.setMenu(NULL);
		}

		if (stepCtrlError::No_ENCODER == stepCtrlError)
		{
			SerialUSB.println("AS5047D not working");
			SerialUSB.println(" try disconnecting power from board for 15+mins");
			SerialUSB.println(" you might have to short out power pins to ground");
			Lcd.lcdShow("Encoder", " Error!", " REBOOT");

			while (1)
			{

			}
		}

	}

	stepPinSetup();					//setup the step pin		//race issue where TC5 is triggering before TC2 setup.
	enablePinSetup();
	Lcd.setMenu(MenuMain);			//
	
#ifdef PIN_ENABLE
	//attachInterrupt(digitalPinToInterrupt(PIN_ENABLE), enableInput, CHANGE);
#ifdef _SAMD21_
	NVIC_SetPriority(EIC_IRQn, 0);  //set port A interrupt as highest priority		//**FF
#else //SAMD51
	NVIC_SetPriority(EIC_15_IRQn, 0);	//PA15 = EXTINT.. yeah hardcode..
#endif // _SAMD21_
#else
	//attachInterrupt(digitalPinToInterrupt(PIN_ERROR), enableInput, CHANGE);
#endif

	SmartPlanner.begin(&stepperCtrl);
	enableInput();						//get initial enable input pin state
	
	//IFDEF enable watchdog?
	wdtimer.setup();					//Begin WDT, could start earlier too..
	LOG("SETUP DONE!");
	RED_LED(false);
}

//
void printLocation(void)
{
	char buf[128] = { 0 };
	Location_t loc;
	int32_t n, i, len;
	int32_t pktSize;

	if (dataEnabled == 0)
	{
		//RED_LED(false);
		return;
	}

	//the packet length for binary print is 12bytes
	// assuming rate of 6Khz this would be 72,000 baud
	i = 0;
	n = stepperCtrl.getLocation(&loc);
	if (n == -1)
	{
		//RED_LED(false);
		return;
	}

	len = 0;
	pktSize = sizeof(Location_t) + 1; //packet lenght is size location plus sync byte

	//binary write
	while ((n >= 0) && (len <= (128 - pktSize)))
	{
		memcpy(&buf[len], &loc, sizeof(Location_t));
		len += sizeof(Location_t);
		buf[len] = 0xAA; //sync
		len++;
		buf[len] = sizeof(Location_t); //data len
		len++;
		n = stepperCtrl.getLocation(&loc);
		i++;
	}
	SerialUSB.write(buf, len);

	//hex write
	// hex write is 29 bytes per tick, @ 6khz this 174000 baud
	//   while(n>=0 && (i*29)<(200-29))
	//   {
	//      sprintf(buf,"%s%08X\t%08X\t%08X\n\r",buf,loc.microSecs,loc.desiredLoc,loc.actualLoc);
	//      n=stepperCtrl.getLocation(&loc);
	//      i++;
	//   }
	//   SerialUSB.write(buf,strlen(buf));

	if (n <= 0)
	{
		RED_LED(false);
	}
	else
	{
		RED_LED(true);
	}
	return;
}

//Main loop
void NZS::loop(void)
{
	eepromData_t eepromData;
	static uint32_t one_second_timer = millis() + 1000;
	static uint32_t sleep_timer = 0;
	
	if (dataEnabled == 1)
	   {
	      LOG("loop time is %dus",stepperCtrl.getLoopTime());
	   }

	// Check if enable state has been updated
	if(enableState != stepperCtrl.getEnable())
	{
		stepperCtrl.enable(enableState);
	}
	
	//Continuously update EEPROM with current position 
	//move this to within eeprom or somewhere else
	if(UPDATE_EEPROM)
	{
		eepromData.angle = stepperCtrl.getCurrentAngle();
		eepromData.encoderAngle = stepperCtrl.getEncoderAngle();
		eepromData.valid = 1;
		eepromWriteCache((uint8_t *)&eepromData, sizeof(eepromData));
	}
	
	commandsProcess(); 		//handle commands
	Lcd.process(); 			//update display and process keypresses - approx 1000 cycles
	//stepperCtrl.PrintData(); //prints steps and angle to serial USB.
	printLocation(); 		//print out the current location

	//could safe shutdown when voltage < 10V
	if(millis() > one_second_timer)		//once per second
	{
		wdtimer.clear();				//reset watchdog
		one_second_timer += 1000;

		if (NVM->SystemParams.autoSleepSeconds > 0)
		{
			if (++sleep_timer > NVM->SystemParams.autoSleepSeconds)
			{
				stepperCtrl.sleep(true);		//currentHoldma = 0
				//enableState = false; shouldnt use this because position will be lost.
				sleep_timer = 0xFFFF;		//prevent overflow
				
				//then need to restore it once a step input is recieved
				//if(error > 0)?
				//sleep_timer = 0;
				//stepperCtrl.sleep(false);
			}
		}
	}

	return;
}

#pragma GCC pop_options
