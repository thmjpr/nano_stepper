/*
 * nzs_lcd.h
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
 *
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

#ifndef NZS_LCD_H_
#define NZS_LCD_H_

#include "Arduino.h"
#include "syslog.h"
#include "board.h"
#include "stepper_controller.h"
#include "adc_analog.h"

#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "gfxfont.h"

#define LCD_WIDTH	128
#define LCD_HEIGHT	64

#define NAME_LEN 		13		//Max function name
#define LCD_MAX_CHAR	11		//Max characters on LCD line

typedef struct {
	char str[NAME_LEN];
	uint32_t val;
} options_t;

typedef struct {
	char str[NAME_LEN];

	//only one of the following should be not null
	int (*func)(int argc, char *argv[]);
	options_t *ptrOptions;

} menuItem_t;


class LCD
{
	private:
		bool displayEnabled;			//Is display active/responding
		Adafruit_SSD1306 display;
		StepperCtrl *ptrStepperCtrl;
		ADC_Peripheral *ptrADC;
		menuItem_t *ptrMenu;
		int32_t menuIndex;
		bool menuActive;
		options_t *ptrOptions;
		int32_t optionIndex;
		int32_t buttonState;
		static const uint8_t icon_splash_screen[];

		void showStatus(void); 						//Calculate rpm, error, show on display
		void showMenu(void);
		void updateMenu(void);
		void showOptions(void);

	public:
		bool displayEn() const { return displayEnabled; }
		void forceMenuActive(void);
		void setMenu(menuItem_t *pMenu);
		void begin(StepperCtrl *ptrStepperCtrl, ADC_Peripheral *ptrADC);	//sets up the LCD
		void showSplash(void);
		void process(void);							//determine what to display
		void lcdShow(const char *line1, const char *line2, const char *line3);
		void showCalibration(int current_step);		//
		void showStepSize(float step);				//
		void showInfo(const int32_t motorcurrent, const int32_t microstep);						//Show various config info?
};


#endif /* NZS_LCD_H_ */
