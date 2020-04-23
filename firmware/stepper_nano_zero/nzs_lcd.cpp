/*
 * nzs_lcd.cpp
 *
 *  Created on: Dec 8, 2016
 *      Author: trampas
 *
 *	Misfit Tech invests time and resources providing this open source code,
 *	please support Misfit Tech and open-source hardware by purchasing
 *	products from Misfit Tech, www.misifittech.net!
 *
 *	Written by Trampas Stern  for Misfit Tech.
 *	BSD license, check license.txt for more information
 *	All text above, must be included in any redistribution
 *********************************************************************/

#include "nzs_lcd.h"
#include <string.h>
#include <stdio.h>
#include <Wire.h>

#ifndef DISABLE_LCD
void LCD::begin(StepperCtrl *ptrsCtrl)
{
#ifndef MECHADUINO_HARDWARE
	pinMode(PIN_SW1, INPUT_PULLUP);
	pinMode(PIN_SW2, INPUT_PULLUP);
	pinMode(PIN_SW3, INPUT_PULLUP);
#endif
	buttonState = 0;

	//we need access to the stepper controller
	ptrStepperCtrl = ptrsCtrl; //save a pointer to the stepper controller

	ptrMenu = NULL;
	menuIndex = 0;
	menuActive = false;
	optionIndex = 0;
	ptrOptions = NULL;
	displayEnabled = true;

	//check that the SCL and SDA are pulled high
	pinMode(PIN_SDA, INPUT);
	pinMode(PIN_SCL, INPUT);

	if (digitalRead(PIN_SDA) == 0)
	{
		//pin is not pulled up
		displayEnabled = false;
	}
	if (digitalRead(PIN_SCL) == 0)
	{
		//pin is not pulled up
		displayEnabled = false;
	}

	if (displayEnabled)
	{
		displayEnabled = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
	}else
	{
		WARNING("SCL/SDA not pulled up");
	}
	if (displayEnabled == false)
	{
		WARNING("No display found, LCD will not be used");
	}

	Wire.setClock(800000);
}

void __attribute__((optimize("Ofast"))) LCD::lcdShow(const char *line1, const char *line2, const char *line3)
{
	skip_when_no_display();

	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.println(line1);
	display.setCursor(0,20);
	display.println(line2);
	display.setCursor(0,40);
	display.println(line3);
	display.display();
}


//Logo splash screen
void LCD::showSplash(void)
{
	skip_when_no_display();

	display.clearDisplay();
	display.drawBitmap(0, 0, icon_splash_screen, 128, 64, WHITE);		//Show logo
	display.setCursor(60, 55);
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.println(BUILD_DATE);
	display.display();							//update
}

//
void LCD::setMenu(menuItem_t *pMenu)
{
	skip_when_no_display();

	ptrMenu = pMenu;
	menuIndex = 0;

#ifdef A5995_DRIVER
	lcdShow("MisfitTech", "NEMA 23", VERSION);
#else
	lcdShow("MisfitTech", "NEMA 17", VERSION);
#endif
}

//Show configuration options
void LCD::showOptions(void)
{
	skip_when_no_display();

	int32_t i, j;
	char str[3][26] = {0};

	i = optionIndex;
	j = 0;

	while((strlen(ptrOptions[i].str) > 0) && (j < 3))
	{
		if (i == optionIndex)
		{
			sprintf(str[j], "*%s", ptrOptions[i].str);
		}else
		{
			sprintf(str[j],  " %s", ptrOptions[i].str);
		}
		j++;
		i++;
	}

	lcdShow(str[0], str[1], str[2]);
	return;
}


void __attribute__ ((optimize("Ofast"))) LCD::showMenu(void)
{
	int32_t i = menuIndex, j = 0;
	char str[3][26] = {0};

	skip_when_no_display();

	while((ptrMenu[i].func != NULL) && (j < 3))
	{
		if (i == menuIndex)
		{
			sprintf(str[j], "*%s", ptrMenu[i].str);
		}else
		{
			sprintf(str[j], " %s", ptrMenu[i].str);
		}
		j++;
		i++;
	}

	//show exit if there is room
	if (j < 3)
	{
		if (j == 0)
		{
			sprintf(str[j],"*Exit");
		}else
		{
			sprintf(str[j]," Exit");
		}
	}

	lcdShow(str[0], str[1], str[2]);
	return;
}


void __attribute__ ((optimize("Ofast"))) LCD::updateMenu(void)
{
	skip_when_no_display();

	if (ptrOptions != NULL)
	{
		showOptions();
	}else
	{
		showMenu();
	}

	//handle push buttons
	if ((digitalRead(PIN_SW3) == 0) && (buttonState & 0x02) == 0)
	{
		buttonState |= 0x02;

		LOG("SW3 pressed");
		if (ptrMenu[menuIndex].func == NULL)
		{
			//exit pressed
			menuIndex = 0; //reset menu index
			menuActive = false;
			return;
		}

		if (ptrMenu[menuIndex].func != NULL)
		{
			LOG("Calling function for %s",ptrMenu[menuIndex].str);
			if (ptrOptions != NULL)
			{
				char *ptrArgV[1];
				char str[25] = {0};
				ptrArgV[0] = str;
				sprintf(str,"%d",optionIndex);
				LOG("Calling function for %s %s",ptrMenu[menuIndex].str,str);
				ptrMenu[menuIndex].func(1,ptrArgV);
				ptrOptions = NULL;
				optionIndex = 0;
			}else
			{
				int i;
				i = ptrMenu[menuIndex].func(0,NULL);
				if (ptrMenu[menuIndex].ptrOptions != NULL)
				{
					LOG("displaying options for %s %d",ptrMenu[menuIndex].str,i);
					ptrOptions = ptrMenu[menuIndex].ptrOptions;
					optionIndex = i;
				}
			}

			return;
		}
	}

	if ((digitalRead(PIN_SW1) == 0) && ((buttonState & 0x01) == 0))
	{
		buttonState |= 0x01;
		LOG("SW1 pressed");
		if (ptrOptions != NULL)
		{
			optionIndex++;
			if (strlen(ptrOptions[optionIndex].str) == 0)
			{
				optionIndex = 0;
			}
		} else
		{
			if (ptrMenu[menuIndex].func != NULL)
			{
				menuIndex++;
			} else
			{
				menuIndex = 0;
			}
		}
	}

	if (digitalRead(PIN_SW1))
	{
		buttonState &= ~0x01;
	}

	if (digitalRead(PIN_SW3))
	{
		buttonState &= ~0x02;
	}
}

void LCD::forceMenuActive(void)
{
	menuActive = true;
}

void __attribute__((optimize("Ofast")))LCD::process(void)
{
	skip_when_no_display();

	if (false == menuActive || ptrMenu == NULL)
	{
		updateLCD();
	}
	else
	{
		updateMenu();
	}

	if (digitalRead(PIN_SW2) == 0 && (buttonState & 0x04) == 0)
	{
		buttonState |= 0x04;
		menuActive = !menuActive;
	}

	if (digitalRead(PIN_SW2))
	{
		buttonState &= ~0x04;
	}
}
#endif

void LCD::updateLCD(void)
{
	skip_when_no_display();

	char str[3][25];
	static int highRPM = 0;
	int32_t y, z, err;

	static int64_t lastAngle, deg;
	static int32_t RPM = 0, lasttime = 0;

	bool state;
	static int32_t dt = 40;
	static uint32_t t0 = 0;

	static bool rpmDone = false;

	if ((millis() - t0) > 500)
	{
		int32_t x, d;

		//do first half of RPM measurement
		if (!rpmDone)
		{
			//LOG("loop time is %dus",ptrStepperCtrl->getLoopTime());
			lastAngle = ptrStepperCtrl->getCurrentAngle();
			lasttime = millis();
			rpmDone = true;
			return;
		}

		//do the second half of rpm measurement and update LCD.
		if (rpmDone && (millis() - lasttime) > (dt))
		{
			rpmDone = false;
			deg = ptrStepperCtrl->getCurrentAngle();
			y = millis() - lasttime;
			err = ptrStepperCtrl->getLoopError();
			t0 = millis();
			d = (int64_t)(lastAngle - deg);
			d = abs(d);
			x = 0;

			if (d > 0)
			{
				x = ((int64_t)d * (60 * 1000UL)) / ((int64_t)y * ANGLE_STEPS);
			}

			lastAngle = deg;
			RPM = (int32_t)x; //(7*RPM+x)/8; //average RPMs
			if (RPM > 500)
			{
				dt = 10;
			}
			else //if (RPM < 100)
			{
				dt = 100;
			}

			str[0][0] = '\0';
			//LOG("RPMs is %d, %d, %d",(int32_t)x,(int32_t)d,(int32_t)y);
			switch (ptrStepperCtrl->getControlMode())
			{
			case CTRL_SIMPLE:
				sprintf(str[0], "%03dRPM simp", RPM);
				break;

			case CTRL_POS_PID:
				sprintf(str[0], "%03dRPM pPID", RPM);
				break;

			case CTRL_POS_VELOCITY_PID:
				sprintf(str[0], "%03dRPM vPID", RPM);
				break;

			case CTRL_OPEN:
				sprintf(str[0], "%03dRPM open", RPM);
				break;
			case CTRL_OFF:
				sprintf(str[0], "%03dRPM off", RPM);
				break;
			default:
				sprintf(str[0], "error %u", ptrStepperCtrl->getControlMode());
				break;
			}

			err = (err * 360 * 100) / (int32_t)ANGLE_STEPS;
			//LOG("error is %d %d %d",err,(int32_t)ptrStepperCtrl->getCurrentLocation(),(int32_t)ptrStepperCtrl->getDesiredLocation());
			z = (err) / 100;
			y = abs(err - (z * 100));

			sprintf(str[1], "%01d.%02d err", z, y);
			deg = ptrStepperCtrl->getDesiredAngle();

#ifndef NZS_LCD_ABSOLUTE_ANGLE
			deg = deg & ANGLE_MAX; //limit to 360 degrees
#endif

			deg = (deg * 360 * 10) / (int32_t)ANGLE_STEPS;

			// if too large to display
			if (abs(deg) > 9999)
			{
				deg = deg / 1000;
				x = deg / 10;
				y = abs(deg - (x * 10));
				sprintf(str[2], "%03d.%01uKdeg", x, y);
			}

			else
			{
				x = deg / 10;
				y = abs(deg - (x * 10));
				sprintf(str[2], "%03d.%01udeg", x, y);
			}

			lcdShow(str[0], str[1], str[2]);
		}
	}
}

//--------------------------------------------
// 
void LCD::showCalibration(int current_step)
{
	char buf[30] = { 0 };
	int x, y, r = LCD_HEIGHT/3;
	float theta = (current_step * 2 * PI) / (float) CALIBRATION_TABLE_SIZE;

	x = (r * cos(theta)) + LCD_WIDTH/2;
	y = (r * sin(theta)) + LCD_HEIGHT/2;

	display.clearDisplay();
	display.setTextSize(1);
	display.setCursor(0, 0);
	display.print("CAL");
	display.setCursor(0, 57);
	display.setTextColor(WHITE);
	sprintf(buf, "%03d/%03d", current_step, CALIBRATION_TABLE_SIZE);
	display.println(buf);

	display.drawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, LCD_HEIGHT / 2 - 5, WHITE);
	display.drawCircle(LCD_WIDTH / 2, LCD_HEIGHT / 2, 2, WHITE);
	display.drawLine(LCD_WIDTH / 2, LCD_HEIGHT / 2, x, y, WHITE);

	if (current_step == CALIBRATION_TABLE_SIZE)
	{
		display.setCursor(85, 56);
		display.print("Saving");
	}

	//could also put a deviation plot, etc. 
	display.display();		//update
}


//--------------------------------------------
// Splash screen 128x64

const uint8_t LCD::icon_splash_screen[] = {
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x1,0xfc,0x0,0xff,0xff,0xc0,0x3e,0xff,0xf1,0xfc,0x3f,0x1,0xf8,0x1f,0xdf,0xf0,
	0x7,0xff,0x0,0xff,0xff,0xc0,0x7e,0xff,0xf3,0xfc,0x7f,0x83,0xfc,0x3f,0xdf,0xf8,
	0xf,0xff,0x80,0xff,0xff,0xc0,0xfe,0xff,0xf7,0xfc,0xff,0xc7,0xfe,0x7f,0xdf,0xfc,
	0x1f,0xff,0xc0,0xff,0xff,0xc1,0xfe,0xff,0xf7,0xfd,0xff,0xef,0xff,0x7f,0xdf,0xfe,
	0x3f,0xff,0xe0,0xff,0xff,0xc1,0xf0,0xf,0x7,0x81,0xe1,0xef,0xf,0x78,0x1e,0x1e,
	0x3f,0xff,0xe0,0xff,0xff,0xc1,0xf0,0xf,0x7,0x81,0xe1,0xef,0xf,0x78,0x1e,0x1e,
	0x7f,0xff,0xf0,0x0,0xff,0xc1,0xf0,0xf,0x7,0x81,0xe1,0xef,0xf,0x78,0x1e,0x1e,
	0x7f,0x8f,0xf0,0x0,0xff,0x81,0xf0,0xf,0x7,0x81,0xe1,0xef,0xf,0x78,0x1e,0x1e,
	0x7f,0x7,0xf0,0x1,0xff,0x81,0xf8,0xf,0x7,0xfd,0xe7,0xef,0x3f,0x7f,0xde,0x3e,
	0x7f,0x7,0xf0,0x1,0xff,0x1,0xfc,0xf,0x7,0xfd,0xe7,0xcf,0x3e,0x7f,0xde,0x7c,
	0x7f,0x7,0xf0,0x3,0xff,0x0,0xfc,0xf,0x7,0xfd,0xe7,0xf,0x38,0x7f,0xde,0xf8,
	0x7f,0x7,0xf0,0x3,0xfe,0x0,0xfc,0xf,0x7,0x81,0xe0,0xf,0x0,0x78,0x1e,0xf0,
	0x7f,0x7,0xf0,0x3,0xfe,0x0,0x7c,0xf,0x7,0x81,0xe0,0xf,0x0,0x78,0x1e,0xf0,
	0x7f,0x7,0xf0,0x7,0xfc,0x0,0x7c,0xf,0x7,0x81,0xe0,0xf,0x0,0x78,0x1e,0xf8,
	0x7f,0x7,0xf0,0x7,0xfc,0x0,0x7c,0xf,0x7,0x81,0xe0,0xf,0x0,0x78,0x1e,0x78,
	0x7f,0x7,0xf0,0xf,0xfc,0x3,0xfc,0xf,0x7,0xfd,0xe0,0xf,0x0,0x7f,0xde,0x7c,
	0x7f,0x7,0xf0,0xf,0xf8,0x3,0xf8,0xf,0x7,0xfd,0xe0,0xf,0x0,0x7f,0xde,0x3c,
	0x7f,0x7,0xf0,0x1f,0xf8,0x3,0xf0,0xf,0x3,0xfd,0xe0,0xf,0x0,0x3f,0xde,0x3e,
	0x7f,0x7,0xf0,0x1f,0xf0,0x3,0xe0,0xf,0x1,0xfd,0xe0,0xf,0x0,0x1f,0xde,0x3e,
	0x7f,0x7,0xf0,0x1f,0xf0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x7f,0x7,0xf0,0x3f,0xe0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x7f,0x7,0xf0,0x3f,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe,
	0x7f,0x7,0xf0,0x7f,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfc,
	0x7f,0x7,0xf0,0x7f,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf8,
	0x7f,0x7,0xf0,0x7f,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xf0,
	0x7f,0x7,0xf0,0xff,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xe0,
	0x7f,0x7,0xf0,0xff,0xff,0xcf,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xc0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
	0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0
};
