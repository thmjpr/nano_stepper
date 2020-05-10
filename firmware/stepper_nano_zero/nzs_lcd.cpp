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
#define skip_when_no_display() if(displayEnabled == false){return;}

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
	}
	else
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
			LOG("Calling function for %s", ptrMenu[menuIndex].str);
			if (ptrOptions != NULL)
			{
				char *ptrArgV[1];
				char str[25] = {0};
				ptrArgV[0] = str;
				sprintf(str,"%d",optionIndex);
				LOG("Calling function for %s %s", ptrMenu[menuIndex].str, str);
				ptrMenu[menuIndex].func(1,ptrArgV);
				ptrOptions = NULL;
				optionIndex = 0;
			}else
			{
				int i;
				i = ptrMenu[menuIndex].func(0, NULL);
				if (ptrMenu[menuIndex].ptrOptions != NULL)
				{
					LOG("displaying options for %s %d", ptrMenu[menuIndex].str, i);
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
		showStatus();
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

//--------------------------------------------
// 
void LCD::showStatus()
{
	skip_when_no_display();
	
	char buf[30] = { 0 };
	int32_t error;
	static uint32_t t0 = 0;
	static bool rpmDone = false;
	static int64_t lastAngle, degrees;
	static int32_t rpm = 0, lasttime = 0, dt =0;

	if ((millis() - t0) > 500)		//update every 500ms
		{
			int32_t x = 0, y, d;

			//do first half of RPM measurement
			if(!rpmDone)
			{
				//LOG("loop time is %dus",ptrStepperCtrl->getLoopTime());
				lastAngle = ptrStepperCtrl->getCurrentAngle();
				lasttime = millis();
				rpmDone = true;
				return;
			}

			//do the second half of rpm measurement and update LCD.
			if(rpmDone && (millis() - lasttime) > (dt))
			{
				rpmDone = false;
				degrees = ptrStepperCtrl->getCurrentAngle();
				y = millis() - lasttime;
				t0 = millis();
				d = abs((int64_t)(lastAngle - degrees));

				if (d > 0)	//if moving
					{
						x = ((int64_t)d * (60 * 1000UL)) / ((int64_t)y * ANGLE_STEPS);
					}

				lastAngle = degrees;
				rpm = (int32_t)x; 				//(7*RPM+x)/8; //average RPMs c
				if(rpm > 500)
					dt = 10;
				else //if (RPM < 100)
					dt = 100;
	
				display.clearDisplay();
	
				//RPM
				display.setTextSize(1);
				display.setCursor(0, 0);
				display.print("RPM:");

				display.setTextSize(2);
				display.setCursor(0, 10);
				sprintf(buf, "%03d", rpm);
				display.println(buf);
	
				//Error
				error = ptrStepperCtrl->getLoopError();
				error = (error * 360 * 100) / (int32_t)ANGLE_STEPS;
				
				display.setTextSize(1);
				display.setCursor(64, 0);
				if (error > 0)
					display.print("Error:");			//pos
				else
					display.print("Error:   -");		//neg
		
				x = error / 100;
				
				if(error > 9999)
					sprintf(buf, "%03d", x);    									//print xxx amount of error								
				else if(error > 0)
					sprintf(buf, "%02d.%02d", x, abs(error - (x * 100)));     		//print xx.yy amount of error
				else if(error > -9999)
					sprintf(buf, "%02d.%02d", abs(x), abs(error - (x * 100)));     	//print -xx.yy amount of error
				else
					sprintf(buf, "%03d", abs(x));								   	//print -xxx amount of error
				
				display.setTextSize(2);
				display.setCursor(64, 12);		
				display.println(buf);

				//Degrees
				display.setTextSize(1);
				display.setCursor(0, 35);
				display.print("Deg:");
				degrees = (degrees * 360 * 10) / (int32_t)ANGLE_STEPS;

				// if too large to display
				if(abs(degrees) > 9999)
				{
					degrees = degrees / 1000;
					x = degrees / 10;
					y = abs(degrees - (x * 10));
					sprintf(buf, "%03d.%01uk", x, y);
				}
				else
				{
					x = degrees / 10;
					y = abs(degrees - (x * 10));
					sprintf(buf, "%03d.%01u", x, y);
				}
	
				display.setCursor(0, 45);
				display.println(buf);
	
				//put degrees symbol..
	
	
				//----------------- control mode
				display.setTextSize(1);
				display.setCursor(90, 52);
	
				switch (ptrStepperCtrl->getControlMode())
				{
				case feedbackCtrl::SIMPLE:
					sprintf(buf, "simpl");
					break;

				case feedbackCtrl::POS_PID:
					sprintf(buf, "pPID");
					break;

				case feedbackCtrl::POS_VELOCITY_PID:
					sprintf(buf, "vPID");
					break;

				case feedbackCtrl::OPEN:
					sprintf(buf, "open");
					break;
				case feedbackCtrl::OFF:
					sprintf(buf, "off");
					break;
				default:
					display.setCursor(80, 52);
					sprintf(buf, "err %u", (uint32_t)ptrStepperCtrl->getControlMode());
					break;
				}
				display.println(buf);
	
				//Er En, !Er !En maybe?
	
	
				display.display();   		//update
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
	display.setTextColor(WHITE);
	
	display.setCursor(0, 0);
	display.print("CAL");
	display.setCursor(0, 57);

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

	//TODO: could also put a deviation plot, etc. 
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

#else		//LCD display disabled
void LCD::updateLCD(void){return;}
void LCD::showMenu(void){return;}
void LCD::updateMenu(void){return;}
void LCD::showOptions(void){return;}
void LCD::forceMenuActive(void){return;}
void LCD::setMenu(menuItem_t *pMenu){return;}
void LCD::begin(StepperCtrl *ptrStepperCtrl){return;} 	//sets up the LCD
void LCD::process(void){return;} 							//processes the LCD and updates as needed
void LCD::showSplash(void){return;}
void LCD::lcdShow(const char *line1, const char *line2, const char *line3){return;}
void LCD::showCalibration(int current_step){return;}

#endif