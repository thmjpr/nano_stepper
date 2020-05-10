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

/*
 *  This file implements a trajectory planner for use with serial
 *  interface. It allows the smart stepper to move at constant velocity.
 *  Additionally you can move to some location at constant velocity or
 *  with a trajectory curve
 */

#ifndef PLANNER_H_
#define PLANNER_H_
#include "board.h"
#include "stepper_controller.h"

#define PLANNER_UPDATE_RATE_HZ (3000UL) //how often planner updates PID

enum class PlannerMode {
	NONE = 0,
	CV = 1,
	  //constant velocity
	//PLANNER_CA =2, //constant accleration
	//PLANNER_S_CURVE =3, //s-curve move
};

class Planner
{
private:
	StepperCtrl *ptrStepperCtrl;
	volatile PlannerMode currentMode = PlannerMode::NONE;
	//todo we should not use floating point, rather use "Angle"
	volatile float endAngle;
	volatile float startAngle;
	volatile float currentSetAngle;
	volatile float tickIncrement;

public:
	void begin(StepperCtrl *ptrStepper);
	bool moveConstantVelocity(float finalAngle, float rpm);  //moves to the final location at a constant RPM
	void tick(void);  //this is called on regular tick interval
	void stop(void);
	bool done(void) {return currentMode == PlannerMode::NONE;}
};

extern Planner SmartPlanner;


#endif /* PLANNER_H_ */
