#include "WPILib.h"
#include <math.h>

float oldStickLeftInput = 0.0;
float oldStickRightInput = 0.0;
float useleft = 0.0;
float useright = 0.0;
float leftchange = 0.0;
float rightchange = 0.0;

#define kDeadzoneThreshold 0.04 //kDeadzoneThreshold = 0.02;
//#define kJoystickChangeThreshold 0.2

void driveTrainValues(float stickLeftInput, float stickRightInput, float kJoystickChangeThreshold) {
	
	/*
	 * This function prevents sudden acceleration of the robot, by placing a cap on the amount that the 
	 * robot's speed can increase or decrease each time a new joystick value is received.
	 * This prevents really large and fast movements of the joystick from translating immediately into
	 * really large acceleration.
	 */
	
	// Assign right joystick value to the variable 'right'
	
	// Find the difference in the last two joystick values received
	rightchange=fabs(oldStickRightInput-stickRightInput);

	// If the change in joystick values is less than 0.2 then use the new right value
	// The change in the joystick value wasn't too sudden...
	// so the driver probably intended for the robot to move at this new speed
	if (rightchange<=kJoystickChangeThreshold) {
		useright=stickRightInput;
	}

	// If the change in the joystick values is dramatic (AKA the joystick was moved really fast)
	// Then, make the change in the speed of the robot less sudden by only allowing the robot to speed up
	// or slow down by a kThreshold value of 0.2 
	else {

		// If the new right value is greater than the old one use the old value plus the kThreshold 
		// (because the driver intended the robot to speed up) 
		// Assign this speed as the speed that will be sent to the drivetrain
		if (oldStickRightInput<stickRightInput) {
			useright=oldStickRightInput+kJoystickChangeThreshold;
		}

		// If the new right is less than the old right use the old value minus the kThreshold 
		// (because the driver intended the robot to slow down)
		// Assign this speed as the speed that will be sent to the drivetrain
		if (oldStickRightInput>stickRightInput) {
			useright=oldStickRightInput-kJoystickChangeThreshold;
		}
	}

	// Do this same process for the left drivetrain values
	leftchange=fabs(oldStickLeftInput-stickLeftInput);

	if (leftchange<=kJoystickChangeThreshold) {
		useleft=stickLeftInput;
	} else {
		if (oldStickLeftInput<stickLeftInput) {
			useleft=oldStickLeftInput+kJoystickChangeThreshold;
		}
		if (oldStickLeftInput>stickLeftInput) {
			useleft=oldStickLeftInput-kJoystickChangeThreshold;
		}
	}

	// Make useright and useleft (the values sent to the robot) the old values for the next loop
	oldStickRightInput=useright;
	oldStickLeftInput=useleft;

	//drivetrain->TankDrive(useleft * useleft * useleft, useright	* useright * useright);
	// useleft and useright are sent to the drivetrain in the TeleopPeriodic void
	
}

void deadzone() {
	if ((useleft >= -kDeadzoneThreshold) && (useleft <= kDeadzoneThreshold)) {
		useleft = 0;
	}
	if ((useright >= -kDeadzoneThreshold) && (useright
			<= kDeadzoneThreshold)) {
		useright = 0;
	}
}

void runDrivetrain(float stickLeftInput, float stickRightInput, RobotDrive *drivetrain, float thresh)
{
	// Drivetrain code
		
	driveTrainValues(stickLeftInput, stickRightInput, thresh);
	deadzone();
	//printf("left: %f, right; %f\n", useleft, useright);
	
	drivetrain->TankDrive(useleft, useright); // Inverted and negated the values to make the back of the robot the "front"
	//drivetrain->TankDrive(-useright, -useleft); // Inverted and negated the values to make the back of the robot the "front"
}

void runDrivetrain(float stickLeftInput, float stickRightInput, RobotDrive *drivetrain)
{
	runDrivetrain(stickLeftInput, stickRightInput, drivetrain, 0.2);
}


//shifting
void shiftGears(Solenoid* shiftUp, Solenoid* shiftDown, Joystick *joystickUp, int buttonUp, Joystick *joystickDown, int buttonDown) //up = true means shift to high gear; false to low gear
{
	if(joystickUp->GetRawButton(buttonUp))
	{
		shiftUp->Set(true);
		shiftDown->Set(false);
	}
	else if (joystickDown->GetRawButton(buttonDown))
	{
		shiftUp->Set(false);
		shiftDown->Set(true);
	}
}
