#include "WPILib.h"
#include "CitrusButton.h"

void HPReceive(CitrusButton *button, Solenoid *armPiston, Talon *secondaryRoller)
{
	if(button->ButtonClicked())
	{
		armPiston->Set(true); //TODO direction
	}
	if(button->ButtonPressed())
	{
		secondaryRoller->Set(1.0); //TODO direction
	}
	if(button->ButtonReleased())
	{
		armPiston->Set(false); //TODO direction; also can you get away with leaving
		//them open?
		secondaryRoller->Set(0.0);
	}
}
