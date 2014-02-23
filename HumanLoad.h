#include "WPILib.h"
#include "CitrusButton.h"

//TODO incorporate into intake system to be able to use RunSecondaryRollers()

void HPReceive(CitrusButton *button, Solenoid *armPiston, Talon *secondaryRollerA, Talon *secondaryRollerB)
{
	if(button->ButtonClicked())
	{
		armPiston->Set(true); //TODO direction
	}
	if(button->ButtonPressed())
	{
		secondaryRollerA->Set(1.0); //TODO direction
		secondaryRollerB->Set(-1.0);
	}
	if(button->ButtonReleased())
	{
		armPiston->Set(false); //TODO direction; also can you get away with leaving
		//them open?
		secondaryRollerA->Set(0.0);
		secondaryRollerB->Set(0.0);
	}
}
