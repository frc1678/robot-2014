#include "WPILib.h"
#include "CitrusButton.h"
#include "SecondaryRollerSystem.h"

//TODO incorporate into intake system to be able to use RunSecondaryRollers()

#ifndef HUMANLOAD_H
#define HUMANLOAD_H

void HPReceive(SecondaryRollerSystem *secondaryRollers,
		IntakeSystem* front, IntakeSystem* back)
{	
		secondaryRollers->Run();
		front->Reverse();
		back->Reverse();
}

#endif
