#include "WPILib.h"
#include "CitrusButton.h"
#include "SecondaryRollerSystem.h"

//TODO incorporate into intake system to be able to use RunSecondaryRollers()

void HPReceive(SecondaryRollerSystem *secondaryRollers,
		IntakeSystem* front, IntakeSystem* back)
{	
		secondaryRollers->Run();
		front->Reverse();
		back->Reverse();
}
