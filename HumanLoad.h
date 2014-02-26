#include "WPILib.h"
#include "CitrusButton.h"

//TODO incorporate into intake system to be able to use RunSecondaryRollers()

void HPReceive(Talon *secondaryRollerA, Talon *secondaryRollerB,
		IntakeSystem* front, IntakeSystem* back)
{	
		front->RunSecondaryRollers();
		front->Reverse();
		back->Reverse();
}
