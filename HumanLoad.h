#include "WPILib.h"
#include "CitrusButton.h"
#include "SecondaryRollerSystem.h"
#include "ShooterSystem.h"

//TODO incorporate into intake system to be able to use RunSecondaryRollers()

#ifndef HUMANLOAD_H
#define HUMANLOAD_H

void HPReceive(SecondaryRollerSystem *secondaryRollers,
		IntakeSystem* front, IntakeSystem* back, ShooterSystem *shooter)
{
		shooter->ShooterPrime(false);
		secondaryRollers->Run();
		front->Reverse();
		back->Reverse();
}

void HPReverse(SecondaryRollerSystem *secondaryRollers, 
		IntakeSystem *frontIntake, IntakeSystem *backIntake, ShooterSystem *shooter)
{
	shooter->ShooterPrime(false);
	secondaryRollers->ReversePulse();
	frontIntake->FrontRollerLoad();
	backIntake->BackRollerSlow();
	
}

#endif
