#include "WPILib.h"

#ifndef SECONDARYROLLERSYSTEM_H
#define SECONDARYROLLERSYSTEM_H //include protection

class SecondaryRollerSystem
{
public:
	
	Talon *secondaryRollerA;
	Talon *secondaryRollerB;
	Solenoid *armPiston;
	Timer *pulseTimer;
	bool armsDown;
	
	SecondaryRollerSystem(Talon *tsecondaryRollerA, Talon *tsecondaryRollerB, Solenoid *tarmPiston)
	{
		
		secondaryRollerA = tsecondaryRollerA;
		secondaryRollerB = tsecondaryRollerB;
		armPiston = tarmPiston;
		armsDown = false;
		pulseTimer = new Timer();
		pulseTimer->Reset();
		pulseTimer->Start();
	}
	
	void Run()
	{
		secondaryRollerA->Set(-1.0);
		secondaryRollerB->Set(1.0);
	}
	
	void Reverse()
	{
		secondaryRollerA->Set(1.0);
		secondaryRollerB->Set(-1.0);
	}
	
	void Stop()
	{
		secondaryRollerA->Set(0.0);
		secondaryRollerB->Set(0.0);
	}
	
	void Pulse()
	{
		float time = pulseTimer->Get();
		if(time <0.1)
		{
			Run();
		}
		else
		{
			Stop();
			if(time >= 0.2)
			{
				pulseTimer->Reset();
			}
		}
	}
	
	void Deploy()
	{
		armPiston->Set(true);
		armsDown = true;
	}
	
	void Undeploy()
	{
		armPiston->Set(false);
		armsDown = (false);
	}
	
	void ToggleArms()
	{
		armsDown = !armsDown;
		armPiston->Set(armsDown);
	}
	
	bool DeployState()
	{
		return armsDown;
	}
	
};

#endif //SECONDARYROLLERSYSTEM_H
