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
	float rollerK;
	
	SecondaryRollerSystem(Talon *tsecondaryRollerA, Talon *tsecondaryRollerB, Solenoid *tarmPiston)
	{
		
		secondaryRollerA = tsecondaryRollerA;
		secondaryRollerB = tsecondaryRollerB;
		armPiston = tarmPiston;
		armsDown = false;
		pulseTimer = new Timer();
		pulseTimer->Reset();
		pulseTimer->Start();
		rollerK = 1.0;
	}
	
	void Run()
	{
		secondaryRollerA->Set(-1.0 * rollerK);
		secondaryRollerB->Set(1.0 * rollerK);
	}
	
	void RunAt(float x)
	{
		secondaryRollerA->Set(-x);
		secondaryRollerB->Set(x);
	}
	
	void Reverse()
	{
		secondaryRollerA->Set(1.0 * rollerK);
		secondaryRollerB->Set(-1.0 * rollerK);
	}
	
	void ReverseSlow()
	{
		secondaryRollerA->Set(0.5 * rollerK);
		secondaryRollerB->Set(-0.5 * rollerK);
	}
	
	void Stop()
	{
		secondaryRollerA->Set(0.0);
		secondaryRollerB->Set(0.0);
		//pulseTimer->Stop();
	}
	
	void Reset()
	{
		armsDown = false;
		secondaryRollerA->Set(0.0);
		secondaryRollerB->Set(0.0);
		pulseTimer->Reset();
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
	
	void RunSlow()
	{
		secondaryRollerA->Set(-0.4 * rollerK);
		secondaryRollerB->Set(0.4 * rollerK);
	}
	
	void PulseSlow()
	{
		float time = pulseTimer->Get();
		if(time <0.1)
		{
			RunSlow();
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
