#include "WPILib.h"

#ifndef SECONDARYROLLERSYSTEM_H
#define SECONDARYROLLERSYSTEM_H //include protection

//Two small rollers on the right and left sides of the robot. Used to settle the ball before shooting.

class SecondaryRollerSystem
{
public:
	
	Talon *secondaryRollerA; 
	Talon *secondaryRollerB;
	Solenoid *armPiston; //They move in and out, out to not bump shot, in when settling the ball.
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
		secondaryRollerA->Set(-1.0 * rollerK); //They need to run in opposite directions.
		secondaryRollerB->Set(1.0 * rollerK);
	}
	
	void RunAt(float x)
	{
		secondaryRollerA->Set(-x); //value between -1.0 and 1.0
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
	}
	
	void Reset()
	{
		armsDown = false;
		secondaryRollerA->Set(0.0);
		secondaryRollerB->Set(0.0);
		pulseTimer->Reset();
	}
	
	void Pulse() //More effective at settling the ball than running alone.
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
	
	void ReversePulse()
	{
		float time = pulseTimer->Get();
		if(time < 0.15)
		{
			Reverse();
		}
		else
		{
			Stop();
			if(time >= 0.3)
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
	
	void Deploy() //put out
	{
		armPiston->Set(true); 
		armsDown = true;
	}
	
	void Undeploy() //pull in
	{
		armPiston->Set(false);
		armsDown = (false);
	}
	
	void ToggleArms() //toggle deployed state
	{
		armsDown = !armsDown;
		armPiston->Set(armsDown);
	}
	
	bool DeployState() //use to find out if they are deployed
	{
		return armsDown;
	}
	
};

#endif //SECONDARYROLLERSYSTEM_H
