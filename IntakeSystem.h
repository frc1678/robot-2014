#include "WPILib.h"
#ifndef INTAKESYSTEM_H
#define INTAKESYSTEM_H //include protection

class IntakeSystem
{
public:
	/*
	 bool ProximityTriggered();
	 void BackRollerInitialPickup();
	 void FrontRollerInitialPickup();
	 void FrontRollerHold();
	 void BackRollerHold();
	 */
	Talon *intakeRoller;
	DigitalInput *intakeSensor;
	Solenoid *intakeUp;
	Talon *secondaryIntakeRollerA;
	Talon *secondaryIntakeRollerB;

	//booleans and such for void Pickup()
	bool readyToPickup;
	bool sensorTriggered;
	bool front;
	Timer *pickupTimer;
	
	//boolean for is intake deployed
	bool intakeDeployed;
	
	IntakeSystem(int rollerTalonPort, Talon *tsecondaryIntakeRollerA, 
			Talon *tsecondaryIntakeRollerB, int sensorPort, Solenoid *tIntakeUp, bool front)
	{
		intakeRoller = new Talon(rollerTalonPort);
		secondaryIntakeRollerA = tsecondaryIntakeRollerA;
		secondaryIntakeRollerB = tsecondaryIntakeRollerB;
		intakeSensor = new DigitalInput(sensorPort);
		intakeUp = tIntakeUp;

		front = true; 
		readyToPickup = true;
		sensorTriggered = false;
		pickupTimer = new Timer();
		intakeDeployed = false;
	}
	//TODO add secondary rollers to methods. Have some internal ones
	//then add to Pickup.
	void Reverse() //Only call inside an if statement.
	{
		//TODO When called with HumanLoad, front intake runs in pickup since 
		//roller motors are set backward
		intakeRoller->Set(-1.0); 
	}
	void Stop()
	{
		intakeRoller->Set(0.0);
		sensorTriggered = false;
		readyToPickup = true;
		pickupTimer->Stop();
		pickupTimer->Reset();
	}
	
	//USE THESE TWO WHENEVER YOU WANT TO DEPLOY THE INTAKES.
	void DeployIntake ()
	{
		intakeDeployed = true;
		intakeUp->Set(true);
	}
	void UndeployIntake()
	{
		intakeDeployed = false;
		intakeUp->Set(false);
	}
	void ToggleIntake()
	{
		intakeDeployed = !intakeDeployed;
		intakeUp->Set(intakeDeployed);
	}
	
	bool DeployState()
	{
		return intakeDeployed;
	}

	//Internal: front roller grabbing onto ball, front roller holding ball, back roller grabbing onto ball, back roller holding ball, front roller reverse, back roller reverse
	void FrontRollerLoad()
	{
		intakeRoller->Set(1.0); //TODO numbers
		//intakeRoller->Set(stick->GetTwist());
	}

	void BackRollerLoad()
	{
		intakeRoller->Set(1.0); //TODO numbers. Flipping for Fembots
		//intakeRoller->Set(stick->GetY());
	}

	void BackBumperHold() 
	{
		intakeRoller->Set(0.0); //TODO numbers
	}
	
	void FrontBumperHold()
	{
		intakeRoller->Set(0.0); //TODO numbers.
	}
	
	void FrontRollerAutoSlow()
	{
		intakeRoller->Set(0.1);
	}
	void BackRollerAutoSlow()
	{
		intakeRoller->Set(0.1);
	}
	
	bool ProximityTriggered()
	{
		if (intakeSensor->Get() == 1)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	
	void FrontRollerSlow(DriverStation *m_ds)
	{
		intakeRoller->Set(0.5);
	}
	void BackRollerSlow()
	{
		intakeRoller->Set(0.5);
	}
	

	void Hold()
	{
		if (!ProximityTriggered())
		{
			if (front)
			{
				FrontRollerLoad();
			}
			else
			{
				BackRollerLoad();
			}
		}
		else
		{
			printf("Proxy!\n");
			if(front)
			{
				FrontBumperHold();
			}
			else
			{
				BackBumperHold();
			}
		}
	}
	void Pickup(Joystick * stick, DriverStation *m_ds)
	{
		if(ProximityTriggered())
		{
			printf("sensor");
			sensorTriggered = true;
		}
		if(!sensorTriggered)
		{
			printf("running");
			BackRollerLoad();
		}
		else
		{
			pickupTimer->Start();
			if(pickupTimer->Get() < m_ds->GetAnalogIn(1))
			{
				BackRollerLoad();
			}
			else
			{
				if(intakeDeployed)
				{
					printf("undeploy");
					UndeployIntake();
				}
				printf("slow");
				BackRollerSlow();
			}
		}
	}
	//TODO make them in the same function.
	void FrontPickup(Joystick * stick, DriverStation *m_ds)
	{
		if(ProximityTriggered())
		{
			printf("sensor");
			sensorTriggered = true;
		}
		if(!sensorTriggered)
		{
			printf("running");
			FrontRollerLoad(); 
		}
		else
		{
			pickupTimer->Start();
			if(pickupTimer->Get() < m_ds->GetAnalogIn(2))
			{
				FrontRollerLoad();
			}
			else
			{
				if(intakeDeployed)
				{
					printf("undeploy");
					UndeployIntake();
				}
				printf("slow");
				FrontRollerSlow(m_ds);
			}
		}
	}
	
};

#endif
